#include "GuiRefTruth.h"

#include <atomic>
#include <cstdio>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <filesystem>

#include "core/PathMemory.h"
#include "core/SinexReader.h"
#include "download/ProductMeta.h"
#include "download/Store.h"
#include "download/Transfer.h"
#include "solvers/common/Task.h"
#include "time/TimeConvert.h"
#include "ui/GuiColors.h"
#include "core/Const.h"
#include "util/StringUtils.h"

namespace Solvers::RefTruth {

    namespace {
        // 一次「获取」的结果
        struct FetchResult {
            bool ok = false;
            Eigen::Vector3d xyz{0, 0, 0};
            std::string station;   // 4 字符站点名（大写）
            std::string file;      // 用到的 SINEX 文件名
            std::string err;       // 失败原因（station 为空时即「未知站点」）
        };

        FetchResult fetchStationXYZ(const std::string &obsPath) {
            FetchResult r;
            std::string station;
            int year = 0, doy = 0;
            if (!SinexReader::obsStationAndDoy(obsPath, station, year, doy)) {
                return r;                                  // 文件名 <4 字符 → 无站点信息（station 空）
            }
            r.station = station;
            if (doy <= 0) {
                r.err = "无法确定观测日期（文件名/头均无年积日），无法定位 WHU 坐标文件";
                return r;
            }

            const int week = ProductMeta::GPSWeek(year, doy);
            const CommonTime wkStart = CommonTime(GPS_EPOCH_MJD, 0.0, TimeSystem::GPS)
                                     + static_cast<double>(week) * 7.0 * SEC_PER_DAY;
            const YDSTime yds = Time::CommonToYDS(wkStart);
            char fname[128];
            std::snprintf(fname, sizeof(fname),
                          "IGS0OPSSNX_%04d%03d0000_07D_07D_CRD.SNX.gz", yds.year, yds.doy);
            r.file = fname;
            char fnameSnx[128];
            std::snprintf(fnameSnx, sizeof(fnameSnx),
                          "IGS0OPSSNX_%04d%03d0000_07D_07D_SNX.SNX.gz", yds.year, yds.doy);
            const std::string base = std::string(ProductMeta::mirrorRoot()) + "/products/"
                                    + std::to_string(week) + "/";

            const std::string tmp = Download::Store::tempPath("ref_truth", fname);
            Download::Store::ensureDir("ref_truth");
            struct TmpCleaner { std::string p; ~TmpCleaner() { std::error_code e; std::filesystem::remove(p, e); } };
            TmpCleaner _cleaner{tmp};
            const char *cands[2] = {fname, fnameSnx};
            bool got = false;
            std::string dlErr;
            for (const char *fn: cands) {
                const std::string url = base + fn;
                dlErr.clear();
                if (Download::Transfer::downloadFile(
                        url, tmp, nullptr,
                        [&](const std::string &m) { dlErr = m; }, 3, 120, nullptr)) {
                    got = true;
                    if (fn != fname) r.file = fn;
                    break;
                }
            }
            if (!got) {
                r.err = "下载失败: " + base + fname + (dlErr.empty() ? "" : " (" + dlErr + ")");
                return r;
            }

            SinexReader reader;
            if (!reader.readFile(tmp, r.err)) return r;                  // r.err 已由 readFile 写入
            if (!reader.getStation(station, year, doy, r.xyz)) return r; // station 已填，ok=false → 未包含该站
            r.ok = true;
            return r;
        }

        void startRefFetch(Task *task) {
            if (task->refFetching.load()) return;
            task->refFetching = true;
            {
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchStatus = "联网获取中…";
                task->refFetchStation.clear();
                task->refFetchFile.clear();
            }
            const std::string obs = task->view.obsPathBuf;
            std::shared_ptr<Task> keep = task->shared_from_this();
            task->refFetchThread = std::thread([keep, obs] {
                Task *t = keep.get();
                const auto [ok, xyz, station, file, err] = fetchStationXYZ(obs);
                std::lock_guard lk(t->refFetchMtx);
                t->refFetching = false;
                t->refFetchStation = station;
                if (ok) {
                    t->refXyz = xyz;
                    t->refFetchFile = file;
                    t->refFetchStatus = "获取到 " + file + " 文件中 " + station + " 站点的真值数据";
                } else if (station.empty()) {
                    t->refFetchStatus = "未知站点";
                } else {
                    t->refFetchStatus = err.empty()
                                            ? "WHU 坐标文件（" + file + "）未包含站点 " + station
                                            : err;
                }
            });
            task->refFetchThread.detach();
        }
    }

    void renderRefTruthField(Task *task) {
        ImGui::Checkbox("参考真值 (ECEF/m)", &task->useRefTruth);
        if (!task->useRefTruth) return;

        Eigen::Vector3d disp;
        std::string status;
        bool fetching = false;
        {
            std::lock_guard<std::mutex> lk(task->refFetchMtx);
            disp = task->refXyz;
            status = task->refFetchStatus;
            fetching = task->refFetching.load();
        }

        ImGui::PushItemWidth(170);
        bool changed = false;
        if (ImGui::InputDouble("X##ref_x", &disp.x())) changed = true;
        ImGui::SameLine();
        if (ImGui::InputDouble("Y##ref_y", &disp.y())) changed = true;
        ImGui::SameLine();
        if (ImGui::InputDouble("Z##ref_z", &disp.z())) changed = true;
        ImGui::PopItemWidth();
        if (changed) {
            std::lock_guard lk(task->refFetchMtx);
            task->refXyz = disp;
        }

        ImGui::SameLine();
        if (ImGui::Button("从剪贴板获取##ref_clip")) {
            if (const char *c = ImGui::GetClipboardText()) {
                double val[3] = {0, 0, 0};
                const int cnt = Str::scanDoubles(c, val, 3);
                if (cnt >= 1) disp.x() = val[0];
                if (cnt >= 2) disp.y() = val[1];
                if (cnt >= 3) disp.z() = val[2];
                std::lock_guard lk(task->refFetchMtx);
                task->refXyz = disp;
            }
        }

        ImGui::SameLine();
        {
            ImGui::BeginDisabled(fetching);
            if (ImGui::Button(fetching ? "联网获取中…" : "联网获取##ref_fetch")) {
                startRefFetch(task);
            }
            ImGui::EndDisabled();
        }

        if (!status.empty()) {
            ImVec4 col;
            if (status.rfind("获取到", 0) == 0) col = UI::kColPass;
            else if (status == "未知站点") col = UI::kColWarn;
            else col = UI::kColFail;
            ImGui::TextColored(col, "%s", status.c_str());
        }
    }

    void persist(Task *task, const char *iniPrefix) {
        const auto &pm = AppConfig::PathMemory::instance();
        pm.set(std::string(iniPrefix) + "ref_truth_enable", task->useRefTruth ? "1" : "0");
        Eigen::Vector3d xyz;
        {
            std::lock_guard lk(task->refFetchMtx);
            xyz = task->refXyz;
        }
        std::ostringstream ss;
        // 全精度写出（double 可无损往返需 17 位有效数字），否则默认 6 位有效数字会把 ECEF
        // 坐标截断到米级误差（如 2919785.8419 → 2.91979e+06）。
        ss << std::setprecision(17) << xyz.x() << ',' << xyz.y() << ',' << xyz.z();
        pm.set(std::string(iniPrefix) + "ref_truth_xyz", ss.str());
    }

    void restore(Task *task, const char *iniPrefix) {
        const auto &pm = AppConfig::PathMemory::instance();
        if (const std::string e = pm.get(std::string(iniPrefix) + "ref_truth_enable");
            !e.empty() && e != "0")
            task->useRefTruth = true;
        if (const std::string xyz = pm.get(std::string(iniPrefix) + "ref_truth_xyz"); !xyz.empty()) {
            const auto parts = Str::split(xyz, ',', false);
            double v[3] = {0, 0, 0};
            bool ok = parts.size() == 3;
            for (std::size_t i = 0; ok && i < 3; ++i) {
                const char *cur = parts[i].c_str();
                ok = Str::nextDouble(cur, cur + parts[i].size(), v[i]);
            }
            if (ok) task->refXyz = Eigen::Vector3d(v[0], v[1], v[2]);
        }
    }
}
