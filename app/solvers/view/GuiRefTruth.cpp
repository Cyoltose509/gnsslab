#include "GuiRefTruth.h"
#include "FileUtils.h"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "core/Const.h"
#include "core/PathMemory.h"
#include "core/SiteTruth.h"
#include "io/SinexReader.h"
#include "download/local/ProductType.h"
#include "io/ObsDate.h"
#include "download/remote/SourceTable.h"
#include "download/remote/Fetcher.h"
#include "download/local/Store.h"
#include "download/remote/http/Core.h"
#include "download/remote/Catalog.h"
#include "download/ui/Widgets.h"
#include "imgui.h"
#include "io/ProductReader.h"
#include "solvers/common/Task.h"
#include "solvers/view/GuiHelpers.h"
#include "ui/FileDialog.h"
#include "ui/GuiColors.h"
#include "StringUtils.h"
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

namespace Solvers::RefTruth {
    namespace {
        std::string keyOf(const Mode m, const char *suffix) {
            return std::string(modeKey(m)) + suffix;
        }

        std::string SP3Key(const Mode m) { return keyOf(m, "_ref_sp3"); }

        /// 状态行分色
        ImVec4 statusColor(const Task::RefStatusLevel lv) {
            switch (lv) {
                case Task::RefStatusLevel::Pass: return UI::kColPass;
                case Task::RefStatusLevel::Warn: return UI::kColWarn;
                default: return UI::kColFail;
            }
        }

        /// 两处联网取真值共用的进度适配：HTTP 回调
        Download::Http::ProgressCb refProgressCb(Task *task, const bool checkCancel) {
            return [task, checkCancel](const double total, const double now) {
                if (checkCancel && task->refFetchCancel.load()) return;
                const float fr = Download::Http::fracOf(total, now);
                task->refFetchPercent = fr < 0.0f ? -1 : static_cast<int>(fr * 100.0f);
                task->refFetchTotal = total > 0.0 ? total : -1.0;
                task->refFetchGot = now;
            };
        }

        /// 观测头 MARKER NAME 里的星号（COSMIC 写成 `C2E2 L77 (POD1)`）：参考轨道按它配对。
        std::string obsSatToken(const std::string &obsPath) {
            std::ifstream in(obsPath);
            if (!in) return {};
            std::string line;
            for (int i = 0; i < 120 && std::getline(in, line); ++i) {
                if (line.find("MARKER NAME") == std::string::npos) continue;
                for (const auto &tok: Str::split(line, ' ', false)) {
                    const bool digits = tok.size() > 1 &&
                                        std::all_of(tok.begin() + 1, tok.end(),
                                                    [](const unsigned char c) { return std::isdigit(c) != 0; });
                    if (tok[0] == 'L' && digits) return tok;
                }
                break;
            }
            return {};
        }

        std::string bestTruth(const std::string &obsPath, const std::vector<std::string> &have,
                              const int year, const int doy) {
            if (have.empty()) return {};
            const std::string mission = Download::SourceTable::LEOMission(File::baseName(obsPath));
            const std::string want = obsSatToken(obsPath);
            if (mission == "COSMIC-2") {
                if (want.empty()) return have.size() == 1 ? have.front() : std::string{};
                const std::string exact = "leoOrb_" + std::to_string(year) + "_" +
                                          std::to_string(doy) + "_" + want + ".sp3";
                for (const auto &f: have)
                    if (File::baseName(f) == exact) return f;
                return {};
            }
            const char sat = Download::SourceTable::LEOSat(obsPath);
            std::vector<std::string> cand;
            for (const auto &f: have)
                if (Download::SourceTable::orbitIsFor(mission, sat, f)) cand.push_back(f);
            if (mission == "Swarm") return cand.empty() ? std::string{} : cand.front();
            if (!cand.empty()) return cand.front();
            return have.size() == 1 ? have.front() : std::string{};
        }

        /// 恒值坐标行的联网获取
        void startXYZFetch(Task *task) {
            if (task->refFetching.load()) {
                // 下载中再点 = 暂停
                task->refFetchCancel = true;
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchStatus = I18n::trc("ref.pausing");
                task->refFetchLevel = Task::RefStatusLevel::Warn;
                return;
            }
            if (task->refFetchThread.joinable()) task->refFetchThread.join();
            task->refFetching = true;
            task->refFetchCancel = false;
            task->refFetchPercent = -1;
            task->refFetchTotal = -1.0;
            task->refFetchGot = 0.0;
            {
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchStatus.clear();
                task->refFetchCurrent.clear();
                task->refFetchPending = {};
            }
            const std::string obs = task->view.obsPathBuf;
            const Eigen::Vector3d prevXyz = task->truth.fixed;
            task->refFetchThread = std::thread([task, obs, prevXyz] {
                std::string status, station;
                Task::RefStatusLevel level = Task::RefStatusLevel::Fail;
                bool got = false;
                Eigen::Vector3d xyz{0, 0, 0};
                int year = 0, doy = 0;
                if (!SinexReader::obsStationAndDOY(obs, station, year, doy)) {
                    status = I18n::trc("ref.unknown_station");
                    level = Task::RefStatusLevel::Warn;
                } else if (doy <= 0) {
                    status = I18n::trc("ref.no_date");
                } else {
                    {
                        std::lock_guard lk(task->refFetchMtx);
                        task->refFetchCurrent = I18n::tr("ref.fetch.sinex_station", station);
                    }
                    double v[3]{0, 0, 0};
                    bool cached = false;
                    std::string file;
                    const Download::Http::ProgressCb cb = refProgressCb(task, true);
                    const bool ok = SiteTruth::fetcher()
                                        ? SiteTruth::fetcher()(station, year, doy, v, cached, file, cb)
                                        : Download::Catalog::ensureSiteTruth(station, year, doy, v, cached, file, cb);
                    if (ok) {
                        xyz = Eigen::Vector3d(v[0], v[1], v[2]);
                        got = true;
                        // 缓存命中也带上站点名，跨期复用时一眼可辨
                        status = cached ? I18n::tr("ref.fetch.cached_station", station) : file;
                        level = Task::RefStatusLevel::Pass;
                    } else {
                        status = file.empty() ? I18n::trc("ref.fetch.failed") : file;
                    }
                }
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchPercent = -1;
                task->refFetchTotal = -1.0;
                task->refFetchGot = 0.0;
                task->refFetchCurrent.clear();
                if (task->refFetchCancel.load()) {
                    task->refFetchStatus = I18n::trc("ref.paused");
                    task->refFetchLevel = Task::RefStatusLevel::Warn;
                } else {
                    if (got) {
                        Task::RefFetchResult res;
                        res.ok = true;
                        res.form = RefForm::Fixed;
                        res.xyz = xyz;
                        res.obsPath = obs;
                        res.prevXyz = prevXyz;
                        task->refFetchPending = res;
                    }
                    task->refFetchStatus = status;
                    task->refFetchLevel = level;
                }
                task->refFetching = false; // 最后一步：界面此后才认为本轮结束
            });
        }
        /// SP3 文件行的联网获取
        void startSP3Fetch(Task *task) {
            if (task->refFetching.load()) {
                // 下载中再点 = 暂停
                task->refFetchCancel = true;
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchStatus = I18n::trc("ref.pausing");
                task->refFetchLevel = Task::RefStatusLevel::Warn;
                return;
            }
            if (task->refFetchThread.joinable()) task->refFetchThread.join();
            task->refFetching = true;
            task->refFetchCancel = false;
            task->refFetchPercent = -1;
            task->refFetchTotal = -1.0;
            task->refFetchGot = 0.0;
            {
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchStatus.clear();
                task->refFetchCurrent.clear();
                task->refFetchPending = {};
            }
            const std::string obs = task->view.obsPathBuf;
            const std::string prevSp3 = task->truth.sp3Path;
            task->refFetchThread = std::thread([task, obs, prevSp3] {
                const std::string name = File::baseName(obs);
                const std::string mission = Download::SourceTable::LEOMission(name);
                std::string status;
                Task::RefStatusLevel level = Task::RefStatusLevel::Fail;
                std::string path;
                const auto date = parseObsDate(obs);
                if (!date) {
                    status = I18n::trc("ref.no_date");
                } else {
                    const int year = date->year;
                    const int doy = date->doy;
                    const auto have = Download::Store::truthFiles(year, doy);
                    const std::string local = bestTruth(obs, have, year, doy);
                    if (!local.empty()) {
                        status = I18n::tr("common.local_exist", File::baseName(local));
                        level = Task::RefStatusLevel::Pass;
                        path = local;
                    } else if (mission.empty()) {
                        status = I18n::trc("ref.leo.unknown_object");
                    } else if (const auto files = Download::Fetcher::resolve(
                                   Download::SourceTable::LEOTruths(name, year, doy), year, doy); files.empty()) {
                        status = I18n::tr("ref.orbit.no_source", mission);
                    } else {
                        const std::string dir = Download::Store::dateDir(ProductType::LEO_SP3, year, doy);
                        std::error_code ec;
                        std::filesystem::create_directories(dir, ec);
                        bool anyOk = false;
                        for (const auto &url: files) {
                            if (task->refFetchCancel.load()) break;
                            // 落盘、解包、合并、记清单全走下载器那一份实现（与「产品下载」窗口同一条路径）
                            Download::Fetcher::Pending p;
                            p.remoteName = Download::Http::nameOfUrl(url);
                            p.url = url;
                            p.type = ProductType::LEO_SP3;
                            p.year = year;
                            p.doy = doy;
                            {
                                std::lock_guard lk(task->refFetchMtx);
                                task->refFetchCurrent = p.remoteName;
                                task->refFetchPercent = -1;
                            }
                            const auto onProgress = refProgressCb(task, false);
                            const auto onUnpack = [task](const double f) {
                                task->refFetchPercent = static_cast<int>(f * 100.0);
                                task->refFetchTotal = -1.0;
                                task->refFetchGot = 0.0;
                            };
                            const auto onStage = [task, remote = p.remoteName](const std::string &s) {
                                std::lock_guard lk(task->refFetchMtx);
                                task->refFetchCurrent = s.empty() ? remote : s + " " + remote;
                            };
                            const auto r = Download::Fetcher::fetchNow(p, onProgress, &task->refFetchCancel,
                                                                       onUnpack, onStage);
                            if (r.paused) break;
                            if (r.ok) anyOk = true;
                        }
                        if (task->refFetchCancel.load()) {
                            status = I18n::trc("ref.paused");
                            level = Task::RefStatusLevel::Warn;
                        } else if (anyOk) {
                            // 只有这轮确实下回来东西，本机清单才可能变化，才值得重扫一遍；
                            // 全部失败时清单与开头那次相同，直接落到「获取失败」。
                            const auto now = Download::Store::truthFiles(year, doy);
                            if (const std::string use = bestTruth(obs, now, year, doy);
                                !use.empty()) {
                                status = I18n::tr("ref.fetch.got", File::baseName(use));
                                level = Task::RefStatusLevel::Pass;
                                path = use;
                            } else {
                                status = I18n::tr("ref.orbit.not_found", mission);
                            }
                        } else {
                            status = I18n::tr("ref.fetch.failed_at", Download::Http::nameOfUrl(files.front()));
                        }
                    }
                }
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchPercent = -1;
                task->refFetchTotal = -1.0;
                task->refFetchGot = 0.0;
                task->refFetchCurrent.clear();
                task->refFetchStatus = status;
                task->refFetchLevel = level;
                if (!path.empty() && !task->refFetchCancel.load()) {
                    Task::RefFetchResult res;
                    res.ok = true;
                    res.form = RefForm::File;
                    res.sp3Path = path;
                    res.obsPath = obs;
                    res.prevSp3Path = prevSp3;
                    task->refFetchPending = res;
                }
                task->refFetching = false; // 最后一步：界面此后才认为本轮结束
            });
        }


        /// 界面渲染要的获取状态快照
        struct FetchUIState {
            bool fetching = false;
            int pct = -1;
            double totalBytes = -1.0, gotBytes = 0.0;
            std::string status, current;
            Task::RefStatusLevel level = Task::RefStatusLevel::Fail;
        };

        FetchUIState fetchUIState(Task *task) {
            FetchUIState s;
            std::lock_guard lk(task->refFetchMtx);
            s.fetching = task->refFetching.load();
            s.pct = task->refFetchPercent.load();
            s.totalBytes = task->refFetchTotal.load();
            s.gotBytes = task->refFetchGot.load();
            s.status = task->refFetchStatus;
            s.level = task->refFetchLevel;
            s.current = task->refFetchCurrent;
            return s;
        }

        void renderSP3Row(Task *task) {
            const ImGuiStyle &style = ImGui::GetStyle();
            const auto buttonW = [&style](const char *text) {
                return ImGui::CalcTextSize(text).x + style.FramePadding.x * 2.0f;
            };
            const float reserve = buttonW(I18n::trc("common.browse")) + style.ItemSpacing.x
                                  + std::max(buttonW(I18n::trc("common.fetch")), buttonW(I18n::trc("ref.pause"))) + style.ItemSpacing.x;

            ImGui::Text(I18n::trc("ref.sp3_file"));
            ImGui::PushItemWidth(-reserve);
            const std::string before = task->truth.sp3Path;
            Helpers::inputTextStd("##ref_sp3_path", &task->truth.sp3Path);
            if (ImGui::BeginPopupContextItem("##ref_sp3_ctx")) {
                std::error_code ec;
                const bool has = !task->truth.sp3Path.empty()
                                 && std::filesystem::exists(task->truth.sp3Path, ec);
                if (ImGui::MenuItem(I18n::trc("common.open_explorer"), nullptr, false, has)) {
                    if (!Download::Store::revealFile(task->truth.sp3Path)) {
                        std::lock_guard lk(task->refFetchMtx);
                        task->refFetchStatus = I18n::trc("ref.open_explorer_failed");
                        task->refFetchLevel = Task::RefStatusLevel::Fail;
                    }
                }
                if (ImGui::MenuItem(I18n::trc("common.delete_permanent"), nullptr, false, has))
                    Download::Widgets::askDelete({task->truth.sp3Path});
                ImGui::EndPopup();
            }
            if (before != task->truth.sp3Path)
                AppConfig::PathMemory::instance().set(SP3Key(task->mode), task->truth.sp3Path);
            ImGui::PopItemWidth();

            const auto &pm = AppConfig::PathMemory::instance();
            ImGui::SameLine();
            if (ImGui::Button(UI::idLabel("common.browse", "##ref_sp3"))) {
                if (std::wstring wpath; FileDialog::open(wpath, FileDialog::fSp3())) {
                    task->truth.sp3Path = Str::wideToACP(wpath);
                    pm.set(SP3Key(task->mode), task->truth.sp3Path);
                }
            }

            ImGui::SameLine();
            const auto [fetching, pct, totalBytes, gotBytes,
                status, current, level] = fetchUIState(task);
            if (ImGui::Button(UI::idLabel(fetching ? "ref.pause" : "common.fetch", "##ref_sp3"))) startSP3Fetch(task);
            if (ImGui::IsItemHovered() && !fetching)
                ImGui::SetTooltip(I18n::trc("ref.fetch_sp3_tip"));

            if (fetching)
                Download::Widgets::renderProgress("ref.downloading", current,
                                                  pct >= 0 ? pct / 100.0 : -1.0, gotBytes, totalBytes);
            if (!fetching && !task->truth.sp3Path.empty() && pm.get(SP3Key(task->mode)) != task->truth.sp3Path)
                pm.set(SP3Key(task->mode), task->truth.sp3Path);
            if (!status.empty())
                ImGui::TextColored(statusColor(level), "%s", status.c_str());
            if (Download::Widgets::deleteConfirm()) {
                if (std::error_code ec; !task->truth.sp3Path.empty()
                                        && !std::filesystem::exists(task->truth.sp3Path, ec))
                    task->truth.sp3Path.clear();
            }
        }
        void renderXYZRow(Task *task, bool allowFetch) {
            // truth 只在界面线程改（获取结果也先落 pending 再在这里套用），读它不必加锁。
            Eigen::Vector3d disp = task->truth.fixed;

            ImGui::PushItemWidth(170);
            bool changed = false;
            if (ImGui::InputDouble("X##ref_x", &disp.x())) changed = true;
            ImGui::SameLine();
            if (ImGui::InputDouble("Y##ref_y", &disp.y())) changed = true;
            ImGui::SameLine();
            if (ImGui::InputDouble("Z##ref_z", &disp.z())) changed = true;
            ImGui::PopItemWidth();
            if (changed) task->truth.fixed = disp;

            ImGui::SameLine();
            if (const Helpers::PasteResult p = Helpers::clipboardXYZButton("ref_clip"); p.count > 0) {
                for (int i = 0; i < p.count; i++) disp[i] = p.xyz[i];
                task->truth.fixed = disp;
            }

            if (allowFetch) {
                const auto [fetching, pct, totalBytes, gotBytes,
                    status, current, level] = fetchUIState(task);
                ImGui::SameLine();
                if (ImGui::Button(UI::idLabel(fetching ? "ref.pause" : "ref.fetch_online", "##ref_fetch"))) startXYZFetch(task);
                if (fetching)
                    Download::Widgets::renderProgress("ref.downloading", current,
                                                      pct >= 0 ? pct / 100.0 : -1.0, gotBytes, totalBytes);

                if (!status.empty())
                    ImGui::TextColored(statusColor(level), "%s", status.c_str());
            }
        }
        void applyPending(Task *task) {
            Task::RefFetchResult r;
            {
                std::lock_guard lk(task->refFetchMtx);
                if (!task->refFetchPending.ok) return;
                r = task->refFetchPending;
                task->refFetchPending = {};
            }

            std::string note;
            if (r.obsPath != task->view.obsPathBuf) {
                note = I18n::trc("ref.note.obs_changed");
            } else if (r.form == RefForm::Fixed) {
                if (task->truth.form == RefForm::File) {
                    note = I18n::trc("ref.note.switched_file");
                } else if (task->truth.fixed != r.prevXyz) {
                    note = I18n::trc("ref.note.coord_manual");
                } else {
                    task->truth.fixed = r.xyz;
                    if (task->truth.form == RefForm::None) note = I18n::trc("ref.note.coord_ready");
                }
            } else if (r.form == RefForm::File) {
                if (task->truth.form == RefForm::Fixed) {
                    note = I18n::trc("ref.note.switched_fixed");
                } else if (task->truth.sp3Path != r.prevSp3Path) {
                    note = I18n::trc("ref.note.orbit_manual");
                } else {
                    task->truth.sp3Path = r.sp3Path;
                    AppConfig::PathMemory::instance().set(SP3Key(task->mode), r.sp3Path);
                    if (task->truth.form == RefForm::None) note = I18n::trc("ref.note.orbit_ready");
                }
            }
            if (!note.empty()) {
                std::lock_guard lk(task->refFetchMtx);
                task->refFetchStatus = note;
                task->refFetchLevel = Task::RefStatusLevel::Warn;
            }
        }


    }

    void renderRefTruthSection(Task *task, bool realtime) {
        applyPending(task);

        const bool fixedOk = modeAllowsRefFixed(task->mode);
        const bool fileOk = !realtime && modeAllowsRefFile(task->mode);
        if (!fixedOk && !fileOk) return;

        if (!fileOk && task->truth.form == RefForm::File) task->truth.form = RefForm::Fixed;
        if (!fixedOk && task->truth.form == RefForm::Fixed) task->truth.form = RefForm::None;

        const char *title = fixedOk && fileOk
                                ? I18n::trc("ref.truth")
                                : fixedOk
                                      ? I18n::trc("ref.truth_ecef")
                                      : I18n::trc("ref.truth_file");
        bool use = task->truth.form != RefForm::None;
        if (ImGui::Checkbox(title, &use))
            task->truth.form = use ? (fixedOk ? RefForm::Fixed : RefForm::File) : RefForm::None;

        if (task->truth.form == RefForm::None) return;

        if (fixedOk && fileOk) {
            const bool byFile = task->truth.form == RefForm::File;
            if (ImGui::RadioButton(UI::idLabel("ref.form_fixed", "##ref_form"), !byFile)) task->truth.form = RefForm::Fixed;
            ImGui::SameLine();
            if (ImGui::RadioButton(UI::idLabel("common.truth_file", "##ref_form"), byFile)) task->truth.form = RefForm::File;
        }

        if (task->truth.form == RefForm::File) {
            renderSP3Row(task);
        } else {
            renderXYZRow(task, !realtime);
        }
    }

    void persist(const Task *task) {
        const auto &pm = AppConfig::PathMemory::instance();
        pm.set(keyOf(task->mode, "_ref_form"),
               task->truth.form == RefForm::File
                   ? "file"
                   : task->truth.form == RefForm::Fixed
                         ? "fixed"
                         : "none");
        pm.set(SP3Key(task->mode), task->truth.sp3Path);
        const Eigen::Vector3d xyz = task->truth.fixed;
        std::ostringstream ss;
        ss << std::setprecision(17) << xyz.x() << ',' << xyz.y() << ',' << xyz.z();
        pm.set(keyOf(task->mode, "_ref_truth_xyz"), ss.str());
    }

    void restore(Task *task) {
        const auto &pm = AppConfig::PathMemory::instance();
        if (const std::string form = pm.get(keyOf(task->mode, "_ref_form")); form == "file") task->truth.form = RefForm::File;
        else if (form == "fixed") task->truth.form = RefForm::Fixed;
        else if (form == "none") task->truth.form = RefForm::None;
        else if (const std::string e = pm.get(keyOf(task->mode, "_ref_truth_enable"));
            !e.empty() && e != "0")
            task->truth.form = RefForm::Fixed; // 旧配置只有启用位

        if (const std::string sp3 = pm.get(SP3Key(task->mode)); !sp3.empty())
            task->truth.sp3Path = sp3;
        if (task->truth.form == RefForm::None && !task->truth.sp3Path.empty()
            && modeAllowsRefFile(task->mode))
            task->truth.form = RefForm::File;

        if (!modeAllowsRefFile(task->mode) && task->truth.form == RefForm::File)
            task->truth.form = modeAllowsRefFixed(task->mode) ? RefForm::Fixed : RefForm::None;
        if (!modeAllowsRefFixed(task->mode) && task->truth.form == RefForm::Fixed)
            task->truth.form = RefForm::None;

        if (const std::string xyz = pm.get(keyOf(task->mode, "_ref_truth_xyz")); !xyz.empty()) {
            const auto parts = Str::split(xyz, ',', false);
            double v[3] = {0, 0, 0};
            bool ok = parts.size() == 3;
            for (std::size_t i = 0; ok && i < 3; ++i) {
                const char *cur = parts[i].c_str();
                ok = Str::next(cur, cur + parts[i].size(), v[i]);
            }
            if (ok) task->truth.fixed = Eigen::Vector3d(v[0], v[1], v[2]);
        }
    }
}
