#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include "GuiFileProcessor.h"
#include "ui/Gui.h"        // 现代 IFileDialog 文件对话框（DPI 清晰）
#include "imgui.h"
#include "OEM7Reader.h"
#include "SPPCodePhase.h"
#include "QualityControl.h"
#include "Const.h"
#include "QCProcessor.h"     // namespace QC: QualityReport / QCObsEpoch / compute
#include "implot.h"

#include <fstream>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <set>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <chrono>
#include <thread>
#include <utility>

#include "Log.h"
#include "RinexObsReader.h"
#include "RinexNavStore.h"

namespace GuiFileProcessor {
    auto oldTime = CommonTime(61120);

    void SppEpochData::getFromSPP(const SPPIFCode &spp) {
        if (auto &result = spp.result; result.numSats > 0) {
            solved = true;
            sppResult = result;
            numSatsResult = result.numSats;
            const auto size = static_cast<int>(satIds.size());
            for (int i = 0; i < size; i++) {
                if (auto it = spp.satElevData.find(satIds[i]); it != spp.satElevData.end())
                    elevations[i] = it->second;
                if (auto it2 = spp.satAzimData.find(satIds[i]); it2 != spp.satAzimData.end())
                    azimuths[i] = it2->second;
                if (auto it3 = spp.satPVTTransTime.find(satIds[i]); it3 != spp.satPVTTransTime.end())
                    satPVTs[i] = it3->second;
                if (spp.satRejected.count(satIds[i])) {
                    rejected[i] = true;
                    numSatsResult--;
                }
            }
        } else { solved = false; }
    }

    void SppEpochData::getFromObs(const ObsData &obs) {
        week = obs.weekSecond.week;
        sow = obs.weekSecond.sow;
        const auto numSats = static_cast<int>(obs.satTypeValueData.size());
        satIds.reserve(numSats);
        elevations.reserve(numSats);
        azimuths.reserve(numSats);
        satPVTs.reserve(numSats);
        rejected.reserve(numSats);
        allObs.reserve(numSats);
        for (auto &[sat, typeMap]: obs.satTypeValueData) {
            satIds.push_back(sat);
            elevations.push_back(0.0);
            azimuths.push_back(0.0);
            satPVTs.emplace_back();
            rejected.push_back(false);
            allObs.push_back(typeMap);
        }
        numObs = static_cast<int>(satIds.size());
    }

    void PlotData::insert(int index, const SppEpochData &ep, const XYZ &refECEF) {
        times.push_back(index);
        const auto &result = ep.sppResult;
        const bool solved = ep.solved;
        sigmaPs.push_back(solved ? result.sigmaP : 0.0);
        sigmaVs.push_back(solved ? result.sigmaV : 0.0);
        pdops.push_back(solved ? result.pdop : 0.0);
        if (solved) {
            auto enu = XYZtoENU(result.xyz, refECEF);
            enu_e.push_back(enu[0]);
            enu_n.push_back(enu[1]);
            enu_u.push_back(enu[2]);
        } else {
            enu_e.push_back(0);
            enu_n.push_back(0);
            enu_u.push_back(0);
        }
        if (solved)
            for (auto &[satID, res]: result.postRes) {
                satResTimes[satID].push_back(index);
                satResVals[satID].push_back(res);
            }
        newed = true;
    }

    void PlotData::refreshENU(const std::vector<SppEpochData> &ep, const XYZ &refECEF) {
        for (int i = 0; i < (int) enu_e.size(); i++) {
            if (ep[i].solved) {
                auto enu = XYZtoENU(ep[i].sppResult.xyz, refECEF);
                enu_e[i] = enu[0];
                enu_n[i] = enu[1];
                enu_u[i] = enu[2];
            } else {
                enu_e[i] = 0;
                enu_n[i] = 0;
                enu_u[i] = 0;
            }
        }
    }

    void PlotData::clear() {
        times.clear();
        sigmaPs.clear();
        sigmaVs.clear();
        pdops.clear();
        enu_e.clear();
        enu_n.clear();
        enu_u.clear();
        satResTimes.clear();
        satResVals.clear();
        resRangeReady = false;
        resYlo = -8.0;
        resYhi = 8.0;
    }

    // 由当前所有后验残差估计稳健 Y 轴范围（忽略极端粗差，使主体散点清晰可见）
    static void computeRobustResRange(PlotData &pp) {
        std::vector<double> allv;
        for (auto &[sat, vals]: pp.satResVals)
            for (double v: vals) allv.push_back(v);
        if (allv.size() < 2) {
            pp.resYlo = -8;
            pp.resYhi = 8;
            pp.resRangeReady = true;
            return;
        }
        std::sort(allv.begin(), allv.end());
        size_t n = allv.size();
        double lo = allv[(size_t) (0.01 * n)];
        double hi = allv[(size_t) (0.99 * n)];
        double half = 0.5 * (hi - lo);
        if (!(half > 0.0)) half = 1.0;
        half = std::max(1.0, std::min(half, 20.0));
        pp.resYlo = -half;
        pp.resYhi = half;
        pp.resRangeReady = true;
    }

    // ----------------------------------------------------------
    // 伴生文件扫描
    // ----------------------------------------------------------
    std::vector<std::string> ScanNavFiles(const std::string &obsPath) {
        std::vector<std::string> result;
        if (obsPath.size() < 4) return result;
        const std::string base = obsPath.substr(0, obsPath.size() - 1);
        // RINEX 3: .??N, .??G, .??C, .??F 等
        for (auto &ext: {"N", "G", "C", "F", "E", "J", "I", "L"}) {
            std::string np = base + ext;
            std::ifstream t(np);
            if (t.good()) {
                t.close();
                result.push_back(np);
            }
        }
        return result;
    }

    // ----------------------------------------------------------
    // 配置面板
    // ----------------------------------------------------------
    void RenderConfigPanel(const std::shared_ptr<SppTask> &task) {
        // 跨文件记住用户选择（static 在进程生命周期内有效；关闭重开则恢复默认）
        static bool s_usePhase = false;
        static std::set s_systems = {'G', 'C'};

        ImGui::SetNextWindowFocus();
        ImGui::Begin("处理配置", nullptr, ImGuiWindowFlags_NoCollapse);
        ImGui::Text("文件: %s", task->fileName.c_str());
        ImGui::Separator();

        // 伴生文件
        if (!task->navFiles.empty()) {
            ImGui::Text("检测到伴生星历文件:");
            for (auto &f: task->navFiles) {
                size_t pos = f.rfind('/');
                if (pos == std::string::npos) pos = f.rfind('\\');
                ImGui::BulletText("%s", pos != std::string::npos ? f.c_str() + pos + 1 : f.c_str());
            }
        } else if (task->filePath.size() >= 4 && std::isdigit(static_cast<unsigned char>(task->filePath[task->filePath.size() - 3]))) {
            ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "未找到伴生星历文件（同目录下 .??N/.??G/.??C 等）");
        }
        ImGui::Separator();

        // 星座（绑定到 static，跨文件保持）
        ImGui::Text("选择星座:");
        struct {
            char ch;
            const char *label;
        } cons[] = {{'G', "GPS"}, {'C', "BDS"}};
        for (auto &c: cons) {
            bool on = s_systems.count(c.ch) > 0;
            ImGui::SameLine();
            ImGui::Checkbox(c.label, &on);
            if (on) s_systems.insert(c.ch);
            else s_systems.erase(c.ch);
        }
        ImGui::NewLine();
        ImGui::Separator();

        // 解法
        ImGui::Text("选择解法:");
        static const char *methods[] = {"IF-code 纯伪距 (SPP)", "IF 组合载波相位"};
        int method = s_usePhase ? 1 : 0;
        ImGui::RadioButton(methods[0], &method, 0);
        ImGui::SameLine();
        ImGui::RadioButton(methods[1], &method, 1);
        s_usePhase = (method == 1);
        ImGui::Separator();

        if (ImGui::Button("开始处理", ImVec2(200, 40))) {
            task->usePhase = s_usePhase;
            task->enabledSystems = s_systems;
            task->state = SppTask::State::Running;
            task->worker = std::thread(SolveThread, task);
        }
        ImGui::SameLine();
        if (ImGui::Button("取消", ImVec2(80, 40))) {
            task->state = SppTask::State::Done;
            task->hasError = true;
            task->errorMsg = "用户取消";
        }
        ImGui::End();
    }

    // ----------------------------------------------------------
    // 读取 RINEX 数据
    // ----------------------------------------------------------
    static bool ReadRinex(const std::shared_ptr<SppTask> &task,
                          std::vector<ObsData> &allObs,
                          std::map<char, std::vector<string> > &availableTypes,
                          EphemerisTable &ephTable) {
        task->phase = SppTask::Phase::Reading;
        RinexNavStore navStore;
        int navLoaded = 0;
        for (auto &np: task->navFiles) {
            try {
                navStore.loadFile(np, ephTable);
                navLoaded++;
                task->readProgress = 0.25f * navLoaded / (std::max)(1, (int) task->navFiles.size());
            } catch (const std::exception &e) { LOG_ERROR << "nav fail " << np << ": " << e.what(); }
        }
        task->hasNav = (navLoaded > 0);
        if (navLoaded == 0)
            LOG_WARN << "未找到伴生星历文件，将无法定位解算；质量分析（仅依赖原始观测）仍正常进行。";
        task->readProgress = 0.25f;

        RinexObsReader obsReader;
        std::fstream obsFile(task->filePath.c_str(), std::ios::in);
        if (!obsFile) {
            task->hasError = true;
            task->errorMsg = "无法打开 RINEX";
            return false;
        }
        obsReader.setFileStream(&obsFile);
        bool hdr = false;
        int safety = 0, ok = 0;
        while (++safety < 100000) {
            try {
                ObsData o = obsReader.parseRinexObs();
                allObs.push_back(std::move(o));
                ok++;
                task->readProgress = 0.25f + 0.70f * (1.0f - 1.0f / (1.0f + ok * 0.005f));
                if (!hdr) {
                    hdr = true;
                    availableTypes = obsReader.getHeader().mapObsTypes;
                }
            } catch (const EndOfFile &) { break; } catch (const std::exception &e) {
                task->hasError = true;
                task->errorMsg = "RINEX parse err: " + std::string(e.what());
                return false;
            }
        }
        if (ok == 0) {
            task->hasError = true;
            task->errorMsg = "RINEX 无历元";
            return false;
        }
        task->totalEpochs = ok;
        {
            std::lock_guard lock(task->mutex);
            task->epochs.clear();
            task->epochs.reserve(allObs.size());
            for (auto &obs: allObs) {
                SppEpochData data;
                data.getFromObs(obs);
                task->epochs.push_back(std::move(data));
            }
        }
        return true;
    }

    // ----------------------------------------------------------
    // 读取 OEM7 数据
    // ----------------------------------------------------------
    static bool ReadOEM7(const std::shared_ptr<SppTask> &task,
                         std::vector<ObsData> &allObs,
                         std::map<char, std::vector<string> > &availableTypes,
                         std::vector<EphemerisTable> &ephSnapshots) {
        task->phase = SppTask::Phase::Reading;
        OEM7Reader oem7;
        if (!oem7.open(task->filePath)) {
            task->hasError = true;
            task->errorMsg = "无法打开 OEM7";
            return false;
        }
        oem7.readAll(allObs, ephSnapshots, &task->readProgress);
        task->totalEpochs = static_cast<int>(allObs.size());
        if (task->totalEpochs == 0) return false;
        {
            std::lock_guard lock(task->mutex);
            task->epochs.clear();
            task->epochs.reserve(allObs.size());
            for (auto &obs: allObs) {
                SppEpochData data;
                data.getFromObs(obs);
                task->epochs.push_back(std::move(data));
            }
        }
        // OEM7 没有 RINEX 头，但可用观测量类型已在读取时收集（availableTypes_），
        // 与 RINEX 完全一致地交由 SPPIFCode::setIFCodeTypesAuto 处理（含默认兜底）。
        availableTypes = oem7.getAvailableTypes();
        return true;
    }

    // ----------------------------------------------------------
    // 质量分析（QC）后台计算：把 SppTask 的观测结果转成 QC 输入并调用 QC::compute。
    // 质量分析与定位解算无因果关系——只依赖原始观测(C/L/S)，与是否解出定位无关。
    // 每历元(无论 solved 与否)都喂给 QC；高度角/DOP/卫星数等几何量在 QCProcessor
    // 内部仅在「解算器确实提供了几何信息(nsat>0)」时才纳入。
    static QualityReport buildReport(SppTask &task) {
        std::vector<QC::QCObsEpoch> qcEpochs;
        {
            // 读取 task->epochs 时加锁，避免与解算线程的逐历元写入竞争
            std::lock_guard lk(task.mutex);
            qcEpochs.reserve(task.epochs.size());
            for (const auto &ep: task.epochs) {
                QC::QCObsEpoch qe;
                qe.sow = ep.sow;
                qe.satIds = ep.satIds;
                qe.allObs = ep.allObs;
                // SppEpochData.elevations/azimuths 为弧度，QC 显示用角度，这里转为度
                for (double e: ep.elevations) qe.elevations.push_back(e * RAD_TO_DEG);
                for (double a: ep.azimuths) qe.azimuths.push_back(a * RAD_TO_DEG);
                qe.pdop = ep.sppResult.pdop;
                qe.hdop = ep.sppResult.hdop;
                qe.vdop = ep.sppResult.vdop;
                qe.gdop = ep.sppResult.gdop;
                qe.nsat = ep.sppResult.numSats;
                qcEpochs.push_back(std::move(qe));
            }
        }
        QualityReport rep = QC::compute(qcEpochs);
        rep.totalInputEpochs = (int) task.epochs.size(); // 全部读取历元（含未解算）
        return rep;
    }

    // 在后台线程算一次 QC 并整体替换 task->qcReport。渲染线程只读缓存，绝不重算。
    // 触发点：读完后(观测指标立即可见) 与 全部解算完(补上 DOP/卫星数) 各一次。
    static void launchQC(const std::shared_ptr<SppTask> &task) {
        if (task->qcWorker.joinable()) task->qcWorker.join(); // 确保上一轮已结束，再起新的一轮
        task->qcComputing = true;
        task->qcWorker = std::thread([task]() {
            try {
                QualityReport rep = buildReport(*task);
                {
                    std::lock_guard lk(task->qcMutex);
                    task->qcReport = std::make_shared<QualityReport>(std::move(rep));
                }
                task->qcReady = true;
            } catch (...) {
                task->qcReady = true; // 出错也标记 ready，避免渲染一直等待
            }
            task->qcComputing = false;
        });
    }

    // 概览 / Skyplot 共用的历元导航（箭头 + 滑块 + 当前历元信息）。
    // selectedIdx 由调用方在锁外计算后传入；滑块只改 task->selectedEpoch（原子）。
    static void RenderEpochNav(const std::shared_ptr<SppTask> &task, int epochCount, int selectedIdx) {
        if (ImGui::Button("<<")) task->selectedEpoch = 0;
        ImGui::SameLine();
        if (ImGui::Button("<")) { if (task->selectedEpoch > 0) task->selectedEpoch--; }
        ImGui::SameLine();
        ImGui::PushItemWidth(150);
        int ep = selectedIdx + 1;
        if (ImGui::SliderInt("##es", &ep, 1, epochCount)) task->selectedEpoch = ep - 1;
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button(">")) { if (task->selectedEpoch < epochCount - 1) task->selectedEpoch++; }
        ImGui::SameLine();
        if (ImGui::Button(">>")) task->selectedEpoch = epochCount - 1;
        ImGui::SameLine();
        if (selectedIdx >= 0) {
            std::lock_guard lock(task->mutex);
            auto &c = task->epochs[selectedIdx];
            ImGui::TextDisabled("| Wk %u SOW %.3f | %s", c.week, c.sow, c.solved ? "定位" : "无解");
        }
    }

    // Skyplot（天顶图，RTKLIB 风格）——轨迹版本：
    // 所有历元卫星位置画细线/小点（轨迹）；选中历元的卫星画大点+标签、按系统着色。
    static void DrawSkyplotTracks(const std::shared_ptr<SppTask> &task, int sel) {
        // ---- 读取所有历元的卫星位置 ----
        struct SatPt {
            int epIdx;
            float azDeg, elDeg;
            bool used;
        };
        std::map<SatID, std::vector<SatPt> > tracks;
        int totalEp = 0;
        {
            std::lock_guard lk(task->mutex);
            totalEp = (int) task->epochs.size();
            for (int e = 0; e < totalEp; ++e) {
                auto &ep = task->epochs[e];
                for (int j = 0; j < (int) ep.satIds.size(); ++j) {
                    double el = ep.elevations[j] * RAD_TO_DEG;
                    if (el <= 0.0) continue; // 无几何信息
                    tracks[ep.satIds[j]].push_back({
                        e,
                        (float) (ep.azimuths[j] * RAD_TO_DEG),
                        (float) el,
                        ep.solved && !ep.rejected[j]
                    });
                }
            }
        }

        // ---- 布局（约束为方形 — 填满可用宽度，高度由 AutoResizeY 跟随） ----
        const float availW = ImGui::GetContentRegionAvail().x;
        const float sizeW = std::max(220.0f, std::min(availW, 600.0f));
        const float pad = 30.0f;
        const float R = std::max(70.0f, (sizeW - 2.0f * pad) / 2.0f);
        ImVec2 c0 = ImGui::GetCursorScreenPos();
        const float cx = c0.x + pad + R, cy = c0.y + pad + R;
        ImGui::Dummy(ImVec2(sizeW, sizeW));
        ImDrawList *dl = ImGui::GetWindowDrawList();

        // ---- 网格 ----
        const ImU32 grid = IM_COL32(90, 90, 110, 255);
        const ImU32 gridFaint = IM_COL32(70, 70, 85, 200);
        dl->AddCircle(ImVec2(cx, cy), R, grid, 64, 1.5f);
        for (int el = 30; el < 90; el += 30) {
            const float r = R * (1.0f - (float) el / 90.0f);
            dl->AddCircle(ImVec2(cx, cy), r, gridFaint, 64, 1.0f);
        }
        const char *dirs[4] = {"N", "E", "S", "W"};
        for (int d = 0; d < 4; ++d) {
            const float a = (float) d * 90.0f * (float) DEG_TO_RAD;
            const float x = cx + R * std::sin(a), y = cy - R * std::cos(a);
            dl->AddLine(ImVec2(cx, cy), ImVec2(x, y), gridFaint, 1.0f);
            const float lx = cx + (R + 14.0f) * std::sin(a), ly = cy - (R + 14.0f) * std::cos(a);
            dl->AddText(ImVec2(lx - 4.0f, ly - 8.0f), IM_COL32(200, 200, 220, 255), dirs[d]);
        }

        // ---- 系统着色 ----
        auto satCol = [](char s) -> ImU32 {
            switch (s) {
                case 'G': return IM_COL32(80, 220, 120, 255);
                case 'C': return IM_COL32(255, 130, 60, 255);
                case 'R': return IM_COL32(80, 200, 255, 255);
                case 'E': return IM_COL32(240, 220, 70, 255);
                default: return IM_COL32(220, 220, 220, 255);
            }
        };
        auto pt = [&](float azDeg, float elDeg) -> ImVec2 {
            const float r = R * (1.0f - elDeg / 90.0f);
            const float a = azDeg * (float) DEG_TO_RAD;
            return {cx + r * std::sin(a), cy - r * std::cos(a)};
        };

        // ---- 轨迹（按卫星系统着色的折线 + 稀疏采样小点） ----
        for (auto &[sat, pts]: tracks) {
            if (pts.size() < 2) continue;
            ImU32 trkCol = satCol(sat.system);
            // 降低 alpha 使轨迹半透明但不至于看不见
            trkCol = IM_COL32((trkCol >> IM_COL32_R_SHIFT) & 0xFF,
                              (trkCol >> IM_COL32_G_SHIFT) & 0xFF,
                              (trkCol >> IM_COL32_B_SHIFT) & 0xFF, 120);
            int segStart = 0;
            for (size_t i = 1; i <= pts.size(); ++i) {
                if (i == pts.size() || (pts[i].epIdx - pts[i - 1].epIdx > 10)) {
                    if (i - segStart >= 2) {
                        std::vector<ImVec2> seg;
                        for (size_t k = segStart; k < i; ++k)
                            seg.push_back(pt(pts[k].azDeg, pts[k].elDeg));
                        dl->AddPolyline(seg.data(), (int) seg.size(), trkCol, ImDrawFlags_None, 1.5f);
                        // 每隔 5 个点画一个小圆点，使轨迹更容易看到
                        for (size_t k = segStart; k < i; k += 5)
                            dl->AddCircleFilled(seg[k - segStart], 1.5f, trkCol);
                    }
                    segStart = (int) i;
                }
            }
        }

        // ---- 选中历元（大点 + 标签） ----
        {
            std::lock_guard lk(task->mutex);
            if (sel >= 0 && sel < totalEp) {
                auto &ep = task->epochs[sel];
                for (int j = 0; j < (int) ep.satIds.size(); ++j) {
                    double elDeg = ep.elevations[j] * RAD_TO_DEG;
                    if (elDeg <= 0.0) continue;
                    double azDeg = ep.azimuths[j] * RAD_TO_DEG;
                    ImVec2 p = pt((float) azDeg, (float) elDeg);
                    bool used = ep.solved && !ep.rejected[j];
                    ImU32 col = satCol(ep.satIds[j].system);
                    if (used) {
                        dl->AddCircleFilled(p, 6.0f, col);
                        dl->AddText(ImVec2(p.x - 10.0f, p.y - 24.0f), col, ep.satIds[j].toString().c_str());
                    } else {
                        dl->AddCircle(p, 4.5f, IM_COL32(150, 150, 160, 200), 12, 1.0f);
                    }
                }
            }
        }
    }

    // ----------------------------------------------------------
    // SolveThread
    // ----------------------------------------------------------
    void SolveThread(const std::shared_ptr<SppTask> &task) {
        try {
            const auto &path = task->filePath;
            LOG_INFO << "处理文件: " << path;

            const bool isRinex = (path.size() >= 4 &&
                                  std::isdigit(static_cast<unsigned char>(path[path.size() - 3])) &&
                                  std::toupper(path.back()) == 'O');

            task->loading = true;
            task->hasError = false;
            task->errorMsg.clear();

            // ---- 阶段 1: 读数据 ----
            std::vector<ObsData> allObs;
            std::map<char, std::vector<string> > availableTypes;
            EphemerisTable ephTable;
            std::vector<EphemerisTable> ephSnaps;

            bool ok = isRinex
                          ? ReadRinex(task, allObs, availableTypes, ephTable)
                          : ReadOEM7(task, allObs, availableTypes, ephSnaps);
            if (!ok) {
                task->loading = false;
                task->done = true;
                return;
            }
            task->readProgress = 1.0f;
            launchQC(task); // 读完后即可后台算一次 QC（观测指标立即可见，无需等解算）

            // ---- 阶段 2: 解算 ----
            task->phase = SppTask::Phase::Solving;
            task->solvingProgress = 0;

            if (!task->hasNav && isRinex) {
                // 无伴生星历：无法进行定位解算（逐历元都会因缺星历失败），直接跳过。
                // 观测数据已在阶段 1 读入，质量分析（仅依赖原始观测）可正常进行。
                task->noEphSolve = true;
            } else {
                // 仅保留启用系统，供自动检测 / 默认兜底使用
                std::map<char, std::vector<string> > availForDetect;
                for (auto &[sys, types]: availableTypes)
                    if (task->enabledSystems.count(sys)) availForDetect[sys] = types;
                IFCodeTypes defaultTypes;
                if (task->enabledSystems.count('G')) defaultTypes['G'] = {"C1", "C2"};
                if (task->enabledSystems.count('C')) defaultTypes['C'] = {"C2", "C6"};

                const int N = task->totalEpochs;
                EphemerisTable *lastGoodEph = nullptr; // OEM7 无星历回退

                if (task->usePhase) {
                    SPPCodePhase spp;
                    spp.enableKalman(true);
                    spp.setIFCodeTypesAuto(availForDetect, defaultTypes);
                    for (int i = 0; i < N; ++i) {
                        if (task->stop) break;
                        ObsData &obs = allObs[i];
                        if (isRinex) spp.setEphemerisTable(&ephTable);
                        else {
                            auto *cur = &ephSnaps[i];
                            if (cur->gps.empty() && cur->bds.empty() && lastGoodEph) cur = lastGoodEph;
                            else if (!cur->gps.empty() || !cur->bds.empty()) lastGoodEph = cur;
                            spp.setEphemerisTable(cur);
                        }
                        spp.preprocess(obs);
                        bool solve_ok = false;
                        try {
                            spp.solve(obs);
                            solve_ok = true;
                        } catch (...) { solve_ok = false; }
                        {
                            std::lock_guard lock(task->mutex);
                            SppEpochData &data = task->epochs[i];
                            if (solve_ok) {
                                data.getFromSPP(spp);
                                if (!task->initializedRefECEF) {
                                    task->refECEF = data.sppResult.xyz;
                                    task->initializedRefECEF = true;
                                }
                            } else { data.solved = false; }
                            task->solvingProgress = i + 1;
                            task->plotData.insert(i, data, task->refECEF);
                            if (task->selectedEpoch == -1 || task->selectedEpoch == i - 1) task->selectedEpoch = i;
                        }
                    }
                } else {
                    // ---- IF-code 纯伪距 ----
                    SPPIFCode spp;
                    spp.setIFCodeTypesAuto(availForDetect, defaultTypes);
                    for (int i = 0; i < N; ++i) {
                        if (task->stop) break;
                        ObsData &obs = allObs[i];
                        if (isRinex) spp.setEphemerisTable(&ephTable);
                        else {
                            auto *cur = &ephSnaps[i];
                            if (cur->gps.empty() && cur->bds.empty() && lastGoodEph) cur = lastGoodEph;
                            else if (!cur->gps.empty() || !cur->bds.empty()) lastGoodEph = cur;
                            spp.setEphemerisTable(cur);
                        }
                        spp.preprocess(obs);
                        bool solve_ok = false;
                        try {
                            spp.solve(obs);
                            solve_ok = true;
                        } catch (...) { solve_ok = false; }
                        {
                            std::lock_guard lock(task->mutex);
                            SppEpochData &data = task->epochs[i];
                            if (solve_ok) {
                                data.getFromSPP(spp);
                                if (!task->initializedRefECEF) {
                                    task->refECEF = data.sppResult.xyz;
                                    task->initializedRefECEF = true;
                                }
                            } else { data.solved = false; }
                            task->solvingProgress = i + 1;
                            task->plotData.insert(i, data, task->refECEF);
                            if (task->selectedEpoch == -1 || task->selectedEpoch == i - 1) task->selectedEpoch = i;
                        }
                    }
                }
            } // 关闭「有星历才解算」的 else 分支
        } catch (const std::exception &e) {
            task->hasError = true;
            task->errorMsg = e.what();
        }
        task->loading = false;
        task->state = SppTask::State::Done;
        task->done = true;
    }

    // ----------------------------------------------------------
    // RenderTask
    // ----------------------------------------------------------
    void RenderTask(const std::shared_ptr<SppTask> &task, const bool isRealtime) {
        if (task->state == SppTask::State::Config) {
            RenderConfigPanel(task);
            return;
        }

        int epochCount = 0;
        int selectedIdx = -1;
        const bool isLoading = task->loading.load();
        const bool isDone = task->done.load();
        const bool hasError = task->hasError;

        {
            std::lock_guard lock(task->mutex);
            epochCount = (int) task->epochs.size();
            if (task->selectedEpoch >= epochCount) task->selectedEpoch = epochCount - 1;
            if (task->selectedEpoch < 0 && epochCount > 0) task->selectedEpoch = 0;
            selectedIdx = task->selectedEpoch;
        }

        if (!isRealtime && isLoading && !hasError && task->phase == SppTask::Phase::Reading) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "阶段 1/2: 正在读取文件...");
            ImGui::ProgressBar(task->readProgress.load(), ImVec2(200, 0), "");
            return;
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem("定位解算")) {
                // 顶部状态
                if (hasError) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "状态: %s", task->errorMsg.c_str());
                else if (epochCount > 0) {
                    if (task->noEphSolve)
                        ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "处理完成: 共 %d 个历元 | 无伴生星历，已跳过定位解算（质量分析基于原始观测可用）", epochCount);
                    else
                        ImGui::TextColored(ImVec4(0.3f, 1, 0.3f, 1), "处理完成: 共 %d 个历元 | 解法: %s%s", epochCount,
                                           task->usePhase ? "IF-Phase" : "IF-code", "");
                } else if (isLoading) ImGui::Text("解算中...");

                // 质量分析后台计算进度（与定位解算解耦；读完后即开始，无需等解算）
                // 注意：QC 计算本身不报告进度（lib 层无线程/回调），这里用跑马灯表示“正在算”。
                if (task->qcComputing && !task->qcReady) {
                    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "质量分析计算中…");
                    float t = (float) fmod(ImGui::GetTime() * 0.6, 1.0);
                    ImGui::ProgressBar(t, ImVec2(220, 0));
                }

                // epoch 导航
                if (epochCount > 0) {
                    RenderEpochNav(task, epochCount, selectedIdx);
                }

                // 参考真值
                {
                    ImGui::Text("参考真值:");
                    ImGui::SameLine();
                    ImGui::PushItemWidth(200);
                    if (ImGui::InputDouble("X/m", &task->refECEF[0])) task->plotData.refreshENU(task->epochs, task->refECEF);
                    ImGui::SameLine();
                    if (ImGui::InputDouble("Y/m", &task->refECEF[1])) task->plotData.refreshENU(task->epochs, task->refECEF);
                    ImGui::SameLine();
                    if (ImGui::InputDouble("Z/m", &task->refECEF[2])) task->plotData.refreshENU(task->epochs, task->refECEF);
                    ImGui::SameLine();
                    if (ImGui::Button("剪贴板")) {
                        if (const char *c = ImGui::GetClipboardText()) {
                            double v[3] = {};
                            char *e;
                            const char *p = c;
                            int cnt = 0;
                            while (*p && cnt < 3) {
                                while (*p && !(*p == '-' || (*p >= '0' && *p <= '9')))p++;
                                if (!*p)break;
                                v[cnt] = strtod(p, &e);
                                if (p == e)break;
                                p = e;
                                cnt++;
                            }
                            if (cnt >= 1) task->refECEF[0] = v[0];
                            if (cnt >= 2) task->refECEF[1] = v[1];
                            if (cnt >= 3) task->refECEF[2] = v[2];
                            task->plotData.refreshENU(task->epochs, task->refECEF);
                        }
                    }
                }

                ImGui::Separator();

                if (epochCount == 0) {
                    ImGui::Text("无历元数据。");
                    ImGui::EndTabItem();
                    if (ImGui::BeginTabItem("质量分析")) {
                        QualityControl::render(task);
                        ImGui::EndTabItem();
                    }
                    ImGui::EndTabBar();
                    return;
                }

                // 主分栏
                const float availY = ImGui::GetContentRegionAvail().y;
                const float plotH = 380;
                float th = availY - plotH - ImGui::GetStyle().ItemSpacing.y;
                if (th < 200) th = 200;

                if (ImGui::BeginTable("##ms", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                    ImGui::TableSetupColumn("L", ImGuiTableColumnFlags_WidthStretch, 0.6f);
                    ImGui::TableSetupColumn("R", ImGuiTableColumnFlags_WidthStretch, 0.4f);
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, th);

                    // 左: 卫星列表
                    ImGui::TableSetColumnIndex(0);
                    ImGui::BeginChild("##sat");
                    if (epochCount > 0 && selectedIdx >= 0) {
                        SppEpochData cur;
                        {
                            std::lock_guard lk(task->mutex);
                            cur = task->epochs[selectedIdx];
                        }
                        ImGui::SeparatorText("卫星概览");
                        if (ImGui::BeginTable(
                            "##sm", 9,
                            ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                            ImVec2(0, ImGui::GetContentRegionAvail().y * 0.8f))) {
                            ImGui::TableSetupColumn("序", ImGuiTableColumnFlags_WidthFixed, 50);
                            ImGui::TableSetupColumn("星", ImGuiTableColumnFlags_WidthFixed, 50);
                            ImGui::TableSetupColumn("X(m)");
                            ImGui::TableSetupColumn("Y(m)");
                            ImGui::TableSetupColumn("Z(m)");
                            ImGui::TableSetupColumn("仰(°)", ImGuiTableColumnFlags_WidthFixed, 80);
                            ImGui::TableSetupColumn("方(°)", ImGuiTableColumnFlags_WidthFixed, 80);
                            ImGui::TableSetupColumn("态", ImGuiTableColumnFlags_WidthFixed, 50);
                            ImGui::TableSetupColumn("残(m)", ImGuiTableColumnFlags_WidthFixed, 80);
                            ImGui::TableHeadersRow();
                            for (int i = 0; i < (int) cur.satIds.size(); i++) {
                                ImGui::TableNextRow();
                                ImGui::TableSetColumnIndex(0);
                                ImGui::Text("%d", i + 1);
                                ImGui::TableSetColumnIndex(1);
                                if (ImGui::Selectable(cur.satIds[i].toString().c_str(), task->selectedSatIdx == i,
                                                      ImGuiSelectableFlags_SpanAllColumns)) task->selectedSatIdx = i;
                                auto sh = [](double v) {
                                    if (v != 0) ImGui::Text("%.1f", v);
                                    else ImGui::TextUnformatted("?");
                                };
                                ImGui::TableSetColumnIndex(2);
                                sh(cur.satPVTs[i].p[0]);
                                ImGui::TableSetColumnIndex(3);
                                sh(cur.satPVTs[i].p[1]);
                                ImGui::TableSetColumnIndex(4);
                                sh(cur.satPVTs[i].p[2]);
                                ImGui::TableSetColumnIndex(5);
                                sh(cur.elevations[i] * RAD_TO_DEG);
                                ImGui::TableSetColumnIndex(6);
                                sh(cur.azimuths[i] * RAD_TO_DEG);
                                ImGui::TableSetColumnIndex(7);
                                if (cur.rejected[i] || !cur.solved) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "排除");
                                else ImGui::TextColored(ImVec4(0.3f, 1, 0.3f, 1), "参与");
                                ImGui::TableSetColumnIndex(8);
                                if (auto it = cur.sppResult.postRes.find(cur.satIds[i]); it != cur.sppResult.postRes.end()) ImGui::Text(
                                    "%.4f", it->second);
                                else ImGui::TextDisabled("-");
                            }
                            ImGui::EndTable();
                        }
                        // 观测详情
                        if (task->selectedSatIdx >= 0 && task->selectedSatIdx < (int) cur.satIds.size()) {
                            int si = task->selectedSatIdx;
                            ImGui::SeparatorText(("观测详情 (" + cur.satIds[si].toString() + ")").c_str());
                            if (ImGui::BeginTable("##od", 5, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                                ImGui::TableSetupColumn("频", ImGuiTableColumnFlags_WidthFixed, 60);
                                ImGui::TableSetupColumn("伪距(m)");
                                ImGui::TableSetupColumn("载波(cyc)");
                                ImGui::TableSetupColumn("Doppler(Hz)");
                                ImGui::TableSetupColumn("强度");
                                ImGui::TableHeadersRow();
                                std::set<std::string> freqs;
                                for (auto &[t,v]: cur.allObs[si]) if (t.size() > 1) freqs.insert(t.substr(1));
                                if (freqs.empty()) freqs.insert("?");
                                for (auto &f: freqs) {
                                    ImGui::TableNextRow();
                                    ImGui::TableSetColumnIndex(0);
                                    std::string l = f;
                                    if (cur.satIds[si].system == 'G') l = "L" + f;
                                    else if (cur.satIds[si].system == 'C') l = "B" + f;
                                    ImGui::Text("%s", l.c_str());
                                    auto show = [&](const char *p) {
                                        auto it = cur.allObs[si].find(p + f);
                                        if (it != cur.allObs[si].end()) ImGui::Text("%.3f", it->second);
                                        else ImGui::TextDisabled("-");
                                    };
                                    ImGui::TableSetColumnIndex(1);
                                    show("C");
                                    ImGui::TableSetColumnIndex(2);
                                    show("L");
                                    ImGui::TableSetColumnIndex(3);
                                    show("D");
                                    ImGui::TableSetColumnIndex(4);
                                    show("S");
                                }
                                ImGui::EndTable();
                            }
                        }
                    }
                    ImGui::EndChild();

                    // 右: 结果
                    ImGui::TableSetColumnIndex(1);
                    ImGui::BeginChild("##res");
                    if (epochCount > 0 && selectedIdx >= 0) {
                        SppEpochData cur;
                        {
                            std::lock_guard lk(task->mutex);
                            cur = task->epochs[selectedIdx];
                        }
                        if (cur.solved) {
                            auto &r = cur.sppResult;
                            auto enu = XYZtoENU(r.xyz, task->refECEF);
                            ImGui::SeparatorText("位置 (WGS84)");
                            ImGui::Text("ECEF: (%.4f, %.4f, %.4f)", r.xyz[0], r.xyz[1], r.xyz[2]);
                            ImGui::Text("  REF: (%.4f, %.4f, %.4f)", task->refECEF[0], task->refECEF[1], task->refECEF[2]);
                            ImGui::Text("  ENU: (%.4f, %.4f, %.4f)", enu[0], enu[1], enu[2]);
                            ImGui::Text("  BLH: (%.8f, %.8f, %.4f)", r.blh[0] * RAD_TO_DEG, r.blh[1] * RAD_TO_DEG, r.blh[2]);
                            ImGui::Text("  σP: %.3f", r.sigmaP);
                            ImGui::SeparatorText("速度");
                            ImGui::Text("  (%.4f, %.4f, %.4f) m/s", r.vel[0], r.vel[1], r.vel[2]);
                            ImGui::SeparatorText("DOP");
                            ImGui::Text("  PDOP:%.2f GDOP:%.2f HDOP:%.2f VDOP:%.2f TDOP:%.2f", r.pdop, r.gdop, r.hdop, r.vdop, r.tdop);
                            ImGui::Text("  卫星: %d/%d", cur.numSatsResult, cur.numObs);
                        } else ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), task->noEphSolve ? "无定位解（缺少星历文件）" : "无定位解");
                        ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 50);
                        if (isDone || isRealtime) {
                            if (ImGui::Button("导出 CSV", ImVec2(-FLT_MIN, 40))) {
                                auto h = (HWND) ImGui::GetMainViewport()->PlatformHandleRaw;
                                if (!h) h = GetActiveWindow();
                                ExportCsv(task, h);
                            }
                        } else ImGui::Button("解算中...", ImVec2(-FLT_MIN, 40));
                    }
                    ImGui::EndChild();
                    ImGui::EndTable();
                }

                // ENU（全宽）
                if (epochCount > 0) {
                    auto &pp = task->plotData;
                    ImGui::Separator();
                    if (ImPlot::BeginPlot("ENU", ImVec2(-1, 400))) {
                        ImPlot::SetupAxes("Epoch", "E/N/U (m)");
                        ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
                        if (pp.newed) ImPlot::SetupAxisLimits(ImAxis_X1, 0, pp.times.empty() ? 10 : pp.times.back(), ImPlotCond_Always);
                        pp.newed = false;
                        if (!pp.times.empty()) {
                            ImPlot::PlotLine("E", pp.times.data(), pp.enu_e.data(), (int) pp.times.size());
                            ImPlot::PlotLine("N", pp.times.data(), pp.enu_n.data(), (int) pp.times.size());
                            ImPlot::PlotLine("U", pp.times.data(), pp.enu_u.data(), (int) pp.times.size());
                        }
                        double sx = (double) task->selectedEpoch;
                        ImPlot::DragLineX(0, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
                        int se = (int) std::lround(sx);
                        if (se < 0) se = 0;
                        if (se >= epochCount) se = epochCount - 1;
                        task->selectedEpoch = se;
                        ImPlot::EndPlot();
                    }
                }

                // Skyplot (左 1/3) + 后验残差 (右 2/3)，同高
                if (epochCount > 0 && selectedIdx >= 0) {
                    ImGui::Separator();
                    auto &pp = task->plotData;
                    // 按窗口宽度 30% 估算天顶图高度，使方形 Skyplot 与残差图等齐
                    float rowH = std::max(280.0f, std::min(ImGui::GetContentRegionAvail().x * 0.30f, 480.0f));

                    ImGui::BeginTable("##skyres", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV);
                    ImGui::TableSetupColumn("Sky", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                    ImGui::TableSetupColumn("Res", ImGuiTableColumnFlags_WidthStretch, 0.67f);
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    if (ImGui::BeginChild("Skyplot", ImVec2(0, rowH), ImGuiChildFlags_FrameStyle)) {
                        DrawSkyplotTracks(task, selectedIdx);
                    }
                    ImGui::EndChild();

                    ImGui::TableSetColumnIndex(1);
                    if (ImPlot::BeginPlot("后验残差", ImVec2(-1, rowH))) {
                        ImPlot::SetupAxes("Epoch", "残差 (m)");
                        bool setLim = pp.newed;
                        if (isDone && !pp.resRangeReady) {
                            computeRobustResRange(pp);
                            setLim = true;
                        }
                        if (setLim) {
                            double xmax = pp.times.empty() ? 10.0 : (double) pp.times.back();
                            double ylo = pp.resRangeReady ? pp.resYlo : -8.0;
                            double yhi = pp.resRangeReady ? pp.resYhi : 8.0;
                            ImPlot::SetupAxesLimits(0, xmax, ylo, yhi, ImPlotCond_Always);
                        }
                        for (auto &[sat, vals]: pp.satResVals) {
                            auto it = pp.satResTimes.find(sat);
                            if (it == pp.satResTimes.end() || vals.empty()) continue;
                            ImPlot::PlotLine(sat.toString().c_str(), it->second.data(), vals.data(), (int) vals.size());
                        }
                        double sx = (double) task->selectedEpoch;
                        ImPlot::DragLineX(1, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
                        int se = (int) std::lround(sx);
                        if (se < 0) se = 0;
                        if (se >= epochCount) se = epochCount - 1;
                        task->selectedEpoch = se;
                        ImPlot::EndPlot();
                    }
                    ImGui::EndTable();
                }

                // SigmaP / SigmaV / PDOP（全宽）
                if (epochCount > 0) {
                    auto &pp = task->plotData;
                    ImGui::Separator();
                    if (ImGui::BeginTable("##bp3", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        ImGui::TableSetupColumn("A", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                        ImGui::TableSetupColumn("B", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                        ImGui::TableSetupColumn("C", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                        ImGui::TableNextRow();
                        const vector<double> *d[3] = {&pp.sigmaPs, &pp.sigmaVs, &pp.pdops};
                        const char *dn[3] = {"SigmaP", "SigmaV", "PDOP"};
                        for (int k = 0; k < 3; k++) {
                            ImGui::TableSetColumnIndex(k);
                            if (ImPlot::BeginPlot(dn[k], ImVec2(-1, 350), ImPlotFlags_NoLegend)) {
                                ImPlot::SetupAxes("Epoch", "m");
                                ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
                                if (pp.newed) ImPlot::SetupAxisLimits(ImAxis_X1, 0, pp.times.empty() ? 10 : pp.times.back(),
                                                                      ImPlotCond_Always);
                                if (!pp.times.empty()) ImPlot::PlotLine(dn[k], pp.times.data(), d[k]->data(), (int) pp.times.size());
                                ImPlot::EndPlot();
                            }
                        }
                        ImGui::EndTable();
                    }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("质量分析")) {
                QualityControl::render(task);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }

    // 宽字符路径 -> 系统 ANSI 代码页（保持与旧版 GetOpenFileNameA 一致，供 ifstream 使用）
    static std::string WideToAcp(const std::wstring &w) {
        if (w.empty()) return {};
        int n = WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
        std::string s(n > 0 ? n - 1 : 0, '\0');
        if (n > 0) WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
        return s;
    }

    std::string ShowOpenFileDialog(HWND /*hwnd*/) {
        // 现代 Explorer 风格对话框（高 DPI 清晰，归属主窗口）
        static const GuiFileFilter filters[] = {
            {L"所有支持 (*.log;*.??O)", L"*.log;*.??O"},
            {L"OEM7 日志 (*.log)", L"*.log"},
            {L"RINEX 观测 (*.??O)", L"*.??O"},
            {L"所有文件 (*.*)", L"*.*"}
        };
        std::wstring path;
        if (::ShowOpenFileDialog(path, filters, 4)) return WideToAcp(path);
        return "";
    }

    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND /*hwnd*/) {
        std::string dn = task->fileName;
        if (auto dp = dn.rfind('.'); dp != std::string::npos) dn = dn.substr(0, dp);
        dn += "_spp.csv";
        if (dn.size() >= MAX_PATH) dn = "spp_results.csv";
        // 默认文件名 -> 宽字符
        int wn = MultiByteToWideChar(CP_ACP, 0, dn.c_str(), -1, nullptr, 0);
        std::wstring wDefName(wn > 0 ? wn - 1 : 0, L'\0');
        if (wn > 0) MultiByteToWideChar(CP_ACP, 0, dn.c_str(), -1, wDefName.data(), wn);

        static const GuiFileFilter filters[] = {
            {L"CSV (*.csv)", L"*.csv"}, {L"所有文件 (*.*)", L"*.*"}
        };
        std::wstring path;
        if (!ShowSaveFileDialog(path, filters, 2, wDefName.c_str(), L"csv")) return;

        std::lock_guard lk(task->mutex);
        std::ofstream out(std::filesystem::path(path), std::ios::out);
        if (!out.is_open()) return;
        out <<
                "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-X/m,REF-Y/m,REF-Z/m,EAST/m,NORTH/m,UP/m,B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
        for (auto &r: task->epochs) {
            out << r.week << ',' << std::fixed << std::setprecision(3) << r.sow << ',';
            if (r.solved) {
                auto &res = r.sppResult;
                auto enu = XYZtoENU(res.xyz, task->refECEF);
                out << std::setprecision(4) << res.xyz[0] << ',' << res.xyz[1] << ',' << res.xyz[2] << ',' << task->refECEF.X() << ',' <<
                        task->refECEF.Y() << ',' << task->refECEF.Z() << ',' << enu.E() << ',' << enu.N() << ',' << enu.U() << ',' <<
                        std::setprecision(8) << res.blh[0] * RAD_TO_DEG << ',' << res.blh[1] * RAD_TO_DEG << ',' << std::setprecision(3) <<
                        res.blh[2] << ',' << res.vel[0] << ',' << res.vel[1] << ',' << res.vel[2] << ',' << std::setprecision(4) << res.pdop
                        << ',' << res.gdop << ',' << res.hdop << ',' << res.vdop << ',' << res.tdop << ',' << res.sigmaP << ',' << res.
                        sigmaV << ',' << r.numSatsResult;
            }
            out << '\n';
        }
    }
} // namespace GuiFileProcessor
