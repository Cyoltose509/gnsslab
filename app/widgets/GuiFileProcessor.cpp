#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include "GuiFileProcessor.h"
#include "GuiHelpers.h"   // InputTextStd（绑定 std::string）
#include "core/AppConfig.h"   // 项目外 ini 记忆（不同页面用不同 key/group）
#include "ui/Gui.h"        // 现代 IFileDialog 文件对话框（DPI 清晰）
#include "imgui.h"
#include "OEM7Reader.h"
#include "QualityControl.h"
#include "Const.h"
#include "QCProcessor.h"     // namespace QC: QualityReport / QCObsEpoch / compute

#include <fstream>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <set>
#include <algorithm>
#include <cctype>
#include <thread>
#include <utility>

#include "Log.h"
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "GuiRealtimeProcessor.h"   // 配置面板「实时」页用到其 ConnectionConfig / SolveRealtimeThread
#include "StringUtils.h"

namespace GuiFileProcessor {
    void SppEpochData::getFromSPP(const SPP &spp) {
        if (auto &result = spp.result; result.numSats > 0) {
            solved = true;
            sppResult = result;
            const auto size = static_cast<int>(satIds.size());
            for (int i = 0; i < size; i++) {
                if (auto it = spp.satElevData.find(satIds[i]); it != spp.satElevData.end())
                    elevations[i] = it->second;
                if (auto it2 = spp.satAzimData.find(satIds[i]); it2 != spp.satAzimData.end())
                    azimuths[i] = it2->second;
                if (auto it3 = spp.satPVTTransTime.find(satIds[i]); it3 != spp.satPVTTransTime.end())
                    satPVTs[i] = it3->second;
                if (spp.satRejected.count(satIds[i]))
                    rejected[i] = true;
                // 没有有效星历/位置的卫星也标记为排除，避免 GUI 显示"参与"但坐标全 0
                if (!rejected[i] && satPVTs[i].p.squaredNorm() <= 1.0)
                    rejected[i] = true;
            }
            // 实际参与解算的卫星数 = 未被排除的观测卫星（避免 result.numSats 与 rejected 重复扣减）
            numSatsResult = 0;
            for (int i = 0; i < size; i++)
                if (!rejected[i]) numSatsResult++;
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

    // 把单历元原始观测转成 QC 输入由 QC::makeQCObsEpoch 统一完成（见 QCProcessor.h），三端共用。

    // ----------------------------------------------------------
    // 配置面板：文件 / 实时 双标签页
    // ----------------------------------------------------------
    void RenderConfigPanel(const std::shared_ptr<SppTask> &task) {
        // 每帧强制夺焦：避免点击背景失焦；但弹窗(历史下拉等)打开时让出焦点，否则会抢走下拉菜单。
        GuiHelpers::beginConfigWindow("处理配置###cfg_", task.get());

        if (ImGui::BeginTabBar("##cfg_tabs")) {
            // ===================== 文件标签页 =====================
            if (ImGui::BeginTabItem("文件")) {
                // ini key 必须用 "obs_path"，与 Application::OpenSppSolve 读回的 key 一致，
                // 否则 obs 文件选择不会被记住（PPP 正是用一致的 "ppp_obs" 键写/读，故 PPP 能记忆）。
                GuiHelpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &task->obsPathBuf, "obs_path",
                                    GuiHelpers::fObs, 3,
                                    [&](const std::string &p) { task->navFiles = ScanNavFiles(p); });
                ImGui::Separator();
                ImGui::Text("广播星历文件:");
                if (task->navFiles.empty()) {
                    ImGui::TextColored(ImVec4(1, 0.5f, 0, 1),
                                       "未识别到伴生星历（同目录下 .??N/.??G/.??C 等）。可手动添加。");
                } else {
                    for (size_t i = 0; i < task->navFiles.size(); i++) {
                        ImGui::Bullet();
                        std::string shortName = task->navFiles[i];
                        if (const auto pos = shortName.find_last_of("\\/"); pos != std::string::npos)
                            shortName = shortName.substr(pos + 1);
                        ImGui::Text("%s", shortName.c_str());
                        ImGui::SameLine();
                        if (ImGui::SmallButton(("删除##nav" + std::to_string(i)).c_str()))
                            task->navFiles.erase(task->navFiles.begin() + i);
                    }
                }
                if (ImGui::Button("添加星历(多选)...")) {
                    std::vector<std::wstring> wpaths;
                    if (ShowOpenFilesDialog(wpaths, GuiHelpers::fRnx, 2)) {
                        for (auto &wp: wpaths) {
                            std::string p = wideToAcp(wp);
                            if (std::find(task->navFiles.begin(), task->navFiles.end(), p) == task->navFiles.end())
                                task->navFiles.push_back(p);
                            AppConfig::instance().addRecent("nav", p);
                        }
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("从记录添加")) ImGui::OpenPopup("##nav_hist_pop");
                if (ImGui::BeginPopup("##nav_hist_pop")) {
                    auto recent = AppConfig::instance().getRecent("nav");
                    if (recent.empty()) ImGui::TextDisabled("(暂无记录)");
                    for (auto &r: recent) {
                        if (ImGui::Selectable(r.c_str())) {
                            if (std::find(task->navFiles.begin(), task->navFiles.end(), r) == task->navFiles.end())
                                task->navFiles.push_back(r);
                            AppConfig::instance().addRecent("nav", r);
                        }
                    }
                    ImGui::EndPopup();
                }
                ImGui::Separator();

                GuiHelpers::renderCutoffAndSystems(&task->cutoffDeg, &task->enabledSystems);

                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (std::string path = task->obsPathBuf; path.empty()) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
                    } else {
                        task->filePath = path;
                        task->fileName = GuiHelpers::baseName(path);
                        task->isRinex = GuiHelpers::isRinexObsPath(path);
                        task->state = SppTask::State::Running;
                        task->loading = true;
                        task->worker = std::thread(SolveThread, task);
                    }
                }
                ImGui::SameLine();
                GuiHelpers::renderCancelButton(task);
                ImGui::EndTabItem();
            }

            // ===================== 实时标签页 =====================
            if (ImGui::BeginTabItem("实时")) {
                ImGui::Text("连接到 Oem7 实时流 (socket):");
                ImGui::PushItemWidth(200);
                GuiHelpers::inputTextStd("IP 地址", &task->rtIpBuf);
                GuiHelpers::inputTextStd("端口", &task->rtPortBuf);
                ImGui::PopItemWidth();
                ImGui::Separator();

                // 截止高度角 + 星座选择（文件页与实时页共用同一控件，保证一致）
                GuiHelpers::renderCutoffAndSystems(&task->cutoffDeg, &task->enabledSystems);

                ImGui::Separator();
                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    std::string &ip = task->rtIpBuf;
                    std::string &port = task->rtPortBuf;
                    task->isRealtime = true;
                    task->fileName = ip + ":" + port;
                    AppConfig::instance().set("rt_ip", ip);
                    AppConfig::instance().set("rt_port", port);
                    GuiRealtimeProcessor::ConnectionConfig config;
                    config.ip = ip;
                    config.port = std::atoi(port.c_str()); //NOLINT
                    task->state = SppTask::State::Running;
                    task->loading = true;
                    task->worker = std::thread(GuiRealtimeProcessor::SolveRealtimeThread, task, config);
                }
                ImGui::SameLine();
                if (ImGui::Button("取消", ImVec2(80, 40))) {
                    task->state = SppTask::State::Done;
                    task->hasError = true;
                    task->errorMsg = "用户取消";
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }

    // ----------------------------------------------------------
    // 读取线程（生产者）：加载星历 → navReady → 流式把历元喂入队列
    // ----------------------------------------------------------
    static void ReaderThread(const std::shared_ptr<SppTask> &task) {
        try {
            const auto &path = task->filePath;
            const bool isRinex = task->isRinex;
            LOG_INFO << "读取文件: " << path;
            task->qcInput.clear(); // 重新收集原始观测，供质量分析（与解算解耦）

            if (isRinex) {
                // 广播星历改由解算线程 SPP::loadBrdc 在类内加载（与 PPP 一致），读取线程仅发 navReady 信号
                {
                    std::lock_guard lk(task->queueMutex);
                    task->navReady = true;
                }
                task->queueCv.notify_all();

                RinexObsReader obsReader;
                std::fstream obsFile(path.c_str(), std::ios::in);
                if (!obsFile) {
                    task->hasError = true;
                    task->errorMsg = "无法打开 RINEX";
                    task->readDone = true;
                    task->queueCv.notify_all();
                    task->loading = false;
                    return;
                }
                obsReader.pFileStream = &obsFile;
                int safety = 0, ok = 0;
                while (++safety < 100000) {
                    if (task->stop) break;
                    try {
                        ObsData o = obsReader.parseRinexObs();
                        task->qcInput.push_back(QC::makeQCObsEpoch(o, o.weekSecond.sow)); // 原始观测 → QC 输入
                        {
                            std::lock_guard lk(task->queueMutex);
                            task->obsQueue.push({std::move(o), nullptr});
                        }
                        task->queueCv.notify_one();
                        ok++;
                    } catch (const EndOfFile &) { break; } catch (const std::exception &e) {
                        task->hasError = true;
                        task->errorMsg = "RINEX parse err: " + std::string(e.what());
                        task->readDone = true;
                        task->queueCv.notify_all();
                        task->loading = false;
                        return;
                    }
                }
                if (ok == 0) {
                    task->hasError = true;
                    task->errorMsg = "RINEX 无历元";
                    task->readDone = true;
                    task->queueCv.notify_all();
                    task->loading = false;
                    return;
                }
                task->totalEpochs = ok;
            } else {
                OEM7Reader oem7;
                if (!oem7.open(path)) {
                    task->hasError = true;
                    task->errorMsg = "无法打开 OEM7";
                    task->readDone = true;
                    task->queueCv.notify_all();
                    task->loading = false;
                    return;
                }
                {
                    std::lock_guard lk(task->queueMutex);
                    task->navReady = true;
                }
                task->queueCv.notify_all();

                ObsData obs;
                int ok = 0;
                while (oem7.getNextEpoch(obs)) {
                    if (task->stop) break;
                    auto eph = std::make_shared<EphemerisTable>();
                    for (auto &[prn, e]: oem7.latestGps) eph->gps[prn] = {std::make_shared<GPSEphem>(e)};
                    for (auto &[prn, e]: oem7.latestBds) eph->bds[prn] = {std::make_shared<BDSEphem>(e)};
                    task->qcInput.push_back(QC::makeQCObsEpoch(obs, obs.weekSecond.sow));
                    {
                        std::lock_guard lk(task->queueMutex);
                        task->obsQueue.push({std::move(obs), std::move(eph)});
                    }
                    task->queueCv.notify_one();
                    ok++;
                }
                if (ok == 0) {
                    task->hasError = true;
                    task->errorMsg = "OEM7 无历元";
                }
                task->totalEpochs = ok;
                task->hasNav = ok > 0; // OEM7 内含星历
                task->readDone = true;
                task->queueCv.notify_all();
                LaunchQC(task);
            }

            task->readDone = true;
            task->queueCv.notify_all();

            LaunchQC(task);
        } catch (const std::exception &e) {
            task->hasError = true;
            task->errorMsg = e.what();
        }
        task->loading = false;
    }

    static void SolverThread(const std::shared_ptr<SppTask> &task) {
        try {
            {
                std::unique_lock lk(task->queueMutex);
                task->queueCv.wait(lk, [&] { return task->navReady.load() || task->stop.load(); });
            }
            if (task->stop) {
                task->solvingDone = true;
                return;
            }

            SPP spp;
            spp.setCutoffElevDeg(task->cutoffDeg); // 截止高度角：低于该仰角的卫星在解算中被剔除
            spp.enabledSystems = task->enabledSystems; // IF 组合由 SPP::solve 每历元从观测自探测（与 PPP/LEO 同构）

            // 文件模式广播星历：由 SPP 类内加载（与 PPP 一致），GUI 不再直达 ephTable
            for (auto &np : task->navFiles) {
                try { spp.loadBrdc(np); task->hasNav = true; }
                catch (const std::exception &e) { LOG_ERROR << "nav fail " << np << ": " << e.what(); }
            }

            std::shared_ptr<EphemerisTable> lastGoodEph;  // 持有最近一份有效星历，避免指向已销毁 item.eph 的裸指针悬垂

            while (true) {
                ObsItem item;
                {
                    std::unique_lock lk(task->queueMutex);
                    task->queueCv.wait(lk, [&] {
                        return !task->obsQueue.empty() || task->readDone.load() || task->stop.load();
                    });
                    if (task->obsQueue.empty() && (task->readDone.load() || task->stop.load())) break;
                    if (task->obsQueue.empty()) continue;
                    item = std::move(task->obsQueue.front());
                    task->obsQueue.pop();
                }
                if (task->stop) break;

                ObsData &obs = item.obs;
                // ephTable 为值存储：流式模式每历元把 item.eph 拷入并覆盖，文件模式保留 loadBrdc 已加载的广播星历
                if (item.eph) {
                    EphemerisTable *tbl = item.eph.get();
                    if (tbl->gps.empty() && tbl->bds.empty() && lastGoodEph) tbl = lastGoodEph.get();
                    else if (!tbl->gps.empty() || !tbl->bds.empty()) lastGoodEph = item.eph;
                    spp.ephTable = *tbl;   // 逐历元覆盖（shared_ptr 元素深拷贝廉价）
                }
                EphemerisTable *eph = &spp.ephTable;   // 统一指向类内值成员
                spp.preprocess(obs);

                SppEpochData data;
                data.getFromObs(obs);
                bool solve_ok = false;
                if (eph) {
                    try {
                        spp.solve(obs);
                        solve_ok = true;
                    } catch (...) { solve_ok = false; }
                }

                if (!task->isRinex) task->hasNav = task->hasNav || item.eph != nullptr;
                if (task->isRinex && !task->hasNav) task->noEphSolve = true;

                {
                    std::lock_guard lk(task->mutex);
                    if (solve_ok) {
                        data.getFromSPP(spp);
                        if (!task->initializedRefECEF) {
                            task->refECEF = data.sppResult.xyz;
                            task->initializedRefECEF = true;
                        }
                    } else {
                        data.solved = false;
                    }
                    const int idx = static_cast<int>(task->epochs.size());
                    task->epochs.push_back(std::move(data));
                    {
                        std::lock_guard plk(task->plotMutex);
                        task->plotData.insert(idx, task->epochs.back(), task->refECEF);
                    }
                    if (task->selectedEpoch == -1 || task->selectedEpoch == idx - 1) task->selectedEpoch = idx;
                    task->solvingProgress = idx + 1;
                    task->solvedCount = static_cast<int>(task->epochs.size());
                }
            }
        } catch (const std::exception &e) {
            task->hasError = true;
            task->errorMsg = e.what();
        }
        task->solvingDone = true;
    }

    void SolveThread(const std::shared_ptr<SppTask> &task) {
        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();
        task->state = SppTask::State::Done;
        task->done = true;
        task->loading = false;
    }

    static QualityReport buildReport(const SppTask &task) {
        const std::vector<QC::QCObsEpoch> qcEpochs = task.qcInput;
        QualityReport rep = QC::compute(qcEpochs);
        rep.totalInputEpochs = static_cast<int>(qcEpochs.size());
        return rep;
    }

    void LaunchQC(const std::shared_ptr<SppTask> &task) {
        if (task->qcWorker.joinable()) task->qcWorker.join(); // 确保上一轮已结束，再起新的一轮
        task->qcComputing = true;
        task->qcWorker = std::thread([task] {
            try {
                {
                    QualityReport rep = buildReport(*task);
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

    static void EnsureSkyTracksBuilt(const std::shared_ptr<SppTask> &task) {
        {
            int totalEp = 0;
            std::lock_guard lk(task->mutex);
            totalEp = static_cast<int>(task->epochs.size());
            for (int e = task->skyTracksBuilt; e < totalEp; ++e) {
                auto &ep = task->epochs[e];
                for (int j = 0; j < static_cast<int>(ep.satIds.size()); ++j) {
                    const double el = ep.elevations[j] * RAD_TO_DEG;
                    if (el <= 0.0) continue;
                    task->skyTracks[ep.satIds[j]].push_back(
                        {e, static_cast<float>(ep.azimuths[j] * RAD_TO_DEG), static_cast<float>(el), ep.solved && !ep.rejected[j]});
                }
            }
            task->skyTracksBuilt = totalEp;
        }
    }

    static std::vector<std::pair<SatID, GuiCharts::SkyPoint> > BuildCurrentSkyPoints(
        const std::shared_ptr<SppTask> &task, const int sel) {
        std::vector<std::pair<SatID, GuiCharts::SkyPoint> > curPts;
        std::lock_guard lk(task->mutex);
        if (sel < 0 || sel >= static_cast<int>(task->epochs.size())) return curPts;
        auto &ep = task->epochs[sel];
        return GuiCharts::buildCurSkyPoints(sel, ep.satIds.size(),
                                            [&](size_t i) {
                                                GuiCharts::SkyView v;
                                                v.sat = ep.satIds[i];
                                                v.azimRad = ep.azimuths[i];
                                                v.elevRad = ep.elevations[i];
                                                v.used = ep.solved && !ep.rejected[i];
                                                return v;
                                            }, 0.0);
    }

    namespace {
        int SehFilter(EXCEPTION_POINTERS *ep, DWORD &outCode, uintptr_t &outAddr) {
            outCode = ep->ExceptionRecord->ExceptionCode;
            outAddr = reinterpret_cast<uintptr_t>(ep->ExceptionRecord->ExceptionAddress);
            return EXCEPTION_EXECUTE_HANDLER;
        }
    }

    void RenderTaskImpl(const std::shared_ptr<SppTask> &task, const bool isRealtime);

    void RenderTask(const std::shared_ptr<SppTask> &task, const bool isRealtime) {
        DWORD code = 0;
        uintptr_t addr = 0;
        __try {
            RenderTaskImpl(task, isRealtime);
        } __except(SehFilter(GetExceptionInformation(), code, addr)) {
            // 调试器下会 first-chance 暂停；点"继续"后才会进入此处。
            // 直接退出本帧渲染，让外层 EndTabItem/EndTabBar 恢复栈。
            ImGui::TextColored(ImVec4(1, 0.2f, 0.2f, 1),
                               "定位页结构化异常 SEH 0x%08X @ 0x%p", code, reinterpret_cast<void *>(addr));
            ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1),
                               "调试器弹窗请点\"继续\"，本帧会被安全跳过。");
        }
    }

    void RenderTaskImpl(const std::shared_ptr<SppTask> &task, const bool isRealtime) {
        if (task->state == SppTask::State::Config) {
            RenderConfigPanel(task);
            return;
        }

        const bool isLoading = task->loading.load();
        const bool isDone = task->done.load();
        const bool hasError = task->hasError;
        auto [epochCount, selectedIdx] = GuiHelpers::readEpochView(task);

        // 把 selectedEpoch 从共享状态解耦：渲染期间只操作局部副本，
        // 避免解算线程在渲染中途改写 selectedEpoch 导致 Slider/DragLine 越界或数据不一致。
        int selectedEpochLocal = selectedIdx;

        if (!isRealtime && isLoading && !hasError) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f),
                               "正在处理 [%s]: 已解算 %d 历元",
                               task->fileName.c_str(), task->solvedCount.load());
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem("定位解算")) {
                GuiCharts::PosTabView v;
                v.epochCount = epochCount;
                v.selectedEpoch = &selectedEpochLocal;
                v.selectedSatIdx = &task->selectedSatIdx;
                v.refECEF = &task->refECEF;
                v.onRefChanged = [&] { task->plotData.refreshENU(task->epochs, task->refECEF); };

                if (hasError) {
                    v.statusText = std::string("状态: ") + task->errorMsg;
                    v.statusColor = ImVec4(1, 0.3f, 0.3f, 1);
                } else if (epochCount > 0) {
                    v.statusText = "解算：共 " + std::to_string(epochCount) + " 个历元";
                    v.statusColor = ImVec4(1, 0.6f, 0.2f, 1);
                } else if (isLoading) {
                    v.statusText = "等待加载... ";
                }

                // 天顶图轨迹增量构建（独立于下方锁，内部自行持锁）
                if (epochCount > 0) EnsureSkyTracksBuilt(task);

                if (epochCount > 0 && selectedIdx >= 0) {
                    std::lock_guard lk(task->mutex);
                    auto &cur = task->epochs[selectedIdx]; //NOLINT

                    v.satRows = GuiCharts::buildSatRows(cur.satIds.size(),
                                                        [&](const size_t i) {
                                                            GuiCharts::SatRowView _v;
                                                            _v.sat = cur.satIds[i];
                                                            _v.used = !cur.rejected[i] && cur.solved;
                                                            _v.elevRad = cur.elevations[i];
                                                            _v.azimRad = cur.azimuths[i];
                                                            const bool haveXYZ = cur.satPVTs[i].p.squaredNorm() > 1.0;
                                                            _v.hasXYZ = haveXYZ;
                                                            if (haveXYZ) {
                                                                _v.x = cur.satPVTs[i].p[0];
                                                                _v.y = cur.satPVTs[i].p[1];
                                                                _v.z = cur.satPVTs[i].p[2];
                                                            }
                                                            return _v;
                                                        },
                                                        [&](GuiCharts::SatRow &sr, size_t i) {
                                                            const auto it = cur.sppResult.postRes.find(cur.satIds[i]);
                                                            sr.extra.emplace_back(
                                                                "伪距残差(m)", it != cur.sppResult.postRes.end()
                                                                               ? fmt4(it->second)
                                                                               : std::string("-"));
                                                        });
                    v.hasSatRows = true;

                    if (task->selectedSatIdx >= 0 && task->selectedSatIdx < static_cast<int>(cur.satIds.size())) {
                        int si = task->selectedSatIdx;
                        v.showObsDetail = true;
                        v.detailSystem = cur.satIds[si].system;
                        v.detailObs = cur.allObs[si];
                    }

                    if (cur.solved) {
                        const auto &r = cur.sppResult;
                        v.solved = true;
                        v.xyz = r.xyz;
                        v.enu = XYZtoENU(r.xyz, task->refECEF);
                        v.blh = r.blh;
                        v.sigmaP = r.sigmaP;
                        v.showVelDop = true;
                        v.showVel = (r.vel.squaredNorm() > 1e-4);
                        v.vel = r.vel;
                        v.pdop = r.pdop;
                        v.gdop = r.gdop;
                        v.hdop = r.hdop;
                        v.vdop = r.vdop;
                        v.tdop = r.tdop;
                        v.numSatsResult = cur.numSatsResult;
                        v.numObs = cur.numObs;
                    } else {
                        v.noSolveMsg = task->noEphSolve ? "无定位解（缺少星历文件）" : "无定位解";
                    }

                    char buf[160];
                    snprintf(buf, sizeof buf, "Wk %u SOW %.3f | %s", cur.week, cur.sow, cur.solved ? "定位" : "无解");
                    v.epochInfo = buf;
                }

                // 图表数据：在各自锁内快照为值，避免渲染期与解算线程竞争
                if (epochCount > 0 && selectedIdx >= 0) {
                    bool sigmaVNonZero = false;
                    {
                        std::lock_guard plk(task->plotMutex);
                        auto &pp = task->plotData; //NOLINT
                        v.times = pp.times;
                        v.enu_e = pp.enu_e;
                        v.enu_n = pp.enu_n;
                        v.enu_u = pp.enu_u;
                        v.newed = pp.newed;
                        v.fixedY = true; // SPP 收敛到米级，固定 Y 轴 ±1m 便于观察微小漂移
                        v.resT = pp.satResTimes;
                        v.resV = pp.satResVals;
                        if (isDone && !pp.resRangeReady) {
                            auto r = GuiHelpers::computeRobustResRange(pp.satResVals);
                            pp.resYlo = r.lo;
                            pp.resYhi = r.hi;
                            pp.resRangeReady = true;
                        }
                        v.robustRes = pp.resRangeReady;
                        v.resYlo = pp.resYlo;
                        v.resYhi = pp.resYhi;
                        v.sigmaPs = pp.sigmaPs;
                        v.sigmaVs = pp.sigmaVs;
                        sigmaVNonZero = std::any_of(pp.sigmaVs.begin(), pp.sigmaVs.end(),
                                                    [](double x) { return x > 1e-6; });
                        v.pdops = pp.pdops;
                    }
                    {
                        std::lock_guard lk(task->mutex);
                        v.skyTracks = task->skyTracks;
                        // 仅当实际估计了速度（>5% 历元有非零速度）时才显示 SigmaV；
                        // PPP / 静态 SPP 速度为 0，避免画出单个尖峰。
                        int solvedCnt = 0, velCnt = 0;
                        for (const auto &ep : task->epochs) {
                            if (!ep.solved) continue;
                            ++solvedCnt;
                            if (ep.sppResult.vel.squaredNorm() > 1e-4) ++velCnt;
                        }
                        v.showSigmaV = (solvedCnt > 0 && static_cast<double>(velCnt) / solvedCnt > 0.05)
                                       && sigmaVNonZero;
                    }
                v.curSkyPts = BuildCurrentSkyPoints(task, selectedIdx); // 内部自行持锁
                v.showSigmaDop = true;
            }

            // LEO 定轨：把解算器填写的 3D 轨迹 / 参考轨道 / RTN 偏差序列注入 PosTabView。
            // 必须在 task->mutex 下快照：SolverThread 在相同 mutex 内写这些字段。
            {
                std::lock_guard lk(task->mutex);
                if (task->hasLeo) {
                    v.show3d = true;
                    v.showRtn = !task->leoRefTraj.empty();   // 无参考轨道则不画 RTN 序列
                    v.hasRef  = !task->leoRefTraj.empty();    // 无参考轨道则不显示 RTN 面板
                    v.showRefEnu = false;                     // LEO 不编辑/显示参考真值
                    v.leoPos3d = task->leoPos3d;
                    v.leoRef3d = task->leoRef3d;
                    v.leoTraj = task->leoTraj;
                    v.leoRefTraj = task->leoRefTraj;
                    v.gnssVis = task->gnssVis;
                    v.gnssTraj = task->gnssTraj;
                    v.rtnTimes = task->rtnTimes;
                    v.rtn_r = task->rtn_r;
                    v.rtn_t = task->rtn_t;
                    v.rtn_n = task->rtn_n;
                    v.rtn_d3 = task->rtn_d3;
                }
            }

            v.isDone = isDone || isRealtime;
            v.onExportCsv = [&] {
                auto h = static_cast<HWND>(ImGui::GetMainViewport()->PlatformHandleRaw);
                if (!h) h = GetActiveWindow();
                ExportCsv(task, h);
            };

            try {
                GuiCharts::RenderPositioningTab(v);
            } catch (const std::exception &e) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "渲染异常: %s", e.what());
            } catch (...) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "渲染发生未知异常");
            }
            // 把局部选中的历元写回共享状态（锁内），保证下一帧 readEpochView 读取的是一致值。
            if (selectedEpochLocal != selectedIdx) {
                std::lock_guard lk(task->mutex);
                if (selectedEpochLocal >= 0 && selectedEpochLocal < static_cast<int>(task->epochs.size()))
                    task->selectedEpoch = selectedEpochLocal;
            }
            ImGui::EndTabItem();
        }
            if (ImGui::BeginTabItem("质量分析")) {
                try {
                    QualityControl::render(task);
                } catch (const std::exception &e) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "质量分析渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "质量分析渲染发生未知异常");
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }


    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND /*hwnd*/) {
        std::string dn = task->fileName;
        if (const auto dp = dn.rfind('.'); dp != std::string::npos) dn = dn.substr(0, dp);
        dn += "_" + task->processorLabel + "_result.csv";
        if (dn.size() >= MAX_PATH) dn = task->processorLabel + "_result.csv";
        std::wstring path;
        if (!GuiHelpers::saveCSVDialog(dn, path)) return;

        std::lock_guard lk(task->mutex);
        std::ofstream out(std::filesystem::path(path), std::ios::out);
        if (!out.is_open()) return;
        // LEO 导出：逐历元参考轨道(SP3)位置作 REF，ENU 改为 RTN(径向/沿迹/法向)；
        // 无参考轨道时不输出 REF 与 RTN 列。SPP/PPP 保持原 ENU + 常数 REF 行为。
        const bool isLeo = (task->processorLabel == "LEO");
        const bool leoRef = task->hasRefOrbit;
        if (isLeo) {
            if (leoRef)
                out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-X/m,REF-Y/m,REF-Z/m,R/m,T/m,N/m,B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
            else
                out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
        } else {
            out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-X/m,REF-Y/m,REF-Z/m,EAST/m,NORTH/m,UP/m,B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
        }
        for (auto &r: task->epochs) {
            out << r.week << ',' << std::fixed << std::setprecision(3) << r.sow << ',';
            if (r.solved) {
                auto &res = r.sppResult;
                out << std::setprecision(4) << res.xyz[0] << ',' << res.xyz[1] << ',' << res.xyz[2];
                if (isLeo) {
                    // LEO：输出逐历元参考轨道(REF)与 RTN；无参考轨道则两者均略过
                    if (leoRef) {
                        out << ',' << r.refECEF[0] << ',' << r.refECEF[1] << ',' << r.refECEF[2];
                        out << ',' << r.rtnR << ',' << r.rtnT << ',' << r.rtnN;
                    }
                } else {
                    auto enu = XYZtoENU(res.xyz, task->refECEF);
                    out << ',' << task->refECEF.X() << ',' << task->refECEF.Y() << ',' << task->refECEF.Z();
                    out << ',' << enu.E() << ',' << enu.N() << ',' << enu.U();
                }
                out << ',' << std::setprecision(8) << res.blh[0] * RAD_TO_DEG << ',' << res.blh[1] * RAD_TO_DEG << ',' << std::setprecision(3) <<
                        res.blh[2] << ',' << res.vel[0] << ',' << res.vel[1] << ',' << res.vel[2] << ',' << std::setprecision(4) << res.pdop
                        << ',' << res.gdop << ',' << res.hdop << ',' << res.vdop << ',' << res.tdop << ',' << res.sigmaP << ',' << res.
                        sigmaV << ',' << r.numSatsResult;
            }
            out << '\n';
        }
    }
}
