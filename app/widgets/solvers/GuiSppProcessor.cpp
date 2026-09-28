#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "GuiSppProcessor.h"
#include "GuiHelpers.h"
#include "core/AppConfig.h"
#include "ui/Gui.h"
#include "imgui.h"
#include "QualityControl.h"
#include "Const.h"
#include "QCProcessor.h"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>
#include "Log.h"
#include "OEM7Reader.h"
#include "GuiRealtimeProcessor.h"
#include "StringUtils.h"
#include "GuiSolverPanel.h"

namespace GuiSppProcessor {
    void SppTask::renderConfigPanel() {
        GuiHelpers::beginConfigWindow("处理配置###cfg_", *this);

        if (ImGui::BeginTabBar("##cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                GuiHelpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &core->obsPathBuf, "spp_obs",
                                    GuiHelpers::fObs, 3,
                                    [&](const std::string &p) { core->navFiles = GnssTask::ScanNavFiles(p); });
                ImGui::Separator();
                ImGui::Text("广播星历文件:");
                {
                    for (size_t i = 0; i < core->navFiles.size(); i++) {
                        ImGui::Bullet();
                        std::string shortName = core->navFiles[i];
                        if (const auto pos = shortName.find_last_of("\\/"); pos != std::string::npos)
                            shortName = shortName.substr(pos + 1);
                        ImGui::Text("%s", shortName.c_str());
                        ImGui::SameLine();
                        if (ImGui::SmallButton(("删除##nav" + std::to_string(i)).c_str()))
                            core->navFiles.erase(core->navFiles.begin() + i);
                    }
                }
                if (ImGui::Button("添加星历(多选)...")) {
                    if (std::vector<std::wstring> wpaths; ShowOpenFilesDialog(wpaths, GuiHelpers::fRnx, 2)) {
                        for (auto &wp: wpaths) {
                            std::string p = wideToAcp(wp);
                            if (std::find(core->navFiles.begin(), core->navFiles.end(), p) == core->navFiles.end())
                                core->navFiles.push_back(p);
                            AppConfig::instance().addRecent("nav", p);
                        }
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("从记录添加")) ImGui::OpenPopup("##nav_hist_pop");
                if (ImGui::BeginPopup("##nav_hist_pop")) {
                    const auto recent = AppConfig::instance().getRecent("nav");
                    if (recent.empty()) ImGui::TextDisabled("(暂无记录)");
                    for (auto &r: recent) {
                        if (ImGui::Selectable(r.c_str())) {
                            if (std::find(core->navFiles.begin(), core->navFiles.end(), r) == core->navFiles.end())
                                core->navFiles.push_back(r);
                            AppConfig::instance().addRecent("nav", r);
                        }
                    }
                    ImGui::EndPopup();
                }
                ImGui::Separator();

                GuiHelpers::renderCutoffAndSystems(&core->cutoffDeg, &core->enabledSystems);

                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (const std::string path = core->obsPathBuf; path.empty()) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
                    } else {
                        core->filePath = path;
                        core->fileName = GuiHelpers::baseName(path);
                        core->isRinex = GuiHelpers::isRinexObsPath(path);
                        state = State::Running;
                        loading = true;
                        core->loading = true;
                        if (worker.joinable()) worker.join();
                        core->stop = false;
                        this->worker = std::thread(SolveThread, std::static_pointer_cast<SppTask>(shared_from_this()));
                    }
                }
                ImGui::SameLine();
                GuiHelpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("实时")) {
                ImGui::Text("连接到 Oem7 实时流 (socket):");
                ImGui::PushItemWidth(200);
                GuiHelpers::inputTextStd("IP 地址", &core->rtIpBuf);
                GuiHelpers::inputTextStd("端口", &core->rtPortBuf);
                ImGui::PopItemWidth();
                ImGui::Separator();

                GuiHelpers::renderCutoffAndSystems(&core->cutoffDeg, &core->enabledSystems);

                ImGui::Separator();
                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    const std::string &ip = core->rtIpBuf;
                    const std::string &port = core->rtPortBuf;
                    core->isRealtime = true;
                    isRealtime = true;
                    core->fileName = ip + ":" + port;
                    AppConfig::instance().set("rt_ip", ip);
                    AppConfig::instance().set("rt_port", port);
                    GuiRealtimeProcessor::ConnectionConfig config;
                    config.ip = ip;
                    config.port = std::atoi(port.c_str()); //NOLINT
                    state = State::Running;
                    loading = true;
                    core->loading = true;
                    if (worker.joinable()) worker.join();
                    core->stop = false;
                    this->worker = std::thread(GuiRealtimeProcessor::SolveRealtimeThread, this->core, config);
                }
                ImGui::SameLine();
                if (ImGui::Button("取消", ImVec2(80, 40))) {
                    state = State::Done;
                    core->stop = true;
                    core->setError("用户取消");
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }

    static void ReaderThread(const std::shared_ptr<GnssTask::SolveTask> &task) {
        try {
            const auto &path = task->filePath;
            const bool isRinex = task->isRinex;
            LOG_INFO << "SPP 读取文件: " << path;

            if (isRinex) {
                if (const int ok = GnssTask::ReadRinexObsToQueue(task, task->stop); ok <= 0) {
                    task->loading = false;
                    return;
                }
            } else {
                {
                    std::lock_guard lk(task->qcInputMutex);
                    task->qcInput.clear(); // 重新收集原始观测，供质量分析（与解算解耦）
                }
                OEM7Reader oem7;
                if (!oem7.open(path)) {
                    task->setError("无法打开 OEM7");
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
                    {
                        std::lock_guard lk(task->qcInputMutex);
                        task->qcInput.push_back(QC::makeQCObsEpoch(obs, obs.weekSecond.sow));
                    }
                    {
                        std::lock_guard lk(task->queueMutex);
                        task->obsQueue.push({std::move(obs), std::move(eph)});
                    }
                    task->queueCv.notify_one();
                    ok++;
                }
                if (ok == 0) {
                    task->setError("OEM7 无历元");
                }
                task->totalEpochs = ok;
                task->hasNav = ok > 0; // OEM7 内含星历
                task->readDone = true;
                task->queueCv.notify_all();
                GuiSolverPanel::LaunchQC(task);
            }

            task->readDone = true;
            task->queueCv.notify_all();

            GuiSolverPanel::LaunchQC(task);
        } catch (const std::exception &e) {
            task->setError(e.what());
        }
        task->loading = false;
    }

    static void SolverThread(const std::shared_ptr<GnssTask::SolveTask> &task) {
        std::shared_ptr<EphemerisTable> lastGoodEph; // 持有最近一份有效星历，避免指向已销毁 item.eph 的裸指针悬垂
        GnssTask::RunSingleStationSolve<SPP>(task,
                                             [&](SPP &spp) {
                                                 for (const auto &np: task->navFiles) {
                                                     try {
                                                         spp.loadBrdc(np);
                                                         task->hasNav = true;
                                                         LOG_INFO << "SPP 读取广播星历: " << np;
                                                     } catch (const std::exception &e) {
                                                         LOG_ERROR << "SPP 读取广播星历失败 " << np << ": " << e.what();
                                                     }
                                                 }
                                                 return true;
                                             },
                                             [&](SPP &spp, GnssTask::ObsItem &item, GnssTask::EpochData &data, int) {
                                                 if (item.eph) {
                                                     const EphemerisTable *tbl = item.eph.get();
                                                     if (tbl->gps.empty() && tbl->bds.empty() && lastGoodEph) tbl = lastGoodEph.get();
                                                     else if (!tbl->gps.empty() || !tbl->bds.empty()) lastGoodEph = item.eph;
                                                     spp.ephTable = *tbl; // shared_ptr 元素深拷贝廉价
                                                 }
                                                 if (!task->isRinex && item.eph) task->hasNav = true;
                                                 if (task->isRinex && !task->hasNav) task->noEphSolve = true;

                                                 if (!spp.processEpoch(item.obs)) return false;
                                                 data.getFromSPP(spp);
                                                 if (!task->initializedRefECEF) {
                                                     task->refECEF = data.sppResult.xyz;
                                                     task->initializedRefECEF = true;
                                                 }
                                                 return true;
                                             });
    }

    void SolveThread(const std::shared_ptr<SppTask> &task) {
        std::thread thRead(ReaderThread, task->core);
        std::thread thSolve(SolverThread, task->core);
        thRead.join();
        thSolve.join();
        task->state = SppTask::State::Done;
        task->done = true;
        task->loading = false;
        task->core->loading = false;
        task->core->done = true;
    }
}
