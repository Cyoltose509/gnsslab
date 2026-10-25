#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "GuiSppProcessor.h"
#include "GuiHelpers.h"
#include "core/PathMemory.h"
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
#include "ObsFormat.h"      // classifyObsPath：RINEX/OEM7 三值判别（Unknown 显式报错）
#include "GuiRealtimeProcessor.h"
#include "StringUtils.h"
#include "GuiSolverPanel.h"

namespace Solvers::SppProcessor {
    void SppTask::renderConfigPanel() {
        Solvers::Helpers::beginConfigBody("处理配置###cfg_", *this);

        if (ImGui::BeginTabBar("##cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                Solvers::Helpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &core->obsPathBuf, "spp_obs",
                                    Solvers::Helpers::fObs, 3,
                                    [&](const std::string &p) {
                                        const std::string d = GnssTask::AutoDetectNav(p);
                                        if (!d.empty()) core->brdcPathBuf = d;
                                    });
                ImGui::Separator();
                Solvers::Helpers::fileRow("广播星历 BRDC", &core->brdcPathBuf, "spp_brdc", Solvers::Helpers::fRnx, 2, {}, "spp_obs");
                ImGui::Separator();

                Solvers::Helpers::renderCutoffAndSystems(&core->cutoffDeg, &core->enabledSystems);

                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    const std::string path = core->obsPathBuf;
                    // 格式判别是显式三值：无法识别时明确报错，不再默认落进 OEM7 读取器
                    // （旧行为会把 typo/.txt/.sp3 也送进去，报"无法打开 OEM7"误导用户）
                    const auto fmt = path.empty() ? GnssTask::ObsFormat::Unknown : GnssTask::classifyObsPath(path);
                    if (path.empty()) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
                    } else if (fmt == GnssTask::ObsFormat::Unknown) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1),
                                           "无法识别观测文件格式（支持 RINEX .??O/.rnx/.obs 与 OEM7 .log/.oem7）");
                    } else {
                        // 记忆手打的观测路径（浏览按钮另有 set，但直接输入/粘贴的只有这里落盘）
                        AppConfig::PathMemory::instance().set("spp_obs", path);
                        AppConfig::PathMemory::instance().set("spp_brdc", core->brdcPathBuf);
                        AppConfig::PathMemory::instance().setNum("spp_cutoff", core->cutoffDeg);
                        AppConfig::PathMemory::instance().set("spp_systems", GnssTask::sysSetToString(core->enabledSystems));
                        core->filePath = path;
                        core->fileName = Solvers::Helpers::baseName(path);
                        core->isRinex = (fmt == GnssTask::ObsFormat::Rinex);
                        LOG_INFO << "[SPP] 开始解算: 观测=" << path;
                        state = State::Running;
                        loading = true;
                        core->loading = true;
                        if (worker.joinable()) worker.join();
                        core->stop = false;
                        this->worker = std::thread(SolveThread, std::static_pointer_cast<SppTask>(shared_from_this()));
                    }
                }
                ImGui::SameLine();
                Solvers::Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("实时")) {
                const auto &pm = AppConfig::PathMemory::instance();
                Solvers::Helpers::renderRealtimeConfig(
                    this, &core->rtEndpoints, {"流动站"},
                    &core->cutoffDeg, &core->enabledSystems,
                    [&]() {
                        const std::string &ip = core->rtEndpoints[0].ip;
                        const std::string &port = core->rtEndpoints[0].port;
                        core->isRealtime = true;
                        isRealtime = true;
                        core->fileName = ip + ":" + port;
                        LOG_INFO << "[SPP] 开始实时解算: " << ip << ":" << port;
                        pm.set("rt_ip", ip);
                        pm.set("rt_port", port);
                        pm.setNum("spp_cutoff", core->cutoffDeg);
                        pm.set("spp_systems", GnssTask::sysSetToString(core->enabledSystems));
                        Solvers::RealtimeProcessor::ConnectionConfig config;
                        config.ip = ip;
                        config.port = std::atoi(port.c_str()); //NOLINT
                        state = State::Running;
                        loading = true;
                        core->loading = true;
                        if (worker.joinable()) worker.join();
                        core->stop = false;
                        this->worker = std::thread(Solvers::RealtimeProcessor::SolveRealtimeThread, this->core, config);
                    });
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        Solvers::Helpers::endConfigBody();
    }

    static void ReaderThread(const std::shared_ptr<GnssTask::SolveTask> &task) {
        try {
            const auto &path = task->filePath;
            LOG_INFO << "SPP 读取文件: " << path;

            // RINEX / OEM7 分派已下沉 GnssTask::ReadObsToQueue（PPP/LEO 共用同一份）：
            // 返回值 -1 = 打开或解析失败；0 = 无历元；>0 = 入队历元数。
            if (const int ok = GnssTask::ReadObsToQueue(task, task->stop); ok <= 0) {
                task->loading = false;
                return;
            }

            task->readDone = true;
            task->queueCv.notify_all();

            Solvers::SolverPanel::LaunchQC(task);
        } catch (const std::exception &e) {
            task->setError(e.what());
        }
        task->loading = false;
    }

    // ---- SppTask 的解算钩子（骨架见 RunSingleStationSolve）----

    bool SppTask::initSolver(void *solverPtr) {
        SPP &spp = *static_cast<SPP *>(solverPtr);
        if (!core->brdcPathBuf.empty()) {
            try {
                spp.loadBrdc(core->brdcPathBuf);
                core->hasNav = true;
                LOG_INFO << "SPP 读取广播星历: " << core->brdcPathBuf;
            } catch (const std::exception &e) {
                LOG_ERROR << "SPP 读取广播星历失败 " << core->brdcPathBuf << ": " << e.what();
            }
        }
        return true;
    }

    // 星历必须在 processEpoch 之前注入，故用 beforeSolveEpoch 而非 afterSolveEpoch
    void SppTask::beforeSolveEpoch(void *solverPtr, GnssTask::ObsItem *itemPtr) {
        SPP &spp = *static_cast<SPP *>(solverPtr);
        GnssTask::ObsItem &item = *itemPtr;
        if (item.eph) {
            const EphemerisTable *tbl = item.eph.get();
            if (tbl->gps.empty() && tbl->bds.empty() && lastGoodEph) tbl = lastGoodEph.get();
            else if (!tbl->gps.empty() || !tbl->bds.empty()) lastGoodEph = item.eph;
            spp.ephTable = *tbl; // shared_ptr 元素深拷贝廉价
        }
        if (!core->isRinex && item.eph) core->hasNav = true;
        if (core->isRinex && !core->hasNav) core->noEphSolve = true;
    }

    bool SppTask::afterSolveEpoch(void *, GnssTask::EpochData *data, GnssTask::ObsItem *, int) {
        if (!data->solved) return false;
        if (!core->initializedRefECEF) {
            core->refECEF = data->sppResult.xyz;
            core->initializedRefECEF = true;
        }
        return true;
    }

    static void SolverThread(const std::shared_ptr<SppTask> &task) {
        SPP spp;
        GnssTask::RunSingleStationSolve<SPP>(task->core, spp, task);
    }

    void SolveThread(const std::shared_ptr<SppTask> &task) {
        std::thread thRead(ReaderThread, task->core);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();
        task->state = SppTask::State::Done;
        task->done = true;
        task->loading = false;
        task->core->loading = false;
        task->core->done = true;
    }
}
