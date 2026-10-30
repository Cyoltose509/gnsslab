#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "GuiSppProcessor.h"
#include "GuiHelpers.h"
#include "core/PathMemory.h"
#include "imgui.h"
#include "Const.h"
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>
#include "Log.h"
#include "GuiRealtime.h"

namespace Solvers::SppProcessor {
    void SppTask::renderConfigPanel() {
        Helpers::beginConfigBody("处理配置###cfg_", *this);

        if (ImGui::BeginTabBar("##spp_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                Helpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &core->obsPathBuf, "spp_obs",
                                    Helpers::fObs, 3, {}, "spp_obs", &core->brdcPathBuf);
                ImGui::Separator();
                Helpers::fileRow("广播星历 BRDC", &core->brdcPathBuf, "spp_brdc", Helpers::fRnx, 2, {}, "spp_obs");
                ImGui::Separator();

                Helpers::renderCutoffAndSystems(&core->cutoffDeg, &core->enabledSystems, &core->sampleInterval);

                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (Helpers::beginFileSolve(this, core->obsPathBuf)) {
                        const std::string &path = core->obsPathBuf;
                        AppConfig::PathMemory::instance().set("spp_obs", path);
                        AppConfig::PathMemory::instance().set("spp_brdc", core->brdcPathBuf);
                        AppConfig::PathMemory::instance().setNum("spp_cutoff", core->cutoffDeg);
                        AppConfig::PathMemory::instance().setNum("spp_interval", core->sampleInterval);
                        AppConfig::PathMemory::instance().set("spp_systems", GnssTask::sysSetToString(core->enabledSystems));
                        Helpers::saveSolverParams(GnssTask::SolverMode::Spp, core->opts);
                        LOG_INFO << "[SPP] 开始解算: 观测=" << path;
                        this->worker = std::thread(SolveThread, std::static_pointer_cast<SppTask>(shared_from_this()));
                    }
                }
                ImGui::SameLine();
                Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("实时")) {
                const auto &pm = AppConfig::PathMemory::instance();
                Helpers::renderRealtimeConfig(
                    this, &core->rtEndpoints, {"流动站"},
                    &core->cutoffDeg, &core->enabledSystems, &core->sampleInterval,
                    [&] {
                        const std::string &ip = core->rtEndpoints[0].ip;
                        const std::string &port = core->rtEndpoints[0].port;
                        core->isRealtime = true;
                        isRealtime = true;
                        core->fileName = ip + ":" + port;
                        LOG_INFO << "[SPP] 开始实时解算: " << ip << ":" << port;
                        pm.set("rt_ip", ip);
                        pm.set("rt_port", port);
                        pm.setNum("spp_cutoff", core->cutoffDeg);
                        pm.setNum("spp_interval", core->sampleInterval);
                        pm.set("spp_systems", GnssTask::sysSetToString(core->enabledSystems));
                        Helpers::saveSolverParams(GnssTask::SolverMode::Spp, core->opts);
                        state = State::Running;
                        loading = true;
                        core->loading = true;
                        if (worker.joinable()) worker.join();
                        this->stop = false;
                        core->stop = false;
                        this->worker = std::thread(Realtime::RunSpp,
                                                   std::static_pointer_cast<SppTask>(this->shared_from_this()),
                                                   core->rtEndpoints[0]);
                    });
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, GnssTask::SolverMode::Spp);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        Helpers::endConfigBody();
    }

    bool SppTask::initSolver(void *solver) {
        SPP &spp = *static_cast<SPP *>(solver);
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


    void SppTask::beforeSolveEpoch(void *solver, GnssTask::ObsItem *item) {
        SPP &spp = *static_cast<SPP *>(solver);
        const auto &[obs, eph] = *item;
        if (eph) {
            const EphemerisTable *tbl = eph.get();
            if (tbl->gps.empty() && tbl->bds.empty() && lastGoodEph) tbl = lastGoodEph.get();
            else if (!tbl->gps.empty() || !tbl->bds.empty()) lastGoodEph = eph;
            spp.ephTable = *tbl;
        }
        if (!core->isRinex && eph) core->hasNav = true;
        if (core->isRinex && !core->hasNav) core->noEphSolve = true;
    }

    static void SolverThread(const std::shared_ptr<SppTask> &task) {
        SPP spp;
        GnssTask::RunSingleStationSolve<SPP>(task->core, spp, task);
    }

    void SolveThread(const std::shared_ptr<SppTask> &task) {
        GnssTask::RunDualThreadJob(task, "SPP", SolverThread);
    }
}
