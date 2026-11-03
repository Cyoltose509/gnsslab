#include "GuiSPPProcessor.h"
#include "view/GuiHelpers.h"
#include "view/GuiProductSection.h"
#include "view/GuiParamPage.h"
#include "core/PathMemory.h"
#include "core/Products.h"
#include "download/ProductStore.h"
#include "download/ProductCatalog.h"
#include "imgui.h"
#include "io/ProductLoad.h"
#include "ui/FileDialog.h"
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>
#include "Log.h"
#include "solvers/spp/Realtime.h"
#include "solvers/rtk/Realtime.h"

namespace Solvers::SPP {
    void Task::renderConfigPanel() {
        Helpers::beginConfigBody("处理配置###cfg_", *this);

        if (ImGui::BeginTabBar("##spp_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                Helpers::fileRow("观测文件", &core->obsPathBuf, "spp_obs",
                                    FileDialog::fObs, 3, {}, "spp_obs", &core->brdcPathBuf);
                ImGui::Separator();
                Helpers::fileRow("广播星历 BRDC", &core->brdcPathBuf, "spp_brdc", FileDialog::fRnx, 2, {}, "spp_obs");
                ImGui::Separator();

                Helpers::renderSimpleConfig(&core->cutoffDeg, &core->enabledSystems, &core->sampleInterval);

                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (Helpers::beginFileSolve(this, core->obsPathBuf)) {
                        const std::string &path = core->obsPathBuf;
                        AppConfig::PathMemory::instance().set("spp_obs", path);
                        AppConfig::PathMemory::instance().set("spp_brdc", core->brdcPathBuf);
                        Helpers::saveCommonCfg("spp", Solvers::Mode::SPP, core->opts,
                                               core->cutoffDeg, core->sampleInterval, core->enabledSystems);
                        LOG_INFO << "[SPP] 开始解算: 观测=" << path;
                        this->startFile();
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
                        Helpers::saveCommonCfg("spp", Solvers::Mode::SPP, core->opts,
                                               core->cutoffDeg, core->sampleInterval, core->enabledSystems);
                        this->startRealtime();
                    });
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, Solvers::Mode::SPP);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        Helpers::endConfigBody();
    }

    bool Task::initSolver(void *solver) {
        ::SPP &spp = *static_cast<::SPP *>(solver);
        if (!core->brdcPathBuf.empty()) {
            try {
                spp.loadBRDC(core->brdcPathBuf);
                core->hasNav = true;
                LOG_INFO << "SPP 读取广播星历: " << core->brdcPathBuf;
            } catch (const std::exception &e) {
                LOG_ERROR << "SPP 读取广播星历失败 " << core->brdcPathBuf << ": " << e.what();
            }
        }
        const auto &prod = AppConfig::Products::instance();
        ProductLoad::loadSPPGrids(spp, prod.vmf, Download::Store::activeFile(Download::Catalog::gpt3()));
        ProductLoad::warnTropoFallback("SPP", core->opts.spp.tropoMap, spp.tropoGridReady());
        return true;
    }


    void Task::beforeSolveEpoch(void *solver, Observer::Item *item) {
        ::SPP &spp = *static_cast<::SPP *>(solver);
        const auto &[obs, eph] = *item;
        spp.ephTable.mergeEmbedded(eph, obs.epoch);
        if (!core->isRINEX && eph) core->hasNav = true;
        if (core->isRINEX && !core->hasNav) core->noEphSolve = true;
    }

    ::SPP *Task::makeSolver() {
        return new ::SPP;
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::SPPSession>(
            std::static_pointer_cast<Task>(shared_from_this()), core->rtEndpoints[0]);
    }
}
