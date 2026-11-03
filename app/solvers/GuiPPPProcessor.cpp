#include "GuiPPPProcessor.h"
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
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "Log.h"

#include <filesystem>
#include <thread>
#include <string>
#include <vector>
#include <future>
#include "view/GuiSolverPanel.h"

namespace Solvers::PPP {
    using Solvers::Epoch;
    using Solvers::Core;
    using Solvers::Mode;

    template<typename PPPSolver>
    static void loadProductsInto(Task &task, PPPSolver &ppp, std::vector<std::future<bool> > &jobs) {
        ProductLoad::core(task, ppp, jobs, "PPP");
        const auto &prod = AppConfig::Products::instance();
        ProductLoad::loadPPPExtras(ppp, jobs, prod.tro, prod.ionex, prod.met, prod.vmf,
                                   Download::Store::activeFile(Download::Catalog::gpt3()));
    }


    bool Task::initSolver(void *solver) {
        std::vector<std::future<bool> > jobs;
        const ::SPP *ppp = nullptr;
        const auto loadInto = [&](auto *p) {
            loadProductsInto(*this, *p, jobs);
            ppp = p;
        };
        if (kinematic) loadInto(static_cast<PPPKinematic *>(solver));
        else loadInto(static_cast<PPPStatic *>(solver));
        for (auto &j: jobs) j.get();
        const bool srcOk = ProductLoad::requireSatellite("PPP");
        ProductLoad::warnTropoFallback("PPP", core->opts.spp.tropoMap, ppp->tropoGridReady());
        return srcOk ? true : ProductLoad::fail(core.get());
    }

    ::SPP *Task::makeSolver() {
        return kinematic ? static_cast<::SPP *>(new PPPKinematic) : static_cast<::SPP *>(new PPPStatic);
    }

    void Task::renderConfigPanel() {
        Helpers::beginConfigBody("PPP 处理配置###ppp_cfg_", *this);

        if (ImGui::BeginTabBar("##ppp_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                ImGui::Spacing();

                Helpers::fileRow("观测文件", &this->obsPathBuf, "ppp_obs", FileDialog::fObs, 3,
                                 {}, "ppp_obs", &AppConfig::Products::instance().brdc);
                Helpers::renderProductSection("ppp_obs");
                ImGui::Separator();
                Helpers::renderSimpleConfig(&this->core->cutoffDeg, &this->core->enabledSystems, &this->core->sampleInterval);
                ImGui::Separator();
                ImGui::Checkbox("动态解算 (Kinematic)", &this->kinematic);
                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (Helpers::beginFileSolve(this, this->obsPathBuf)) {
                        const std::string &path = this->obsPathBuf;

                        const auto &cfg = AppConfig::PathMemory::instance();
                        cfg.set("ppp_obs", path);
                        AppConfig::Products::instance().save();
                        cfg.set("ppp_kinematic", this->kinematic ? "1" : "0");
                        Helpers::saveCommonCfg("ppp", Solvers::Mode::PPP, core->opts,
                                               this->core->cutoffDeg, this->core->sampleInterval, this->core->enabledSystems);
                        LOG_INFO << "[PPP] 开始解算: 观测=" << path
                                << ", 格式=" << (this->core->isRINEX ? "RINEX" : "NovAtel")
                                << ", 动态=" << (this->kinematic ? "是" : "否");
                        this->startFile();
                    }
                }
                ImGui::SameLine();
                Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, Solvers::Mode::PPP);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Helpers::endConfigBody();
    }
}
