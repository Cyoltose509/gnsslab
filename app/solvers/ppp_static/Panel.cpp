#include "Task.h"

#include <string>

#include "Log.h"
#include "core/PathMemory.h"
#include "io/Products.h"
#include "imgui.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "solvers/view/GuiRefTruth.h"
#include "ui/FileDialog.h"

namespace Solvers::PPPStatic {
    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        view.obsPathBuf = pm.get("ppp_static_obs");
        Helpers::loadSolverParams(Mode::PPPStatic, opts);
        AppConfig::Products::of(modeKey(Mode::PPPStatic)).load();
        RefTruth::restore(this, "ppp_static_");
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        auto &prod = AppConfig::Products::of(modeKey(Mode::PPPStatic));
        const auto onStart = [self] {
            self->renderFileStart("ppp_static_obs", true,
                [self] { Solvers::RefTruth::persist(self, "ppp_static_"); },
                [self] {
                    LOG_INFO << "[PPP (静态)] 开始解算: 观测=" << self->view.obsPathBuf
                            << ", 格式=" << (self->isRINEX ? "RINEX" : "NovAtel");
                });
        };
        const auto fileTab = [self, &prod] {
            Helpers::fileRow("观测文件", &self->view.obsPathBuf, "ppp_static_obs",
                             FileDialog::fObs, 3, {}, "ppp_static_obs", &prod.brdc);
            Helpers::renderProductSection(prod, "ppp_static_obs");
            ImGui::Separator();
            RefTruth::renderRefTruthField(self);
        };
        self->renderConfigTabs("PPP (静态) 处理配置###ppp_static_cfg_", fileTab, onStart);
    }
}
