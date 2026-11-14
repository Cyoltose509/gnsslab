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
        RefTruth::restore(this);
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        auto &prod = AppConfig::Products::of(modeKey(Mode::PPPStatic));
        const auto onStart = [self] {
            self->renderFileStart("ppp_static_obs", true,
                [self] { RefTruth::persist(self); },
                [self] {
                    LOG_INFO << "[PPP (静态)] 开始解算: 观测=" << self->view.obsPathBuf
                            << ", 格式=" << Observer::formatName(self->obsFormat);
                });
        };
        const auto fileTab = [self, &prod] {
            Helpers::fileRow("观测文件", &self->view.obsPathBuf, "ppp_static_obs",
                             FileDialog::fObs, 3, {}, "ppp_static_obs", &prod.brdc);
            Helpers::renderProductSection(prod, "ppp_static_obs", true);
            ImGui::Separator();
            RefTruth::renderRefTruthSection(self);
        };
        self->renderConfigTabs("PPP (静态) 处理配置###ppp_static_cfg_", fileTab, onStart);
    }
}
