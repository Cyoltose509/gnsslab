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

namespace Solvers::PPPKinematic {
    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        view.obsPathBuf = pm.get("ppp_kin_obs");
        Helpers::loadSolverParams(Mode::PPPKinematic, opts);
        RefTruth::restore(this);
        AppConfig::Products::of(modeKey(Mode::PPPKinematic)).load();
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        auto &prod = AppConfig::Products::of(modeKey(Mode::PPPKinematic));
        const auto onStart = [self] {
            self->renderFileStart("ppp_kin_obs", true,
                [self] { RefTruth::persist(self); }, [self] {
                    LOG_INFO << "[PPP (动态)] 开始解算: 观测=" << self->view.obsPathBuf
                            << ", 格式=" << Observer::formatName(self->obsFormat);
                });
        };
        const auto fileTab = [self, &prod] {
            Helpers::fileRow("观测文件", &self->view.obsPathBuf, "ppp_kin_obs",
                             FileDialog::fObs, 3, {}, "ppp_kin_obs", &prod.brdc);
            Helpers::renderProductSection(prod, "ppp_kin_obs", true);
            ImGui::Separator();
            RefTruth::renderRefTruthSection(self);
        };
        self->renderConfigTabs("PPP (动态) 处理配置###ppp_kin_cfg_", fileTab, onStart);
    }
}
