#include "Task.h"

#include <string>

#include "Log.h"
#include "core/PathMemory.h"
#include "io/ProductPaths.h"
#include "imgui.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "solvers/view/GuiRefTruth.h"
#include "ui/FileDialog.h"
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

namespace Solvers::PPPKinematic {
    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        view.obsPathBuf = pm.get("ppp_kin_obs");
        Helpers::loadSolverParams(Mode::PPPKinematic, opts);
        RefTruth::restore(this);
        AppConfig::ProductPaths::of(modeKey(Mode::PPPKinematic)).load();
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        auto &prod = AppConfig::ProductPaths::of(modeKey(Mode::PPPKinematic));
        const auto onStart = [self] {
            self->renderFileStart("ppp_kin_obs", true,
                [self] { RefTruth::persist(self); }, [self] {
                    LOG_INFO << "[PPPKinematic] " << I18n::tr("log.start2", self->view.obsPathBuf,
                                                                      Observer::formatName(self->obsFormat));
                });
        };
        const auto fileTab = [self, &prod] {
            Helpers::fileRow(I18n::trc("ObservationFile"), &self->view.obsPathBuf, "ppp_kin_obs",
                             FileDialog::fObs(), {}, "ppp_kin_obs", &prod.nav);
            Helpers::renderProductSection(prod, "ppp_kin_obs", true);
            ImGui::Separator();
            RefTruth::renderRefTruthSection(self);
        };
        self->renderConfigTabs(UI::idLabel("ppp.cfg_tab_kinematic", "###ppp_kin_cfg_"), fileTab, onStart);
    }
}
