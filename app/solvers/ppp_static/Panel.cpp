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

namespace Solvers::PPPStatic {
    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        view.obsPathBuf = pm.get("ppp_static_obs");
        Helpers::loadSolverParams(Mode::PPPStatic, opts);
        AppConfig::ProductPaths::of(modeKey(Mode::PPPStatic)).load();
        RefTruth::restore(this);
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        auto &prod = AppConfig::ProductPaths::of(modeKey(Mode::PPPStatic));
        const auto onStart = [self] {
            self->renderFileStart("ppp_static_obs", true,
                [self] { RefTruth::persist(self); },
                [self] {
                    LOG_INFO << "[PPPStatic] " << I18n::tr("log.start2", self->view.obsPathBuf,
                                                                   Observer::formatName(self->obsFormat));
                });
        };
        const auto fileTab = [self, &prod] {
            Helpers::fileRow(I18n::trc("ObservationFile"), &self->view.obsPathBuf, "ppp_static_obs",
                             FileDialog::fObs(), {}, "ppp_static_obs", &prod.nav);
            Helpers::renderProductSection(prod, "ppp_static_obs", true);
            ImGui::Separator();
            RefTruth::renderRefTruthSection(self);
        };
        self->renderConfigTabs(UI::idLabel("ppp.cfg_tab_static", "###ppp_static_cfg_"), fileTab, onStart);
    }
}
