#include "solvers/ins/Task.h"

#include "Log.h"
#include "core/Const.h"
#include "core/PathMemory.h"
#include "imgui.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"
#include "ui/GuiColors.h"
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

namespace Solvers::INS {
    void Task::loadFromMemory() {
        view.obsPathBuf = AppConfig::PathMemory::instance().get("ins_imu");
        opts.ins.imuPath = view.obsPathBuf;
        Helpers::loadSolverParams(Mode::INS, opts);
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        const auto onStart = [self] {
            self->scaleText = self->scaleDet.text;
            self->renderFileStart("ins_imu", false,
                [self] { self->opts.ins.imuPath = self->view.obsPathBuf; },
                [self] { LOG_INFO << "[INS] " << I18n::tr("ins.log.start_file", self->view.obsPathBuf); });
        };
        const auto fileTab = [self] {
            Helpers::fileRow(I18n::trc("ins.imu_log_file"), &self->view.obsPathBuf, "ins_imu",
                             FileDialog::fIns());
            ImGui::Separator();
            Helpers::renderInsInitialState(self->opts);
            ImGui::Separator();
            Helpers::renderImuScale(self->opts.ins.manual, self->scaleDet, self->view.obsPathBuf,
                                    getGravity(self->opts.ins.latDeg * DEG_TO_RAD, self->opts.ins.hM),
                                    "ins_scale");
            ImGui::Separator();
        };
        self->renderConfigTabs(UI::idLabel("ins.cfg_tab", "###ins_cfg_"), fileTab, onStart, {}, false);
    }
}  // namespace Solvers::INS
