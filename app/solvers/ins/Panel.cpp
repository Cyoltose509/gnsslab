#include "solvers/ins/Task.h"

#include "Log.h"
#include "core/PathMemory.h"
#include "imgui.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"
#include "ui/GuiColors.h"

namespace Solvers::INS {
    void Task::loadFromMemory() {
        view.obsPathBuf = AppConfig::PathMemory::instance().get("ins_imu");
        opts.ins.imuPath = view.obsPathBuf;
        Helpers::loadSolverParams(Mode::INS, opts);
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        const auto onStart = [self] {
            self->renderFileStart("ins_imu", false,
                [self] { self->opts.ins.imuPath = self->view.obsPathBuf; },
                [self] { LOG_INFO << "[INS] 开始解算: IMU=" << self->view.obsPathBuf; });
        };
        const auto fileTab = [self] {
            Helpers::fileRow("IMU 日志", &self->view.obsPathBuf, "ins_imu",
                             FileDialog::fIns, 2);
            ImGui::Separator();
            Helpers::renderInsInitialState(self->opts);
            ImGui::Separator();
        };
        self->renderConfigTabs("INS 处理配置###ins_cfg_", fileTab, onStart, {}, false);
    }
}  // namespace Solvers::INS
