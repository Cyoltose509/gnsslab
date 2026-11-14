#include "Task.h"
#include "FileUtils.h"

#include <cstdio>
#include <string>

#include "solvers/leo/Antenna.h"
#include "Log.h"
#include "StringUtils.h"
#include "core/PathMemory.h"
#include "io/Products.h"
#include "imgui.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "solvers/view/GuiRefTruth.h"
#include "ui/FileDialog.h"
#include "ui/Gui.h"
#include "ui/GuiColors.h"

namespace Solvers::LEO {
    namespace {
        void clearTextBox(Task::TextBox &box) {
            for (auto &t: box.text) t[0] = '\0';
        }

        void syncDefault(Task *self) {
            if (self->antennaObsPath == self->view.obsPathBuf) return;
            const std::string oldKey = Antenna::missionKey(self->antennaObsPath);
            const std::string newKey = Antenna::missionKey(self->view.obsPathBuf);
            self->antennaObsPath = self->view.obsPathBuf;
            if (!oldKey.empty() && oldKey != newKey) {
                self->antenna = {};
                clearTextBox(self->antennaArpText);
                clearTextBox(self->antennaPcoText);
            }
            self->antennaDef = Antenna::defaults(self->view.obsPathBuf);
        }

        void renderSegment(const char *tag, Task::TextBox &box, Antenna::Segment &seg,
                           const Antenna::Segment &def) {
            static const char *kAxis[3] = {"X", "Y", "Z"};
            bool anyText = false;
            bool edited = false;
            for (int i = 0; i < 3; i++) {
                if (i) ImGui::SameLine();
                ImGui::TextUnformatted(kAxis[i]);
                ImGui::SameLine(0, 4);
                ImGui::SetNextItemWidth(108);
                const std::string hint = Str::fmtG(def.xyz[i]);
                const std::string id = std::string("##") + tag + kAxis[i];
                if (ImGui::InputTextWithHint(id.c_str(), hint.c_str(), box.text[i], sizeof box.text[i],
                                             ImGuiInputTextFlags_CharsScientific))
                    edited = true;
                if (box.text[i][0]) anyText = true;
            }
            if (!anyText) {
                seg = {}; // 三个框都空了：整段退回缺省
                return;
            }
            seg.known = true;
            if (edited || seg.from.empty()) seg.from = "手工填写";
            for (int i = 0; i < 3; i++) {
                seg.xyz[i] = def.xyz[i]; // 空着的轴仍用缺省
                double v = 0.0;
                if (box.text[i][0] && Str::scanDoubles(box.text[i], &v, 1) == 1) seg.xyz[i] = v;
            }
        }

        void renderGroup(const char *tag, const char *label,
                         Task::TextBox &box, Antenna::Segment &user, const Antenna::Segment &def) {
            ImGui::TextUnformatted(label);
            ImGui::SameLine(0, 10);
            renderSegment(tag, box, user, def);
            ImGui::SameLine(0, 14);
            if (const auto [clicked, count, xyz] = Helpers::clipboardXYZButton(tag); clicked && count > 0)
                for (int i = 0; i < count; i++)
                    std::snprintf(box.text[i], sizeof box.text[i], "%s", Str::fmtG(xyz[i]).c_str());
        }

        void renderAntennaSection(Task *self) {
            syncDefault(self);

            ImGui::SeparatorText("接收机天线改正");

            renderGroup("arp", "天线改正  质心→ARP",
                        self->antennaArpText, self->antenna.comToArp, self->antennaDef.comToArp);
            renderGroup("pco", "PCO  ARP→相位中心",
                        self->antennaPcoText, self->antenna.pco, self->antennaDef.pco);
        }
    }

    void Task::loadFromMemory() {
        const auto &cfg = AppConfig::PathMemory::instance();
        view.obsPathBuf = cfg.get("leo_obs");
        RefTruth::restore(this);
        Helpers::loadSolverParams(Mode::LEO, opts);
        AppConfig::Products::of(modeKey(Mode::LEO)).load();
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        auto &prod = AppConfig::Products::of(modeKey(Mode::LEO));
        const auto onStart = [self] {
            self->renderFileStart("leo_obs", true,
                [self] { RefTruth::persist(self); },
                [self] {
                    const Antenna::Correction ant = Antenna::merge(self->antenna, self->antennaDef);
                    const std::string refInfo = self->truth.sp3Path.empty() ? std::string("无") : self->truth.sp3Path;
                    LOG_INFO << "[LEO] 开始定轨: 观测=" << self->view.obsPathBuf
                            << ", 格式=" << Observer::formatName(self->obsFormat)
                            << ", 参考轨道=" << refInfo
                            << ", 天线改正=" << ant.total().transpose() << " m ← "
                            << (ant.comToArp.from.empty() ? std::string("无") : ant.comToArp.from);
                });
        };
        const auto fileTab = [self, &prod] {
            Helpers::fileRow("观测文件", &self->view.obsPathBuf, "leo_obs", FileDialog::fObs, 3,
                             {}, "leo_obs", &prod.brdc, ProductType::LEO_OBS);
            Helpers::renderProductSection(prod, "leo_obs", false, /*crossDayHint=*/true);
            RefTruth::renderRefTruthSection(self);
            renderAntennaSection(self);
            ImGui::Separator();
        };
        self->renderConfigTabs("LEO 处理配置###leo_cfg_", fileTab, onStart);
    }
}
