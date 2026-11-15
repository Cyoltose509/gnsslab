#include <windows.h>

#include "solvers/common/Task.h"
#include "FileUtils.h"

#include <functional>
#include <string>
#include <vector>

#include "Log.h"
#include "Observer.h"
#include "StringUtils.h"
#include "core/PathMemory.h"
#include "io/ProductPaths.h"
#include "solvers/common/Config.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "ui/GuiColors.h"
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

namespace Solvers {
    void Task::renderRealtimeConfig(const std::vector<std::string> &labels,
                                    const std::function<void()> &onStart,
                                    const std::function<void()> &extraContent) {
        const std::size_t n = rtEndpoints.size();
        ImGui::Text(I18n::trc("task.rt.connect"));
        ImGui::PushItemWidth(200);
        for (std::size_t i = 0; i < n && i < labels.size(); ++i) {
            if (!labels[i].empty()) ImGui::Text("%s：", labels[i].c_str());
            Helpers::inputTextStd(UI::idLabel("task.rt.ip", "##rt_ep_", std::to_string(i)), &rtEndpoints[i].ip);
            Helpers::inputTextStd(UI::idLabel("task.rt.port", "##rt_ep_", std::to_string(i)), &rtEndpoints[i].port);
        }
        ImGui::PopItemWidth();
        ImGui::Separator();

        if (extraContent) {
            ImGui::Separator();
            extraContent();
        }
        ImGui::Separator();
        if (ImGui::Button(I18n::trc("task.start"), ImVec2(200, 40))) {
            if (onStart) onStart();
        }
        ImGui::SameLine();
        renderCancelButton();
    }

    void Task::renderConfigTabs(const std::string &title,
                                const std::function<void()> &fileTab,
                                const std::function<void()> &onStart,
                                const std::function<void()> &realtimeTab,
                                const bool showParamTab) {
        const std::string childId = title + std::to_string(cfgIndex);
        ImGui::BeginChild(childId.c_str(), ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        if (ImGui::BeginTabBar(("##cfg_tabs_" + std::string(modeKey(mode))).c_str())) {
            if (ImGui::BeginTabItem(I18n::trc("common.file"))) {
                ImGui::Spacing();
                if (fileTab) fileTab();
                if (ImGui::Button(I18n::trc("task.start"), ImVec2(200, 40))) {
                    if (onStart) onStart();
                }
                ImGui::SameLine();
                renderCancelButton();
                ImGui::EndTabItem();
            }
            if (realtimeTab) {
                if (ImGui::BeginTabItem(I18n::trc("RealTime"))) {
                    realtimeTab();
                    ImGui::EndTabItem();
                }
            }
            if (showParamTab) {
                if (ImGui::BeginTabItem(I18n::trc("common.params"))) {
                    ImGui::Spacing();
                    Helpers::renderAdvancedParams(opts, mode);
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        ImGui::EndChild();
    }

    void Task::renderFileStart(const char *obsIniKey, const bool hasProducts,
                               const std::function<void()> &onPersist,
                               const std::function<void()> &onLog) {
        if (!beginFileSolve(view.obsPathBuf)) return;
        const std::string &path = view.obsPathBuf;
        const auto &cfg = AppConfig::PathMemory::instance();
        cfg.set(obsIniKey, path);
        if (hasProducts) AppConfig::ProductPaths::of(modeKey(mode)).save();
        if (onPersist) onPersist();
        Helpers::saveSolverParams(mode, opts);
        if (onLog) onLog();
        else LOG_INFO << "[" << modeLabel(mode) << I18n::tr("task.log.start", path,
                                                                   Observer::formatName(obsFormat));
        startFile();
    }

    void Task::renderCancelButton() {
        if (ImGui::Button(I18n::trc("common.cancel"), ImVec2(80, 40))) {
            LOG_WARN << I18n::trc("task.log.cancelled");
            cancel();
            state = State::Done;
            setError(I18n::trc("task.cancelled"));
        }
    }

    bool Task::beginFileSolve(const std::string &obsPath) {
        if (obsPath.empty()) {
            ImGui::TextColored(UI::kColFail, I18n::trc("task.err.no_obs"));
            return false;
        }
        const auto fmt = Observer::classifyObsPath(obsPath);
        if (fmt == Observer::Format::Unknown) {
            ImGui::TextColored(UI::kColFail, I18n::trc("io.err.bad_format"));
            return false;
        }
        if (worker.joinable()) worker.join();
        applyObsPath(obsPath);
        isRealtime = false;
        return true;
    }
}
