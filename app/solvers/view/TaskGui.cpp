#include <windows.h>

#include "solvers/common/Task.h"

#include <functional>
#include <string>
#include <vector>

#include "Log.h"
#include "Observer.h"
#include "StringUtils.h"
#include "core/PathMemory.h"
#include "io/Products.h"
#include "solvers/common/Config.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "ui/GuiColors.h"

namespace Solvers {
    void Task::renderRealtimeConfig(const std::vector<std::string> &labels,
                                    const std::function<void()> &onStart,
                                    const std::function<void()> &extraContent) {
        const std::size_t n = rtEndpoints.size();
        ImGui::Text("连接到 NovAtel 实时流 (socket)：");
        ImGui::PushItemWidth(200);
        for (std::size_t i = 0; i < n && i < labels.size(); ++i) {
            if (!labels[i].empty()) ImGui::Text("%s：", labels[i].c_str());
            Helpers::inputTextStd(("IP 地址##rt_ep_" + std::to_string(i)).c_str(), &rtEndpoints[i].ip);
            Helpers::inputTextStd(("端口##rt_ep_" + std::to_string(i)).c_str(), &rtEndpoints[i].port);
        }
        ImGui::PopItemWidth();
        ImGui::Separator();

        if (extraContent) {
            ImGui::Separator();
            extraContent();
        }
        ImGui::Separator();
        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            if (onStart) onStart();
        }
        ImGui::SameLine();
        renderCancelButton();
    }

    void Task::renderConfigTabs(const std::string &title,
                                const std::function<void()> &fileTab,
                                const std::function<void()> &onStart,
                                const std::function<void()> &realtimeTab) {
        const std::string childId = title + std::to_string(cfgIndex);
        ImGui::BeginChild(childId.c_str(), ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        if (ImGui::BeginTabBar(("##cfg_tabs_" + std::string(modeKey(mode))).c_str())) {
            if (ImGui::BeginTabItem("文件")) {
                ImGui::Spacing();
                if (fileTab) fileTab();
                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (onStart) onStart();
                }
                ImGui::SameLine();
                renderCancelButton();
                ImGui::EndTabItem();
            }
            if (realtimeTab) {
                if (ImGui::BeginTabItem("实时")) {
                    realtimeTab();
                    ImGui::EndTabItem();
                }
            }
            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(opts, mode);
                ImGui::EndTabItem();
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
        if (hasProducts) AppConfig::Products::of(modeKey(mode)).save();
        if (onPersist) onPersist();
        Helpers::saveSolverParams(mode, opts);
        if (onLog) onLog();
        else LOG_INFO << "[" << modeLabel(mode) << "] 开始解算: 观测=" << path
                     << ", 格式=" << (isRINEX ? "RINEX" : "NovAtel");
        startFile();
    }

    void Task::renderCancelButton() {
        if (ImGui::Button("取消", ImVec2(80, 40))) {
            LOG_WARN << "[任务] 用户取消";
            cancel();
            state = State::Done;
            setError("用户取消");
        }
    }

    bool Task::beginFileSolve(const std::string &obsPath) {
        const auto fmt = obsPath.empty()
                             ? Observer::Format::Unknown
                             : Observer::classifyObsPath(obsPath);
        if (obsPath.empty()) {
            ImGui::TextColored(UI::kColFail, "请先选择观测文件");
            return false;
        }
        if (fmt == Observer::Format::Unknown) {
            ImGui::TextColored(UI::kColFail, "无法识别观测文件格式");
            return false;
        }
        if (worker.joinable()) worker.join();
        filePath = obsPath;
        fileName = Str::baseName(obsPath);
        view.obsPathBuf = obsPath;
        isRINEX = Observer::isRINEX(fmt);
        isRealtime = false;
        return true;
    }
}
