#include "DragBar.h"

#include <algorithm>
#include <map>
#include <string>

#include "imgui.h"

namespace {
    /// 会话内的尺寸记忆：key → 高度。落地在 .cpp 内部，头文件里不放可变 static。
    std::map<std::string, float> &heightStore() {
        static std::map<std::string, float> s;
        return s;
    }
}

namespace UI {
    float &dragHeight(const char *key, const float defH) {
        std::map<std::string, float> &store = heightStore();
        const auto it = store.find(key);
        if (it == store.end()) return store.emplace(key, defH).first->second;
        return it->second;
    }

    void heightGrip(const char *id, float &height, const float minH, const float maxH) {
        ImGui::InvisibleButton(id, ImVec2(ImGui::GetContentRegionAvail().x, 7.0f));
        if (ImGui::IsItemActive())
            height = std::clamp(height + ImGui::GetIO().MouseDelta.y, minH, std::max(minH, maxH));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive())
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("上下拖动调整高度");
        const ImVec2 a = ImGui::GetItemRectMin();
        const ImVec2 b = ImGui::GetItemRectMax();
        const float cy = (a.y + b.y) * 0.5f;
        const float cx = (a.x + b.x) * 0.5f;
        ImGui::GetWindowDrawList()->AddLine(ImVec2(cx - 26.0f, cy), ImVec2(cx + 26.0f, cy),
                                            ImGui::GetColorU32(ImGuiCol_Separator), 2.0f);
    }
}
