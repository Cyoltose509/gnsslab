#include "ui/ParamPage.h"

#include "i18n/I18n.h"

#include <algorithm>

namespace {
    constexpr const char *kWidthKey = "ui.param_field_width";

    float gControlWidth = 0.0f;
    bool gWidthLoaded = false;

    void loadWidth() {
        if (gWidthLoaded) return;
        gWidthLoaded = true;
        gControlWidth = static_cast<float>(
            AppConfig::PathMemory::instance().getNum(kWidthKey, 0.0, 0.0, 4096.0));
    }

    float widthLimit() {
        loadWidth();
        return gControlWidth > 0.0f ? gControlWidth : ImGui::GetFontSize() * 12.0f;
    }

    float fieldWidth() {
        const float avail = ImGui::GetContentRegionAvail().x;
        const float wanted = std::min(widthLimit(), avail);
        return std::max(wanted, std::min(ImGui::GetFontSize() * 4.0f, avail));
    }
}

namespace UI::ParamPage {
    float labelWidth = 0.0f;

    void beginField(const char *label, const bool autoWidth) {
        const float indent = labelWidth > 0.0f ? labelWidth : ImGui::GetFontSize() * 14.0f;
        const float baseX = ImGui::GetCursorPosX();
        ImGui::TextUnformatted(I18n::trc(label));
        ImGui::SameLine();
        ImGui::SetCursorPosX(baseX + indent);
        if (autoWidth) ImGui::SetNextItemWidth(fieldWidth());
    }

    void drawGroupTitle(const char *title) {
        if (!title || !*title) return;
        const char *text = I18n::trc(title);
        ImGui::SetWindowFontScale(1.16f);
        const ImVec2 pos = ImGui::GetCursorScreenPos();
        const ImU32 col = ImGui::GetColorU32(ImGuiCol_Text);
        ImGui::GetWindowDrawList()->AddText(ImVec2(pos.x + 0.8f, pos.y), col, text);
        ImGui::TextUnformatted(text);
        ImGui::SetWindowFontScale(1.0f);
    }
}
