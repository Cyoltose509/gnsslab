#include "ui/Markdown.h"

#include "StringUtils.h"
#include "ui/GuiColors.h"

#include "imgui.h"
#include "imgui_markdown.h"

#include <windows.h>
#include <shellapi.h>

#include <cstddef>
#include <string>
#include <string_view>

namespace {
    void openLink(const ImGui::MarkdownLinkCallbackData &data) {
        if (data.isImage) return;
        const std::string url(data.link, static_cast<std::size_t>(data.linkLength));
        const std::wstring wide = Str::UTF8ToWide(url);
        ShellExecuteW(nullptr, L"open", wide.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    }

    /// 标题只用现有字体，靠颜色和分隔线分级，不额外加载字体。
    void formatMarkdown(const ImGui::MarkdownFormatInfo &info, const bool start) {
        if (info.type == ImGui::MarkdownFormatType::HEADING) {
            ImVec4 color = UI::kColInfo;
            if (info.level == 2) color = UI::kColPass;
            else if (info.level >= 3) color = UI::kColWarn;
            if (start) ImGui::PushStyleColor(ImGuiCol_Text, color);
            else ImGui::PopStyleColor();
        }
        ImGui::defaultMarkdownFormatCallback(info, start);
    }

    /// 库不支持围栏与行内代码，双星号也会显示成噪声，先清成纯文本。
    std::string plainForDisplay(const char *text, const std::size_t len) {
        std::string out;
        out.reserve(len);
        for (std::size_t i = 0; i < len;) {
            std::size_t e = i;
            while (e < len && text[e] != '\n') ++e;
            const std::string_view line(text + i, e - i);
            const std::size_t first = line.find_first_not_of(" \t\r");
            if (first == std::string_view::npos || line.substr(first, 3) != "```") {
                for (std::size_t k = 0; k < line.size(); ++k) {
                    if (line[k] == '`') continue;
                    if (line[k] == '*' && k + 1 < line.size() && line[k + 1] == '*') { ++k; continue; }
                    out.push_back(line[k]);
                }
                out.push_back('\n');
            }
            i = e < len ? e + 1 : e;
        }
        return out;
    }
}

void UI::renderMarkdown(const char *text, const std::size_t len) {
    if (!text || len == 0) return;
    const std::string md = plainForDisplay(text, len);
    ImGui::MarkdownConfig cfg;
    cfg.linkCallback = [](ImGui::MarkdownLinkCallbackData data) { openLink(data); };
    cfg.tooltipCallback = ImGui::defaultMarkdownTooltipCallback;
    cfg.linkIcon = "";
    cfg.formatCallback = formatMarkdown;
    cfg.formatFlags = ImGuiMarkdownFormatFlags_GithubStyle;
    ImGui::Markdown(md.c_str(), md.size(), cfg);
}
