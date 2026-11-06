#include "DatePicker.h"

#include "GuiColors.h"

#include "TimeConvert.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>

namespace UI::DatePicker {
    namespace {
        constexpr ImVec4 kAccent(0.26f, 0.59f, 0.98f, 1.00f);
        constexpr ImVec4 kAccentHover(0.33f, 0.66f, 1.00f, 1.00f);
        constexpr ImVec4 kAccentActive(0.20f, 0.50f, 0.90f, 1.00f);
        constexpr const char *kWeekCn[7] = {"周日", "周一", "周二", "周三", "周四", "周五", "周六"};
        constexpr float kGap = 3.0f;
        constexpr float kCellH = 40.0f;

        /// 弹窗里正在浏览的年月：翻页只动它，点格子才写回调用的日期。
        int s_viewYear = 0;
        int s_viewMonth = 0;

        /// 0＝周日 … 6＝周六。1970-01-01 是周四，故 +4 对齐。
        int weekdayOf(const int y, const int m, const int d) {
            const long days = Time::civilToYMD({y, m, d});
            return static_cast<int>(((days + 4) % 7 + 7) % 7);
        }

        /// 单格宽度：中文星期字头比数字宽，按两者取大，格子才不会被字撑破。
        float cellWidth() {
            const ImGuiStyle &st = ImGui::GetStyle();
            const float head = ImGui::CalcTextSize(kWeekCn[3]).x + 6.0f;
            const float num = ImGui::CalcTextSize("30").x + st.FramePadding.x * 2.0f + 6.0f;
            return std::ceil(std::max(head, num));
        }

        /// 标题行：两只左箭头 + 两只右箭头 + 中间年月。
        float titleRowWidth() {
            char buf[32];
            std::snprintf(buf, sizeof buf, "0000 年 00 月");
            return ImGui::CalcTextSize(buf).x + ImGui::GetFrameHeight() * 4.0f + kGap * 4.0f;
        }

        /// 弹窗正文。popup 尺寸由内容反推，所以宽度只能算出来、不能读回来。
        bool calendarBody(int &year, int &month, int &day, const int *todayYmd) {
            bool changed = false;
            const float fh = ImGui::GetFrameHeight();
            const float x0 = ImGui::GetCursorPosX();
            const float cellW = cellWidth();
            const float gridW = cellW * 7.0f + kGap * 6.0f;
            const float contentW = std::max(gridW, titleRowWidth());
            const float gridX = x0 + (contentW - gridW) * 0.5f;

            char title[32];
            std::snprintf(title, sizeof title, "%04d 年 %d 月", s_viewYear, s_viewMonth);
            const float titleW = ImGui::CalcTextSize(title).x;

            ImGui::SetCursorPosX(x0);
            if (ImGui::ArrowButton("##prevYear", ImGuiDir_Left)) --s_viewYear;
            ImGui::SameLine(0.0f, kGap);
            if (ImGui::ArrowButton("##prevMonth", ImGuiDir_Left)) {
                if (--s_viewMonth < 1) {
                    s_viewMonth = 12;
                    --s_viewYear;
                }
            }
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::SetCursorPosX(x0 + std::max((contentW - titleW) * 0.5f, fh * 2.0f + kGap * 2.0f));
            ImGui::TextUnformatted(title);
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::SetCursorPosX(x0 + contentW - fh * 2.0f - kGap);
            if (ImGui::ArrowButton("##nextMonth", ImGuiDir_Right)) {
                if (++s_viewMonth > 12) {
                    s_viewMonth = 1;
                    ++s_viewYear;
                }
            }
            ImGui::SameLine(0.0f, kGap);
            if (ImGui::ArrowButton("##nextYear", ImGuiDir_Right)) ++s_viewYear;

            s_viewYear = std::clamp(s_viewYear, 1980, 2099);
            s_viewMonth = std::clamp(s_viewMonth, 1, 12);

            int tY = 0, tM = 0, tD = 0;
            if (todayYmd) {
                tY = todayYmd[0];
                tM = todayYmd[1];
                tD = todayYmd[2];
            } else {
                today(tY, tM, tD);
            }

            const ImVec4 dim = ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
            const ImVec4 fg = ImGui::GetStyleColorVec4(ImGuiCol_Text);
            for (int i = 0; i < 7; ++i) {
                const float tw = ImGui::CalcTextSize(kWeekCn[i]).x;
                if (i) ImGui::SameLine(0.0f, kGap);
                ImGui::SetCursorPosX(gridX + static_cast<float>(i) * (cellW + kGap) + (cellW - tw) * 0.5f);
                ImGui::TextColored(i == 0 || i == 6 ? dim : fg, "%s", kWeekCn[i]);
            }

            const int first = weekdayOf(s_viewYear, s_viewMonth, 1);
            const int nDays = Time::daysInMonth(s_viewYear, s_viewMonth);
            const ImVec4 frameBg = ImGui::GetStyleColorVec4(ImGuiCol_FrameBg);
            const ImVec4 frameHover = ImGui::GetStyleColorVec4(ImGuiCol_FrameBgHovered);
            const ImVec4 frameActive = ImGui::GetStyleColorVec4(ImGuiCol_FrameBgActive);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
            for (int slot = 0; slot < 42; ++slot) {
                if (slot % 7 == 0) ImGui::SetCursorPosX(gridX);
                else ImGui::SameLine(0.0f, kGap);
                const int dd = slot - first + 1;
                if (dd < 1 || dd > nDays) {
                    ImGui::Dummy(ImVec2(cellW, kCellH));
                    continue;
                }
                const bool sel = year == s_viewYear && month == s_viewMonth && dd == day;
                const bool isToday = s_viewYear == tY && s_viewMonth == tM && dd == tD;
                ImGui::PushStyleColor(ImGuiCol_Button, sel ? kAccent : frameBg);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, sel ? kAccentHover : frameHover);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, sel ? kAccentActive : frameActive);
                if (sel || isToday) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                char num[8];
                std::snprintf(num, sizeof num, "%d", dd);
                if (ImGui::Button(num, ImVec2(cellW, kCellH))) {
                    year = s_viewYear;
                    month = s_viewMonth;
                    day = dd;
                    changed = true;
                }
                if (isToday && !sel)
                    ImGui::GetWindowDrawList()->AddRect(
                        ImGui::GetItemRectMin(), ImGui::GetItemRectMax(),
                        ImGui::GetColorU32(kColPass), 4.0f, 0, 1.5f);
                if (sel || isToday) ImGui::PopStyleColor();
                ImGui::PopStyleColor(3);
            }
            ImGui::PopStyleVar();

            ImGui::Separator();
            ImGui::TextUnformatted("跳到");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(78.0f);
            if (ImGui::InputInt("##goY", &year, 0, 0)) {
                year = std::clamp(year, 1980, 2099);
                changed = true;
            }
            ImGui::SameLine(0.0f, kGap);
            ImGui::SetNextItemWidth(54.0f);
            if (ImGui::InputInt("##goM", &month, 0, 0)) {
                month = std::clamp(month, 1, 12);
                changed = true;
            }
            ImGui::SameLine(0.0f, kGap);
            ImGui::SetNextItemWidth(54.0f);
            if (ImGui::InputInt("##goD", &day, 0, 0)) {
                day = std::clamp(day, 1, Time::daysInMonth(year, month));
                changed = true;
            }
            if (changed) {
                day = std::clamp(day, 1, Time::daysInMonth(year, month));
                s_viewYear = year;
                s_viewMonth = month;
            }

            if (ImGui::Button("今天")) {
                year = tY;
                month = tM;
                day = tD;
                s_viewYear = year;
                s_viewMonth = month;
                changed = true;
            }
            ImGui::SameLine(0.0f, kGap);
            if (ImGui::Button("关闭")) ImGui::CloseCurrentPopup();
            return changed;
        }
    }

    void today(int &year, int &month, int &day) {
        const std::time_t t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        year = tm.tm_year + 1900;
        month = tm.tm_mon + 1;
        day = tm.tm_mday;
    }

    bool button(int &year, int &month, int &day, const int *todayYmd) {
        char label[48];
        std::snprintf(label, sizeof label, "%04d-%02d-%02d  %s", year, month, day,
                      kWeekCn[weekdayOf(year, month, day)]);
        if (ImGui::Button(label, ImVec2(ImGui::CalcTextSize(label).x + 22.0f, 0.0f))) {
            s_viewYear = year;
            s_viewMonth = month;
            ImGui::OpenPopup("##calendar");
        }
        if (!ImGui::BeginPopup("##calendar")) return false;
        const bool changed = calendarBody(year, month, day, todayYmd);
        ImGui::EndPopup();
        return changed;
    }
}
