#include "GuiTimeConverter.h"
#include "imgui.h"
#include "ui/DatePicker.h"
#include "ui/GuiColors.h"
#include "core/TimeStruct.h"
#include "util/Clock.h"

#include <cstdio>
#include <cstring>
#include <exception>
#include <iterator>
#include <string>
#include "i18n/I18n.h"

namespace Tools::TimeConverter {
    struct SysItem {
        const char *label{};
        TimeSystem type;
        ImVec4 color;
    };

    constexpr SysItem kSystems[] = {
        {"UTC", TimeSystem::UTC, {1.00f, 1.00f, 1.00f, 1.00f}},
        {"GPS", TimeSystem::GPS, {0.40f, 1.00f, 0.40f, 1.00f}},
        {"BDT", TimeSystem::BDT, {1.00f, 0.45f, 0.45f, 1.00f}},
        {"TAI", TimeSystem::TAI, {0.55f, 0.75f, 1.00f, 1.00f}},
    };
    constexpr int kSysCount = std::size(kSystems);

    static int s_Civil[kSysCount][6] = {};
    static bool s_Syncing = false;
    static bool s_Initialized = false;
    static std::string s_Error;


    /// 把一个时间系统的字段当基准，推算出其余各系统；失败原因留在面板上。
    static void SyncFrom(const int source) {
        if (s_Syncing) return;
        s_Syncing = true;

        try {
            const CivilTime src(s_Civil[source][0], s_Civil[source][1], s_Civil[source][2],
                                s_Civil[source][3], s_Civil[source][4], s_Civil[source][5],
                                kSystems[source].type);
            const CommonTime base = src.toCommonTime();

            for (int i = 0; i < kSysCount; i++) {
                if (i == source) continue;
                CommonTime target = base;
                target.convertSystem(kSystems[i].type);
                const CivilTime civ = target.toCivilTime();
                s_Civil[i][0] = civ.year;
                s_Civil[i][1] = civ.month;
                s_Civil[i][2] = civ.day;
                s_Civil[i][3] = civ.hour;
                s_Civil[i][4] = civ.minute;
                s_Civil[i][5] = static_cast<int>(civ.second);
            }
            s_Error.clear();
        } catch (const std::exception &e) {
            s_Error = e.what();
        } catch (...) {
            s_Error = I18n::tr("tool.time.err_convert");
        }

        s_Syncing = false;
    }

    static void SetToNow() {
        const std::tm utc = Clock::UTC(Clock::wallNow());
        s_Civil[0][0] = utc.tm_year + 1900;
        s_Civil[0][1] = utc.tm_mon + 1;
        s_Civil[0][2] = utc.tm_mday;
        s_Civil[0][3] = utc.tm_hour;
        s_Civil[0][4] = utc.tm_min;
        s_Civil[0][5] = utc.tm_sec;
        SyncFrom(0);
    }

    /// UTC 的今天，供日历标「今天」用——本工具全按 UTC 读，本地时区在晚间会差一天。
    static void UtcToday(int out[3]) {
        const std::tm g = Clock::UTC(Clock::wallNow());
        out[0] = g.tm_year + 1900;
        out[1] = g.tm_mon + 1;
        out[2] = g.tm_mday;
    }


    void Render(bool *p_open) {
        if (!s_Initialized) {
            s_Initialized = true;
            SetToNow();
        }

        ImGui::SetNextWindowSize(ImVec2(620, 330), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin(I18n::trc("tool.time.title"), p_open)) {
            ImGui::End();
            return;
        }


        if (ImGui::Button(I18n::trc("tool.time.now")))
            SetToNow();
        ImGui::SameLine(0, 16);
        {
            // 日历只改日期，时分秒沿用 UTC 行现值
            int utcToday[3];
            UtcToday(utcToday);
            int y = s_Civil[0][0], m = s_Civil[0][1], d = s_Civil[0][2];
            ImGui::PushID("dateJump");
            if (UI::DatePicker::button(y, m, d, utcToday)) {
                s_Civil[0][0] = y;
                s_Civil[0][1] = m;
                s_Civil[0][2] = d;
                SyncFrom(0);
            }
            ImGui::PopID();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();


        if (ImGui::BeginTable("##tsys", 7,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn(I18n::trc("common.system"), ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn(I18n::trc("tool.time.col.year"));
            ImGui::TableSetupColumn(I18n::trc("tool.time.col.month"));
            ImGui::TableSetupColumn(I18n::trc("tool.time.col.day"));
            ImGui::TableSetupColumn(I18n::trc("tool.time.col.hour"));
            ImGui::TableSetupColumn(I18n::trc("tool.time.col.min"));
            ImGui::TableSetupColumn(I18n::trc("tool.time.col.sec"));
            ImGui::TableHeadersRow();

            static const char *FID[] = {"##yr", "##mo", "##dy", "##hr", "##mn", "##sc"};

            for (int i = 0; i < kSysCount; i++) {
                ImGui::PushID(i);
                ImGui::TableNextRow();


                ImGui::TableSetColumnIndex(0);
                ImGui::PushStyleColor(ImGuiCol_Text, kSystems[i].color);
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(kSystems[i].label);
                ImGui::PopStyleColor();


                int prev[6];
                std::memcpy(prev, s_Civil[i], sizeof(prev));


                for (int j = 0; j < 6; j++) {
                    ImGui::TableSetColumnIndex(j + 1);
                    ImGui::PushItemWidth(-FLT_MIN);
                    ImGui::InputInt(FID[j], &s_Civil[i][j], 1, 10);
                    ImGui::PopItemWidth();
                }


                if (std::memcmp(prev, s_Civil[i], sizeof(prev)) != 0)
                    SyncFrom(i);

                ImGui::PopID();
            }

            ImGui::EndTable();
        }


        ImGui::Spacing();
        ImGui::Separator();

        if (!s_Error.empty()) {
            ImGui::TextColored(UI::kColFail, "%s", s_Error.c_str());
        } else {
            try {
                const CivilTime utcCiv(s_Civil[0][0], s_Civil[0][1], s_Civil[0][2],
                                       s_Civil[0][3], s_Civil[0][4], s_Civil[0][5],
                                       TimeSystem::UTC);
                const CommonTime utc = utcCiv.toCommonTime();
                CommonTime gps = utc;
                gps.convertSystem(TimeSystem::GPS);

                const double leap = gps.leapSeconds();
                const MJD mjd = gps.toMJD();
                char leapText[32], mjdText[32];
                std::snprintf(leapText, sizeof(leapText), "%.0f", leap);
                std::snprintf(mjdText, sizeof(mjdText), "%.6f", static_cast<double>(mjd.mjd));
                ImGui::TextUnformatted(I18n::tr("tool.time.leap_mjd", leapText, mjdText).c_str());
            } catch (const std::exception &e) {
                ImGui::TextColored(UI::kColFail, "%s", e.what());
            } catch (...) {
                ImGui::TextColored(UI::kColFail, "%s", I18n::trc("tool.time.err_mjd"));
            }
        }

        ImGui::End();
    }
}
