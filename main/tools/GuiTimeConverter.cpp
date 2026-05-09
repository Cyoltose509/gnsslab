#include "GuiTimeConverter.h"
#include "imgui.h"
#include "TimeConvert.h"
#include "TimeStruct.h"
#include "Const.h"

#include <ctime>
#include <cstring>

namespace GuiTimeConverter
{
    // ---------- Configuration ----------
    static constexpr int SYS_COUNT = 4;

    static const char*                  SYS_LABELS[SYS_COUNT] = { "UTC", "GPS", "BDT", "TAI" };
    static const TimeSystem::SystemType SYS_TYPES[SYS_COUNT]  = {
        TimeSystem::UTC, TimeSystem::GPS, TimeSystem::BDT,
        TimeSystem::TAI
    };

    // Label colors per system
    static const ImVec4 SYS_COLORS[SYS_COUNT] = {
        {1.0f, 1.0f, 1.0f, 1.0f},   // UTC  - white
        {0.4f, 1.0f, 0.4f, 1.0f},   // GPS  - green
        {1.0f, 0.45f, 0.45f, 1.0f}, // BDT  - red
        {0.55f, 0.75f, 1.0f, 1.0f}, // TAI  - blue
    };

    // ---------- State ----------
    static int  s_Civil[SYS_COUNT][6] = {}; // [sys][Y, M, D, H, m, S]
    static bool s_Syncing     = false;
    static bool s_Initialized = false;

    // ---------- Core Logic ----------
    static void SyncFrom(int source)
    {
        if (s_Syncing) return;
        s_Syncing = true;

        try
        {
            CivilTime src(
                s_Civil[source][0], s_Civil[source][1], s_Civil[source][2],
                s_Civil[source][3], s_Civil[source][4], (double)s_Civil[source][5],
                TimeSystem(SYS_TYPES[source])
            );
            CommonTime base = CivilTime2CommonTime(src);

            for (int i = 0; i < SYS_COUNT; i++)
            {
                if (i == source) continue;
                CommonTime target = base;
                convertTimeSystem(target, TimeSystem(SYS_TYPES[i]));
                CivilTime civ = CommonTime2CivilTime(target);
                s_Civil[i][0] = civ.year;
                s_Civil[i][1] = civ.month;
                s_Civil[i][2] = civ.day;
                s_Civil[i][3] = civ.hour;
                s_Civil[i][4] = civ.minute;
                s_Civil[i][5] = (int)civ.second;
            }
        }
        catch (...) {}

        s_Syncing = false;
    }

    static void SetToNow()
    {
        std::time_t t = std::time(nullptr);
        std::tm* utc = std::gmtime(&t);
        if (!utc) return;

        s_Civil[0][0] = utc->tm_year + 1900;
        s_Civil[0][1] = utc->tm_mon + 1;
        s_Civil[0][2] = utc->tm_mday;
        s_Civil[0][3] = utc->tm_hour;
        s_Civil[0][4] = utc->tm_min;
        s_Civil[0][5] = utc->tm_sec;
        SyncFrom(0);
    }

    // ---------- Render ----------
    void Render(bool* p_open)
    {
        if (!s_Initialized)
        {
            s_Initialized = true;
            SetToNow();
        }

        ImGui::SetNextWindowSize(ImVec2(620, 330), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("时间转换", p_open))
        {
            ImGui::End();
            return;
        }

        // --- Toolbar ---
        if (ImGui::Button("此刻"))
            SetToNow();
        ImGui::SameLine(0, 16);
        ImGui::TextDisabled("编辑任意字段，自动同步所有时间系统");

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // --- Time System Table ---
        if (ImGui::BeginTable("##tsys", 7,
            ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchSame))
        {
            ImGui::TableSetupColumn("系统", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("年");
            ImGui::TableSetupColumn("月");
            ImGui::TableSetupColumn("日");
            ImGui::TableSetupColumn("时");
            ImGui::TableSetupColumn("分");
            ImGui::TableSetupColumn("秒");
            ImGui::TableHeadersRow();

            static const char* FID[] = { "##yr", "##mo", "##dy", "##hr", "##mn", "##sc" };

            for (int i = 0; i < SYS_COUNT; i++)
            {
                ImGui::PushID(i);
                ImGui::TableNextRow();

                // System label with color
                ImGui::TableSetColumnIndex(0);
                ImGui::PushStyleColor(ImGuiCol_Text, SYS_COLORS[i]);
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(SYS_LABELS[i]);
                ImGui::PopStyleColor();

                // Snapshot before user interaction
                int prev[6];
                std::memcpy(prev, s_Civil[i], sizeof(prev));

                // 6 InputInt fields
                for (int j = 0; j < 6; j++)
                {
                    ImGui::TableSetColumnIndex(j + 1);
                    ImGui::PushItemWidth(-FLT_MIN);
                    ImGui::InputInt(FID[j], &s_Civil[i][j], 1, 10);
                    ImGui::PopItemWidth();
                }

                // Detect change and sync
                if (std::memcmp(prev, s_Civil[i], sizeof(prev)) != 0)
                    SyncFrom(i);

                ImGui::PopID();
            }

            ImGui::EndTable();
        }

        // --- Info bar ---
        ImGui::Spacing();
        ImGui::Separator();

        try
        {
            CivilTime utcCiv(
                s_Civil[0][0], s_Civil[0][1], s_Civil[0][2],
                s_Civil[0][3], s_Civil[0][4], (double)s_Civil[0][5],
                TimeSystem(TimeSystem::UTC)
            );
            CommonTime utc = CivilTime2CommonTime(utcCiv);
            CommonTime gps = utc;
            convertTimeSystem(gps, TimeSystem(TimeSystem::GPS));

            double leap = getLeapSeconds(gps);

            MJD mjd;
            CommonTime2MJD(gps, mjd);

            ImGui::Text("跳秒: %.0f    MJD (GPS): %.6f",
                        leap, (double)mjd.mjd);
        }
        catch (...) {}

        ImGui::End();
    }
}
