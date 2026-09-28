#include "GuiCoordConverter.h"
#include "imgui.h"
#include "Const.h"
#include "CoordStruct.h"
#include "CoordConvert.h"

#include <cstring>
#include <cmath>

namespace GuiCoordConverter {
    // ========== Frame selection ==========
    static const char *FRAME_NAMES[] = {"WGS84", "GPS", "CGCS2000", "PZ90"};
    static constexpr int FRAME_COUNT = 4;
    static const FrameInfo *FRAMES[] = {
        &Frame::WGS84, &Frame::GPS, &Frame::CGCS2000, &Frame::PZ90
    };
    static int s_FrameIdx = 0;

    // ========== XYZ <-> BLH ==========
    static double s_X = -2700000.0;
    static double s_Y = 4700000.0;
    static double s_Z = 3300000.0;

    static double s_B_deg = 0.0;
    static double s_L_deg = 0.0;
    static double s_H = 0.0;

    // 0 = XYZ last edited, 1 = BLH last edited
    static int s_XyzBlhSrc = 0;

    // ========== ENU ==========
    static double s_RefB_deg = 30.0;
    static double s_RefL_deg = 114.0;
    static double s_RefH = 50.0;

    static double s_TgtX = -2700000.0;
    static double s_TgtY = 4700000.0;
    static double s_TgtZ = 3300000.0;

    static double s_E = 0.0;
    static double s_N = 0.0;
    static double s_U = 0.0;

    // 0 = XYZ last edited, 1 = ENU last edited
    static int s_EnuSrc = 0;

    static bool s_Initialized = false;

    // ========== Conversions ==========
    static const FrameInfo &CurFrame() { return *FRAMES[s_FrameIdx]; }

    static void SyncXYZtoBLH() {
        try {
            const BLH blh = XYZtoBLH({s_X, s_Y, s_Z}, CurFrame());
            s_B_deg = blh.B() * RAD_TO_DEG;
            s_L_deg = blh.L() * RAD_TO_DEG;
            s_H = blh.H();
        } catch (...) {
        }
    }

    static void SyncBLHtoXYZ() {
        try {
            XYZ xyz = BLHtoXYZ({s_B_deg * DEG_TO_RAD, s_L_deg * DEG_TO_RAD, s_H}, CurFrame());
            s_X = xyz.X();
            s_Y = xyz.Y();
            s_Z = xyz.Z();
        } catch (...) {
        }
    }

    static void SyncXYZtoENU() {
        try {
            const XYZ refXYZ = BLHtoXYZ({s_RefB_deg * DEG_TO_RAD, s_RefL_deg * DEG_TO_RAD, s_RefH}, CurFrame());
            const ENU enu = XYZtoENU({s_TgtX, s_TgtY, s_TgtZ}, refXYZ, CurFrame());
            s_E = enu.E();
            s_N = enu.N();
            s_U = enu.U();
        } catch (...) {
        }
    }

    static void SyncENUtoXYZ() {
        try {
            const XYZ refXYZ = BLHtoXYZ({s_RefB_deg * DEG_TO_RAD, s_RefL_deg * DEG_TO_RAD, s_RefH}, CurFrame());
            const XYZ xyz = ENUtoXYZ({s_E, s_N, s_U}, refXYZ, CurFrame());
            s_TgtX = xyz.X();
            s_TgtY = xyz.Y();
            s_TgtZ = xyz.Z();
        } catch (...) {
        }
    }

    static bool InputRow3(const char *id, const char *l1, double *v1, const char *l2, double *v2,
                          const char *l3, double *v3, const char *fmt, ImGuiInputTextFlags flags = 0) { //NOLINT
        bool edited = false;
        if (ImGui::BeginTable(id, 3, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            edited |= ImGui::InputDouble(l1, v1, 0, 0, fmt, flags);
            ImGui::TableSetColumnIndex(1);
            edited |= ImGui::InputDouble(l2, v2, 0, 0, fmt, flags);
            ImGui::TableSetColumnIndex(2);
            edited |= ImGui::InputDouble(l3, v3, 0, 0, fmt, flags);
            ImGui::EndTable();
        }
        return edited;
    }

    // ========== Render ==========
    void Render(bool *p_open) {
        if (!s_Initialized) {
            s_Initialized = true;
            SyncXYZtoBLH();
            SyncXYZtoENU();
        }

        ImGui::SetNextWindowSize(ImVec2(560, 340), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("坐标转换", p_open)) {
            ImGui::End();
            return;
        }

        // --- Frame selector ---
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("参考系:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        if (ImGui::Combo("##frame", &s_FrameIdx, FRAME_NAMES, FRAME_COUNT)) {
            SyncXYZtoBLH();
            SyncXYZtoENU();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // ======== XYZ <-> BLH ========
        ImGui::SeparatorText("XYZ <-> BLH");

        const bool xyz_edited = InputRow3("##xyz", "X (m)", &s_X, "Y (m)", &s_Y, "Z (m)", &s_Z, "%.4f");
        const bool blh_edited = InputRow3("##blh", "B (度)", &s_B_deg, "L (度)", &s_L_deg, "H (m)", &s_H, "%.9f");

        if (xyz_edited) {
            s_XyzBlhSrc = 0;
            SyncXYZtoBLH();
        } else if (blh_edited) {
            s_XyzBlhSrc = 1;
            SyncBLHtoXYZ();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // ======== ENU ========
        ImGui::SeparatorText("站心坐标 (ENU)");

        // Reference point
        ImGui::TextDisabled("参考点:");
        ImGui::SameLine();
        const bool ref_edited = InputRow3("##ref", "B (度)", &s_RefB_deg, "L (度)", &s_RefL_deg, "H (m)", &s_RefH, "%.9f");

        ImGui::Spacing();

        // XYZ <-> ENU rows
        const bool tgt_edited = InputRow3("##tgt", "目标 X (m)", &s_TgtX, "目标 Y (m)", &s_TgtY, "目标 Z (m)", &s_TgtZ, "%.4f");
        const bool enu_edited = InputRow3("##enu", "E (m)", &s_E, "N (m)", &s_N, "U (m)", &s_U, "%.4f");

        if (tgt_edited) {
            s_EnuSrc = 0;
            SyncXYZtoENU();
        } else if (enu_edited) {
            s_EnuSrc = 1;
            SyncENUtoXYZ();
        }

        if (ref_edited) {
            if (s_EnuSrc == 0) SyncXYZtoENU();
            else SyncENUtoXYZ();
        }

        ImGui::End();
    }
}
