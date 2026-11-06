#include "GuiCoordConverter.h"
#include "imgui.h"
#include "ui/GuiColors.h"
#include "Const.h"
#include "CoordStruct.h"
#include "CoordConvert.h"

#include <exception>
#include <iterator>
#include <string>


namespace Tools::CoordConverter {
    struct FrameItem {
        const char *name;
        const FrameInfo *info;
    };

    constexpr FrameItem kFrames[] = {
        {"WGS84", &Frame::WGS84},
        {"GPS", &Frame::GPS},
        {"CGCS2000", &Frame::CGCS2000},
        {"PZ90", &Frame::PZ90},
    };
    constexpr int kFrameCount = std::size(kFrames);

    static int s_FrameIdx = 0;


    static double s_X = -2700000.0;
    static double s_Y = 4700000.0;
    static double s_Z = 3300000.0;

    static double s_B_deg = 0.0;
    static double s_L_deg = 0.0;
    static double s_H = 0.0;


    static double s_RefB_deg = 30.0;
    static double s_RefL_deg = 114.0;
    static double s_RefH = 50.0;

    static double s_TgtX = -2700000.0;
    static double s_TgtY = 4700000.0;
    static double s_TgtZ = 3300000.0;

    static double s_E = 0.0;
    static double s_N = 0.0;
    static double s_U = 0.0;


    /// 用户最后改的是哪一侧：0 = 目标 XYZ，1 = ENU；改参考点后按这一侧重算另一侧。
    static int s_EnuSrc = 0;

    static bool s_Initialized = false;
    static std::string s_Error;


    static const FrameInfo &CurFrame() { return *kFrames[s_FrameIdx].info; }

    /// 跑一次换算：失败原因留在面板上，而不是静默吞掉让界面看着没反应。
    template <typename F>
    static void attempt(const char *what, F &&fn) {
        try {
            fn();
            s_Error.clear();
        } catch (const std::exception &e) {
            s_Error = std::string(what) + ": " + e.what();
        } catch (...) {
            s_Error = std::string(what) + " 失败";
        }
    }

    static void SyncXYZToBLH() {
        attempt("XYZ→BLH", [] {
            const BLH blh = Coord::XYZToBLH({s_X, s_Y, s_Z}, CurFrame());
            s_B_deg = blh.B() * RAD_TO_DEG;
            s_L_deg = blh.L() * RAD_TO_DEG;
            s_H = blh.H();
        });
    }

    static void SyncBLHToXYZ() {
        attempt("BLH→XYZ", [] {
            const XYZ xyz = Coord::BLHToXYZ({s_B_deg * DEG_TO_RAD, s_L_deg * DEG_TO_RAD, s_H}, CurFrame());
            s_X = xyz.X();
            s_Y = xyz.Y();
            s_Z = xyz.Z();
        });
    }

    static void SyncXYZToENU() {
        attempt("XYZ→ENU", [] {
            const XYZ refXYZ = Coord::BLHToXYZ({s_RefB_deg * DEG_TO_RAD, s_RefL_deg * DEG_TO_RAD, s_RefH}, CurFrame());
            const ENU enu = Coord::XYZToENU({s_TgtX, s_TgtY, s_TgtZ}, refXYZ, CurFrame());
            s_E = enu.E();
            s_N = enu.N();
            s_U = enu.U();
        });
    }

    static void SyncENUToXYZ() {
        attempt("ENU→XYZ", [] {
            const XYZ refXYZ = Coord::BLHToXYZ({s_RefB_deg * DEG_TO_RAD, s_RefL_deg * DEG_TO_RAD, s_RefH}, CurFrame());
            const XYZ xyz = Coord::ENUToXYZ({s_E, s_N, s_U}, refXYZ, CurFrame());
            s_TgtX = xyz.X();
            s_TgtY = xyz.Y();
            s_TgtZ = xyz.Z();
        });
    }

    static bool InputRow3(const char *id, const char *l1, double *v1, const char *l2, double *v2,
                          const char *l3, double *v3, const char *fmt, const ImGuiInputTextFlags flags = 0) {
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

    static bool frameCombo() {
        bool changed = false;
        ImGui::SetNextItemWidth(120);
        if (ImGui::BeginCombo("##frame", kFrames[s_FrameIdx].name)) {
            for (int i = 0; i < kFrameCount; ++i) {
                if (ImGui::Selectable(kFrames[i].name, i == s_FrameIdx)) {
                    s_FrameIdx = i;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        return changed;
    }

    void Render(bool *p_open) {
        if (!s_Initialized) {
            s_Initialized = true;
            SyncXYZToBLH();
            SyncXYZToENU();
        }

        ImGui::SetNextWindowSize(ImVec2(560, 340), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("坐标转换", p_open)) {
            ImGui::End();
            return;
        }

        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("参考系:");
        ImGui::SameLine();
        if (frameCombo()) {
            SyncXYZToBLH();
            SyncXYZToENU();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();


        ImGui::SeparatorText("XYZ <-> BLH");

        const bool xyz_edited = InputRow3("##xyz", "X (m)", &s_X, "Y (m)", &s_Y, "Z (m)", &s_Z, "%.4f");
        const bool blh_edited = InputRow3("##blh", "B (度)", &s_B_deg, "L (度)", &s_L_deg, "H (m)", &s_H, "%.9f");

        if (xyz_edited) {
            SyncXYZToBLH();
        } else if (blh_edited) {
            SyncBLHToXYZ();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();


        ImGui::SeparatorText("站心坐标 (ENU)");


        ImGui::TextDisabled("参考点:");
        ImGui::SameLine();
        const bool ref_edited = InputRow3("##ref", "B (度)", &s_RefB_deg, "L (度)", &s_RefL_deg, "H (m)", &s_RefH, "%.9f");

        ImGui::Spacing();


        const bool tgt_edited = InputRow3("##tgt", "目标 X (m)", &s_TgtX, "目标 Y (m)", &s_TgtY, "目标 Z (m)", &s_TgtZ, "%.4f");
        const bool enu_edited = InputRow3("##enu", "E (m)", &s_E, "N (m)", &s_N, "U (m)", &s_U, "%.4f");

        if (tgt_edited) {
            s_EnuSrc = 0;
            SyncXYZToENU();
        } else if (enu_edited) {
            s_EnuSrc = 1;
            SyncENUToXYZ();
        }

        if (ref_edited) {
            if (s_EnuSrc == 0) SyncXYZToENU();
            else SyncENUToXYZ();
        }

        if (!s_Error.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(UI::kColFail, "%s", s_Error.c_str());
        }

        ImGui::End();
    }
}
