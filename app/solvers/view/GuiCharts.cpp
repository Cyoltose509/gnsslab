#include "GuiCharts.h"

#include "Const.h"
#include "StringUtils.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <excpt.h>

#include "implot.h"
#include "implot3d.h"
#include "ui/GuiColors.h"

namespace Solvers::Charts {
    ImU32 getSatCol(const char s) {
        switch (s) {
            case 'G': return IM_COL32(80, 220, 120, 255);
            case 'C': return IM_COL32(255, 130, 60, 255);
            case 'R': return IM_COL32(80, 200, 255, 255);
            case 'E': return IM_COL32(240, 220, 70, 255);
            default: return IM_COL32(220, 220, 220, 255);
        }
    }


    void epochCursor(const int id, int *selectedEpoch, const int epochCount) {
        double sx = selectedEpoch ? static_cast<double>(*selectedEpoch) : 0.0;
        ImPlot::DragLineX(id, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
        if (!selectedEpoch) return;
        int se = static_cast<int>(std::lround(sx));
        if (se < 0) se = 0;
        if (epochCount > 0 && se >= epochCount) se = epochCount - 1;
        *selectedEpoch = se;
    }

    void renderSatelliteTable(const std::vector<SatRow> &rows, int &selectedSatIdx) {
        if (rows.empty()) {
            ImGui::TextDisabled("(无卫星)");
            return;
        }
        bool hasAzim = false, hasXYZ = false;
        for (const auto &r: rows) {
            if (r.hasAzim) hasAzim = true;
            if (r.hasSatXYZ) hasXYZ = true;
        }

        if (ImGui::BeginTable("##satTable", 2 + 1 + (hasAzim ? 1 : 0) + (hasXYZ ? 3 : 0) + 2 + 1,
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                              ImVec2(0, ImGui::GetContentRegionAvail().y * 0.8f))) {
            ImGui::TableSetupColumn("序号", ImGuiTableColumnFlags_WidthFixed, 44);
            ImGui::TableSetupColumn("卫星", ImGuiTableColumnFlags_WidthFixed, 56);
            ImGui::TableSetupColumn("仰角(°)", ImGuiTableColumnFlags_WidthFixed, 80);
            if (hasAzim) ImGui::TableSetupColumn("方位角(°)", ImGuiTableColumnFlags_WidthFixed, 80);
            if (hasXYZ) {
                ImGui::TableSetupColumn("X(m)", ImGuiTableColumnFlags_WidthFixed, 90);
                ImGui::TableSetupColumn("Y(m)", ImGuiTableColumnFlags_WidthFixed, 90);
                ImGui::TableSetupColumn("Z(m)", ImGuiTableColumnFlags_WidthFixed, 90);
            }
            ImGui::TableSetupColumn("伪距残差(m)", ImGuiTableColumnFlags_WidthFixed, 110);
            ImGui::TableSetupColumn("相位残差(m)", ImGuiTableColumnFlags_WidthFixed, 110);
            ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthFixed, 56);
            ImGui::TableHeadersRow();

            for (std::size_t i = 0; i < rows.size(); ++i) {
                const SatRow &r = rows[i];
                ImGui::TableNextRow();
                int c = 0;
                ImGui::TableSetColumnIndex(c++);
                ImGui::Text("%d", static_cast<int>(i) + 1);
                ImGui::TableSetColumnIndex(c++);
                if (ImGui::Selectable(r.sat.toString().c_str(), selectedSatIdx == static_cast<int>(i),
                                      ImGuiSelectableFlags_SpanAllColumns))
                    selectedSatIdx = static_cast<int>(i);
                ImGui::TableSetColumnIndex(c++);
                ImGui::Text("%.2f", r.elevDeg);
                if (hasAzim) {
                    ImGui::TableSetColumnIndex(c++);
                    ImGui::Text("%.2f", r.azimDeg);
                }
                if (hasXYZ) {
                    ImGui::TableSetColumnIndex(c++);
                    ImGui::Text("%.1f", r.satX);
                    ImGui::TableSetColumnIndex(c++);
                    ImGui::Text("%.1f", r.satY);
                    ImGui::TableSetColumnIndex(c++);
                    ImGui::Text("%.1f", r.satZ);
                }
                ImGui::TableSetColumnIndex(c++);
                ImGui::Text("%s", std::isfinite(r.codeRes) ? Str::fmt4(r.codeRes).c_str() : "-");
                ImGui::TableSetColumnIndex(c++);
                ImGui::Text("%s", std::isfinite(r.phaseRes) ? Str::fmt4(r.phaseRes).c_str() : "-");
                ImGui::TableSetColumnIndex(c++);
                if (r.used) ImGui::TextColored(UI::kColPass, "参与");
                else ImGui::TextColored(UI::kColFail, "排除");
            }
            ImGui::EndTable();
        }
    }


    void renderENUPlot(const BrokenAxis &axis,
                       const std::vector<double> &enu_e,
                       const std::vector<double> &enu_n,
                       const std::vector<double> &enu_u,
                       int &selectedEpoch, bool &newed,
                       const bool hasFloat,
                       const std::vector<double> &float_e,
                       const std::vector<double> &float_n,
                       const std::vector<double> &float_u,
                       const int epochCount,
                       const float plotH) {
        if (axis.kept.empty() || !ImPlot::BeginPlot("ENU 偏差", ImVec2(-1, plotH))) return;
        const int nEpoch = epochCount > 0 ? epochCount : static_cast<int>(axis.t.size());
        ImPlot::SetupAxes("Epoch", "E/N/U (m)");
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
        const int nf = static_cast<int>(axis.t.size());
        const std::vector<double> ee = axis.align(enu_e), nn = axis.align(enu_n), uu = axis.align(enu_u);
        std::vector<double> fe, fn, fu;
        if (hasFloat) {
            fe = axis.align(float_e);
            fn = axis.align(float_n);
            fu = axis.align(float_u);
        }
        xAxisFollow(axis.t, newed);
        newed = false;
        if (nf > 0) {
            if (hasFloat && !fe.empty()) {                ImPlotSpec fspec;
                fspec.LineColor = UI::kColFloat;
                fspec.LineWeight = 1.0f;
                ImPlot::PlotLine("E(浮点)", axis.t.data(), fe.data(), nf, fspec);
                ImPlot::PlotLine("N(浮点)", axis.t.data(), fn.data(), nf, fspec);
                ImPlot::PlotLine("U(浮点)", axis.t.data(), fu.data(), nf, fspec);
            }

            ImPlotSpec espec;
            espec.LineColor = UI::kColX;
            espec.LineWeight = 1.5f;
            ImPlot::PlotLine("E", axis.t.data(), ee.data(), nf, espec);
            ImPlotSpec nspec;
            nspec.LineColor = UI::kColY;
            nspec.LineWeight = 1.5f;
            ImPlot::PlotLine("N", axis.t.data(), nn.data(), nf, nspec);
            ImPlotSpec uspec;
            uspec.LineColor = UI::kColZ;
            uspec.LineWeight = 1.5f;
            ImPlot::PlotLine("U", axis.t.data(), uu.data(), nf, uspec);

            {
                std::vector<double> rms3d;
                rms3d.reserve(nf);
                for (int k = 0; k < nf; ++k)
                    rms3d.push_back(std::sqrt(ee[k] * ee[k] + nn[k] * nn[k] + uu[k] * uu[k]));
                ImPlotSpec rs;
                rs.LineColor = ImVec4(1.0f, 0.85f, 0.2f, 0.95f);
                rs.LineWeight = 1.5f;
                ImPlot::PlotLine("RMS 3D", axis.t.data(), rms3d.data(), nf, rs);
            }
        }
        if (nf > 0)
            epochCursor(0, &selectedEpoch, nEpoch);
        ImPlot::EndPlot();
    }


    void renderSatResidualPlot(const std::map<SatID, std::vector<double> > &satResV,
                               const std::map<SatID, std::vector<double> > &phaseV,
                               const std::map<SatID, BrokenAxis> &codeAxis,
                               const std::map<SatID, BrokenAxis> &phaseAxis,
                               int *resMode,
                               int &selectedEpoch, const int epochCount, bool &newed,
                               const double *codeLo, const double *codeHi,
                               const double *phaseLo, const double *phaseHi,
                               const float plotH) {

        int mode = resMode ? *resMode : 0;
        ImGui::PushID("resmode");
        if (ImGui::RadioButton("伪距", mode == 0)) mode = 0;
        ImGui::SameLine();
        if (ImGui::RadioButton("相位", mode == 1)) mode = 1;
        ImGui::PopID();
        if (resMode) *resMode = mode;
        const bool phase = mode == 1;

        const auto &V = phase ? phaseV : satResV;
        const auto &A = phase ? phaseAxis : codeAxis;
        const double *lo = phase ? phaseLo : codeLo;
        const double *hi = phase ? phaseHi : codeHi;

        if (phase && V.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("(无相位残差：该模式不使用载波相位)");
        }

        float ph = ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y;
        if (plotH > 0.0f && plotH < ph) ph = plotH;
        if (ph < 120.0f) ph = 120.0f;

        if (ImPlot::BeginPlot("##residualSeries", ImVec2(-1, ph), ImPlotFlags_NoTitle)) {
            ImPlot::SetupAxes("Epoch", phase ? "载波相位残差 (m)" : "码(伪距)残差 (m)");
            {
                ImPlot::SetupAxisLimits(ImAxis_X1, 0, std::max(10.0, static_cast<double>(epochCount)), ImPlotCond_Once);
                if (lo && hi && std::isfinite(*lo) && std::isfinite(*hi) && *hi > *lo)
                    ImPlot::SetupAxisLimits(ImAxis_Y1, *lo, *hi, ImPlotCond_Once);
            }

            for (auto &[sat, vals]: V) {
                const auto itA = A.find(sat);
                if (itA == A.end() || itA->second.kept.empty() || vals.empty()) continue;
                const BrokenAxis &axis = itA->second;
                const std::vector<double> vv = axis.align(vals);
                const std::string base = sat.toString();
                ImPlot::PlotLine(base.c_str(), axis.t.data(), vv.data(), static_cast<int>(axis.t.size()));
            }
            epochCursor(1, &selectedEpoch, epochCount);
            ImPlot::EndPlot();
        }
        newed = false;
    }

    void renderSkyplot(const std::map<SatID, std::vector<SkyPoint> > &tracks,
                       const std::vector<std::pair<SatID, SkyPoint> > &curPts,
                       const float sizePx) {
        const float availW = ImGui::GetContentRegionAvail().x;
        const float availH = ImGui::GetContentRegionAvail().y;
        float sizeW = std::min(availW, sizePx);
        if (availH > 60.0f) sizeW = std::min(sizeW, availH - ImGui::GetStyle().ItemSpacing.y);
        sizeW = std::max(160.0f, sizeW);
        constexpr float pad = 30.0f;
        const float R = std::max(40.0f, (sizeW - 2.0f * pad) / 2.0f);
        const ImVec2 c0 = ImGui::GetCursorScreenPos();
        const float cx = c0.x + pad + R, cy = c0.y + pad + R;
        ImGui::Dummy(ImVec2(sizeW, sizeW));
        ImDrawList *dl = ImGui::GetWindowDrawList();

        constexpr ImU32 grid = IM_COL32(90, 90, 110, 255);
        constexpr ImU32 gridFaint = IM_COL32(70, 70, 85, 200);
        dl->AddCircle(ImVec2(cx, cy), R, grid, 64, 1.5f);
        for (int el = 30; el < 90; el += 30) {
            const float r = R * (1.0f - static_cast<float>(el) / 90.0f);
            dl->AddCircle(ImVec2(cx, cy), r, gridFaint, 64, 1.0f);
        }
        for (int d = 0; d < 4; ++d) {
            const char *dirs[4] = {"N", "E", "S", "W"};
            const float a = static_cast<float>(d) * 90.0f * static_cast<float>(DEG_TO_RAD);
            const float x = cx + R * std::sin(a), y = cy - R * std::cos(a);
            dl->AddLine(ImVec2(cx, cy), ImVec2(x, y), gridFaint, 1.0f);
            const float lx = cx + (R + 14.0f) * std::sin(a), ly = cy - (R + 14.0f) * std::cos(a);
            dl->AddText(ImVec2(lx - 4.0f, ly - 8.0f), IM_COL32(200, 200, 220, 255), dirs[d]);
        }

        auto pt = [&](const float azDeg, const float elDeg) -> ImVec2 {
            const float el = std::clamp(elDeg, 0.0f, 90.0f);   // 防止非有限/越界值画出圆外
            const float r = R * (1.0f - el / 90.0f);
            const float a = azDeg * static_cast<float>(DEG_TO_RAD);
            return {cx + r * std::sin(a), cy - r * std::cos(a)};
        };

        for (auto &[sat, pts]: tracks) {
            if (pts.size() < 2) continue;
            ImU32 trkCol = getSatCol(sat.system);
            trkCol = IM_COL32((trkCol >> IM_COL32_R_SHIFT) & 0xFF,
                              (trkCol >> IM_COL32_G_SHIFT) & 0xFF,
                              (trkCol >> IM_COL32_B_SHIFT) & 0xFF, 120);

            // 先把每个点映射到屏幕坐标，再分段。分段依据：
            //  1) epIdx 间隔过大（卫星落到地平线下 / 数据中断）；
            //  2) 相邻两点屏幕距离过大（方位角绕回、跨地平线、或观测异常导致的跳变）。
            //  两者任一成立即断开，避免把圆两侧的点用一条直线（横穿线）相连。
            //  阈值 0.5R 远小于正常采样步长（≤0.02R），不会误断连续弧段。
            std::vector<ImVec2> sp(pts.size());
            for (std::size_t k = 0; k < pts.size(); ++k)
                sp[k] = pt(pts[k].azDeg, pts[k].elDeg);

            int gapThr = 2;
            {
                std::vector<int> gaps;
                gaps.reserve(pts.size());
                for (std::size_t i = 1; i < pts.size(); ++i)
                    gaps.push_back(pts[i].epIdx - pts[i - 1].epIdx);
                if (!gaps.empty()) {
                    const int minGap = *std::min_element(gaps.begin(), gaps.end());
                    gapThr = std::max(2, 3 * minGap);
                }
            }
            const float maxJump = 0.5f * R;

            int segStart = 0;
            for (std::size_t i = 1; i <= pts.size(); ++i) {
                bool brk = (i == pts.size());
                if (!brk) {
                    const int gap = pts[i].epIdx - pts[i - 1].epIdx;
                    const float dx = sp[i].x - sp[i - 1].x;
                    const float dy = sp[i].y - sp[i - 1].y;
                    if (gap > gapThr || (dx * dx + dy * dy) > maxJump * maxJump) brk = true;
                }
                if (brk) {
                    if (i - segStart >= 2) {
                        dl->AddPolyline(&sp[segStart], static_cast<int>(i - segStart),
                                        trkCol, ImDrawFlags_None, 1.5f);
                        for (int k = segStart; k < static_cast<int>(i); k += 5)
                            dl->AddCircleFilled(sp[k], 1.5f, trkCol);
                    }
                    segStart = static_cast<int>(i);
                }
            }
        }

        for (auto &[sat, sp]: curPts) {
            if (sp.elDeg <= 0.0f) continue;
            ImVec2 p = pt(sp.azDeg, sp.elDeg);
            const ImU32 col = getSatCol(sat.system);
            if (sp.used) {
                dl->AddCircleFilled(p, 6.0f, col);
                dl->AddText(ImVec2(p.x - 10.0f, p.y - 24.0f), col, sat.toString().c_str());
            } else {
                dl->AddCircle(p, 4.5f, IM_COL32(150, 150, 160, 200), 12, 1.0f);
            }
        }
    }


    void renderLeo3D(const Eigen::Vector3d &leoPos,
                     const std::vector<TrajPoint> &leoTraj,
                     const std::vector<TrajPoint> &refTraj,
                     const float sizePx) {
        if (leoTraj.size() < 2) {
            ImGui::TextDisabled("(轨迹点不足，无法绘制 3D)");
            return;
        }
        auto fillXYZ = [&](const std::vector<TrajPoint> &traj,
                           std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &zs) {
            xs.clear();
            ys.clear();
            zs.clear();
            for (const auto &[epIdx, pos]: traj) {
                const double x = pos.x(), y = pos.y(), z = pos.z();
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                if (pos.norm() < 1e3) continue;
                xs.push_back(x);
                ys.push_back(y);
                zs.push_back(z);
            }
        };
        std::vector<double> xs, ys, zs;
        fillXYZ(leoTraj, xs, ys, zs);
        if (xs.size() < 2) {
            ImGui::TextDisabled("(有效轨迹点不足，无法绘制 3D)");
            return;
        }
        std::vector<double> rx, ry, rz;
        if (!refTraj.empty()) fillXYZ(refTraj, rx, ry, rz);
        if (ImPlot3D::BeginPlot("LEO 轨迹 (ECEF XYZ)", ImVec2(-1, sizePx))) {
            ImPlot3D::SetupAxes("X (m)", "Y (m)", "Z (m)");
            ImPlot3D::PlotLine("LEO 估计", xs.data(), ys.data(), zs.data(), static_cast<int>(xs.size()));
            if (rx.size() >= 2)
                ImPlot3D::PlotLine("参考轨道", rx.data(), ry.data(), rz.data(), static_cast<int>(rx.size()));
            if (leoPos.norm() > 1e3 && std::isfinite(leoPos.x()) && std::isfinite(leoPos.y()) && std::isfinite(leoPos.z())) {
                const double px[1] = {leoPos.x()}, py[1] = {leoPos.y()}, pz[1] = {leoPos.z()};
                ImPlot3D::PlotScatter("当前位置", px, py, pz, 1);
            }
            ImPlot3D::EndPlot();
        }
    }
}
