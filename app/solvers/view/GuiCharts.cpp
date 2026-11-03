#include "GuiCharts.h"

#include "Const.h"
#include "StringUtils.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <excpt.h>

#include "implot.h"
#include "implot3d.h"

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


    BrokenSeries::BrokenSeries(const std::vector<double> &times) {
        const int n = static_cast<int>(times.size());
        double step = 1.0;
        {
            std::vector<double> d;
            d.reserve(n > 1 ? n - 1 : 0);
            for (int i = 1; i < n; ++i) {
                if (std::isfinite(times[i]) && std::isfinite(times[i - 1]) && times[i] > times[i - 1])
                    d.push_back(times[i] - times[i - 1]);
            }
            if (!d.empty()) {
                std::sort(d.begin(), d.end());
                step = d[d.size() / 2];
            }
        }
        const double thr = 2.5 * step;
        for (int i = 0; i < n; ++i) {
            if (!std::isfinite(times[i])) continue;

            if (!kept.empty() && times[i] - t.back() > thr) {
                t.push_back(t.back());
                kept.push_back(-1);
            }
            t.push_back(times[i]);
            kept.push_back(i);
        }
    }

    std::vector<double> BrokenSeries::align(const std::vector<double> &y) const {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        std::vector<double> vv;
        vv.reserve(kept.size());
        for (const int k: kept) {
            if (k < 0) {
                vv.push_back(nan);
                continue;
            }
            vv.push_back(k < static_cast<int>(y.size()) && std::isfinite(y[k]) ? y[k] : nan);
        }
        return vv;
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


    void renderENUPlot(const std::vector<double> &times,
                       const std::vector<double> &enu_e,
                       const std::vector<double> &enu_n,
                       const std::vector<double> &enu_u,
                       int &selectedEpoch, bool &newed, const bool fixedY,
                       const bool hasFloat,
                       const std::vector<double> &float_e,
                       const std::vector<double> &float_n,
                       const std::vector<double> &float_u,
                       const int epochCount,
                       const float plotH) {
        const int n = static_cast<int>(times.size());
        const int nEpoch = epochCount > 0 ? epochCount : n;
        if (n <= 0 || !ImPlot::BeginPlot("ENU 偏差", ImVec2(-1, plotH))) return;
        ImPlot::SetupAxes("Epoch", "E/N/U (m)");
        if (fixedY) ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
        const BrokenSeries br(times);
        const int nf = static_cast<int>(br.t.size());
        const std::vector<double> ee = br.align(enu_e), nn = br.align(enu_n), uu = br.align(enu_u);
        std::vector<double> fe, fn, fu;
        if (hasFloat) {
            fe = br.align(float_e);
            fn = br.align(float_n);
            fu = br.align(float_u);
        }
        xAxisFollow(br.t, newed);
        newed = false;
        if (nf > 0) {
            if (hasFloat && !fe.empty()) {                ImPlotSpec fspec;
                fspec.LineColor = UI::kColFloat;
                fspec.LineWeight = 1.0f;
                ImPlot::PlotLine("E(浮点)", br.t.data(), fe.data(), nf, fspec);
                ImPlot::PlotLine("N(浮点)", br.t.data(), fn.data(), nf, fspec);
                ImPlot::PlotLine("U(浮点)", br.t.data(), fu.data(), nf, fspec);
            }

            ImPlotSpec espec;
            espec.LineColor = UI::kColX;
            espec.LineWeight = 1.5f;
            ImPlot::PlotLine("E", br.t.data(), ee.data(), nf, espec);
            ImPlotSpec nspec;
            nspec.LineColor = UI::kColY;
            nspec.LineWeight = 1.5f;
            ImPlot::PlotLine("N", br.t.data(), nn.data(), nf, nspec);
            ImPlotSpec uspec;
            uspec.LineColor = UI::kColZ;
            uspec.LineWeight = 1.5f;
            ImPlot::PlotLine("U", br.t.data(), uu.data(), nf, uspec);

            {
                std::vector<double> rms3d;
                rms3d.reserve(nf);
                for (int k = 0; k < nf; ++k)
                    rms3d.push_back(std::sqrt(ee[k] * ee[k] + nn[k] * nn[k] + uu[k] * uu[k]));
                ImPlotSpec rs;
                rs.LineColor = ImVec4(1.0f, 0.85f, 0.2f, 0.95f);
                rs.LineWeight = 1.5f;
                ImPlot::PlotLine("RMS 3D", br.t.data(), rms3d.data(), nf, rs);
            }
        }
        if (nf > 0)
            epochCursor(0, &selectedEpoch, nEpoch);
        ImPlot::EndPlot();
    }


    void renderRatioPlot(const std::vector<double> &times,
                         const std::vector<double> &ratioVals,
                         int &selectedEpoch, bool &newed, const int epochCount) {
        const int n = static_cast<int>(times.size());
        const int nEpoch = epochCount > 0 ? epochCount : n;
        if (n <= 0 || !ImPlot::BeginPlot("Ratio 时序", ImVec2(-1, 300))) return;
        ImPlot::SetupAxes("Epoch", "Ratio");
        const BrokenSeries br(times);
        const int nf = static_cast<int>(br.t.size());
        const std::vector<double> rr = br.align(ratioVals);
        xAxisFollow(br.t, newed);
        newed = false;
        if (nf > 0) {

            if (br.t.front() <= br.t.back()) {
                double yt[2] = {br.t.front(), br.t.back()};
                double yv[2] = {3.0, 3.0};
                ImPlotSpec thr;
                thr.LineColor = ImVec4(1.0f, 0.6f, 0.1f, 0.9f);
                thr.LineWeight = 1.5f;
                ImPlot::PlotLine("阈值 3.0", yt, yv, 2, thr);
            }
            ImPlotSpec rspec;
            rspec.LineColor = ImVec4(0.4f, 0.8f, 1.0f, 1.0f);
            rspec.LineWeight = 1.5f;
            ImPlot::PlotLine("Ratio", br.t.data(), rr.data(), nf, rspec);
        }
        if (nf > 0)
            epochCursor(5, &selectedEpoch, nEpoch);
        ImPlot::EndPlot();
    }


    void renderSatResidualPlot(const std::map<SatID, std::vector<double> > &satResT,
                               const std::map<SatID, std::vector<double> > &satResV,
                               const std::map<SatID, std::vector<double> > &phaseT,
                               const std::map<SatID, std::vector<double> > &phaseV,
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

        const auto &T = phase ? phaseT : satResT;
        const auto &V = phase ? phaseV : satResV;
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

            auto plotOne = [&](const std::map<SatID, std::vector<double> > &Ts,
                               const std::map<SatID, std::vector<double> > &Vs,
                               const std::string &suffix) {
                for (auto &[sat, vals]: Vs) {
                    auto it = Ts.find(sat);
                    if (it == Ts.end() || vals.empty()) continue;
                    const std::string base = sat.toString() + suffix;
                    const BrokenSeries br(it->second);
                    const std::vector<double> vv = br.align(vals);
                    if (br.t.empty()) continue;
                    ImPlot::PlotLine(base.c_str(), br.t.data(), vv.data(), static_cast<int>(br.t.size()));
                }
            };

            plotOne(T, V, "");
            epochCursor(1, &selectedEpoch, epochCount);
            ImPlot::EndPlot();
        }
        newed = false;
    }

    void renderSkyplot(const std::map<SatID, std::vector<Solvers::SkyPoint> > &tracks,
                       const std::vector<std::pair<SatID, Solvers::SkyPoint> > &curPts,
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
            const float r = R * (1.0f - elDeg / 90.0f);
            const float a = azDeg * static_cast<float>(DEG_TO_RAD);
            return {cx + r * std::sin(a), cy - r * std::cos(a)};
        };

        for (auto &[sat, pts]: tracks) {
            if (pts.size() < 2) continue;
            ImU32 trkCol = getSatCol(sat.system);
            trkCol = IM_COL32((trkCol >> IM_COL32_R_SHIFT) & 0xFF,
                              (trkCol >> IM_COL32_G_SHIFT) & 0xFF,
                              (trkCol >> IM_COL32_B_SHIFT) & 0xFF, 120);
            int segStart = 0;
            int gapThr = 2;
            {
                std::vector<int> gaps;
                gaps.reserve(pts.size());
                for (std::size_t i = 1; i < pts.size(); ++i)
                    gaps.push_back(pts[i].epIdx - pts[i - 1].epIdx);
                if (!gaps.empty()) {
                    std::sort(gaps.begin(), gaps.end());
                    gapThr = std::max(2, 3 * gaps[gaps.size() / 2]);
                }
            }
            for (std::size_t i = 1; i <= pts.size(); ++i) {
                if (i == pts.size() || pts[i].epIdx - pts[i - 1].epIdx > gapThr) {
                    if (i - segStart >= 2) {
                        std::vector<ImVec2> seg;
                        for (int k = segStart; k < static_cast<int>(i); ++k)
                            seg.push_back(pt(pts[k].azDeg, pts[k].elDeg));
                        dl->AddPolyline(seg.data(), static_cast<int>(seg.size()), trkCol, ImDrawFlags_None, 1.5f);
                        for (int k = segStart; k < static_cast<int>(i); k += 5)
                            dl->AddCircleFilled(seg[k - segStart], 1.5f, trkCol);
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


    static void renderLeo3DImpl(const Eigen::Vector3d &leoPos,
                                const std::vector<Solvers::TrajPoint> &leoTraj,
                                const std::vector<Solvers::TrajPoint> &refTraj,
                                const float sizePx) {
        if (leoTraj.size() < 2) {
            ImGui::TextDisabled("(轨迹点不足，无法绘制 3D)");
            return;
        }
        auto fillXYZ = [&](const std::vector<Solvers::TrajPoint> &traj,
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

        if (ImPlot3D::BeginPlot("LEO 轨迹 (ECEF XYZ)", ImVec2(-1, sizePx))) {
            ImPlot3D::SetupAxes("X (m)", "Y (m)", "Z (m)");
            ImPlot3D::PlotLine("LEO 估计", xs.data(), ys.data(), zs.data(), static_cast<int>(xs.size()));
            if (!refTraj.empty()) {
                std::vector<double> rx, ry, rz;
                fillXYZ(refTraj, rx, ry, rz);
                if (rx.size() >= 2)
                    ImPlot3D::PlotLine("参考轨道", rx.data(), ry.data(), rz.data(), static_cast<int>(rx.size()));
            }
            if (leoPos.norm() > 1e3 && std::isfinite(leoPos.x()) && std::isfinite(leoPos.y()) && std::isfinite(leoPos.z())) {
                const double px[1] = {leoPos.x()}, py[1] = {leoPos.y()}, pz[1] = {leoPos.z()};
                ImPlot3D::PlotScatter("当前位置", px, py, pz, 1);
            }
            ImPlot3D::EndPlot();
        }
    }


    void renderLeo3D(const Eigen::Vector3d &leoPos,
                     const std::vector<Solvers::TrajPoint> &leoTraj,
                     const std::vector<Solvers::TrajPoint> &refTraj,
                     const float sizePx) {
        __try {
            renderLeo3DImpl(leoPos, leoTraj, refTraj, sizePx);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ImGui::TextColored(UI::kColFail, "LEO 3D 轨迹渲染异常，已跳过");
        }
    }
}
