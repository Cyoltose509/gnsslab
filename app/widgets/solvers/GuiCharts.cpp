#define NOMINMAX
#include "GuiCharts.h"

#include "Const.h"
#include "CoordConvert.h" // ENUToXYZ（E-N 中心点拖拽 → 参考坐标平移）
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <cfloat>
#include <limits>
#include <set>
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

    // 断点感知时间轴：相邻点 x 间隔超过 ~2.5×典型间隔则插入一个 NaN 断点
    static void buildBrokenAxis(const std::vector<double> &times,
                                std::vector<double> &tt,
                                std::vector<int> &keptIdx) {
        tt.clear();
        keptIdx.clear();
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
            if (!keptIdx.empty() && times[i] - tt.back() > thr) {
                tt.push_back(tt.back());
                keptIdx.push_back(-1);
            }
            tt.push_back(times[i]);
            keptIdx.push_back(i);
        }
    }

    static void alignBrokenY(const std::vector<int> &keptIdx,
                             const std::vector<double> &y,
                             std::vector<double> &vv) {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        vv.clear();
        vv.reserve(keptIdx.size());
        for (const int k: keptIdx) {
            if (k < 0) {
                vv.push_back(nan);
                continue;
            }
            vv.push_back(k < static_cast<int>(y.size()) && std::isfinite(y[k]) ? y[k] : nan);
        }
    }

    void renderSatelliteTable(const std::vector<SatRow> &rows, int &selectedSatIdx, const char *id) {
        if (rows.empty()) {
            ImGui::TextDisabled("(无卫星)");
            return;
        }
        bool hasAzim = false, hasXYZ = false;
        for (const auto &r: rows) {
            if (r.hasAzim) hasAzim = true;
            if (r.hasSatXYZ) hasXYZ = true;
        }
        size_t nExtra = 0;
        for (const auto &r: rows) nExtra = std::max(nExtra, r.extra.size());

        if (ImGui::BeginTable(id, 2 + 1 + (hasAzim ? 1 : 0) + (hasXYZ ? 3 : 0) + static_cast<int>(nExtra) + 1,
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
            for (size_t k = 0; k < nExtra; ++k) {
                const char *hdr = k < rows.front().extra.size() ? rows.front().extra[k].first.c_str() : "?";
                ImGui::TableSetupColumn(hdr, ImGuiTableColumnFlags_WidthFixed, 110);
            }
            ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthFixed, 56);
            ImGui::TableHeadersRow();

            for (size_t i = 0; i < rows.size(); ++i) {
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
                for (size_t k = 0; k < nExtra; ++k) {
                    ImGui::TableSetColumnIndex(c++);
                    if (k < r.extra.size()) ImGui::Text("%s", r.extra[k].second.c_str());
                    else ImGui::TextDisabled("-");
                }
                ImGui::TableSetColumnIndex(c++); //NOLINT
                if (r.used) ImGui::TextColored(ImVec4(0.3f, 1, 0.3f, 1), "参与");
                else ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "排除");
            }
            ImGui::EndTable();
        }
    }

    // ENU 收敛轨迹图
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
        std::vector<double> tt, fe, fn, fu;
        std::vector<int> kept;
        buildBrokenAxis(times, tt, kept);
        const int nf = static_cast<int>(tt.size());
        std::vector<double> ee, nn, uu;
        ee.reserve(nf);
        nn.reserve(nf);
        uu.reserve(nf);
        alignBrokenY(kept, enu_e, ee);
        alignBrokenY(kept, enu_n, nn);
        alignBrokenY(kept, enu_u, uu);
        if (hasFloat) {
            fe.reserve(nf);
            fn.reserve(nf);
            fu.reserve(nf);
            alignBrokenY(kept, float_e, fe);
            alignBrokenY(kept, float_n, fn);
            alignBrokenY(kept, float_u, fu);
        }
        if (newed && nf > 0 && std::isfinite(tt.back()))
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Always);
        newed = false;
        if (nf > 0) {
            if (hasFloat && !fe.empty()) {
                ImPlotSpec fspec;
                fspec.LineColor = ImVec4(0.75f, 0.75f, 0.75f, 0.35f);
                fspec.LineWeight = 1.0f;
                ImPlot::PlotLine("E(浮点)", tt.data(), fe.data(), nf, fspec);
                ImPlot::PlotLine("N(浮点)", tt.data(), fn.data(), nf, fspec);
                ImPlot::PlotLine("U(浮点)", tt.data(), fu.data(), nf, fspec);
            }
            // 主曲线（固定解优先，否则浮点解）
            ImPlotSpec espec;
            espec.LineColor = ImVec4(0.9f, 0.3f, 0.3f, 1.0f);
            espec.LineWeight = 1.5f;
            ImPlot::PlotLine("E", tt.data(), ee.data(), nf, espec);
            ImPlotSpec nspec;
            nspec.LineColor = ImVec4(0.3f, 0.9f, 0.3f, 1.0f);
            nspec.LineWeight = 1.5f;
            ImPlot::PlotLine("N", tt.data(), nn.data(), nf, nspec);
            ImPlotSpec uspec;
            uspec.LineColor = ImVec4(0.3f, 0.5f, 1.0f, 1.0f);
            uspec.LineWeight = 1.5f;
            ImPlot::PlotLine("U", tt.data(), uu.data(), nf, uspec);

            {
                std::vector<double> rms3d;
                rms3d.reserve(nf);
                for (int k = 0; k < nf; ++k)
                    rms3d.push_back(std::sqrt(ee[k] * ee[k] + nn[k] * nn[k] + uu[k] * uu[k]));
                ImPlotSpec rs;
                rs.LineColor = ImVec4(1.0f, 0.85f, 0.2f, 0.95f);
                rs.LineWeight = 1.5f;
                ImPlot::PlotLine("RMS 3D", tt.data(), rms3d.data(), nf, rs);
            }
        }
        if (nf > 0) {
            auto sx = static_cast<double>(selectedEpoch);
            ImPlot::DragLineX(0, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
            int se = static_cast<int>(std::lround(sx));
            if (se < 0) se = 0;
            if (se >= nEpoch) se = nEpoch - 1; // 钳制到完整历元数（X 轴为历元序号）
            selectedEpoch = se;
        }
        ImPlot::EndPlot();
    }

    // Ratio 时序图
    void renderRatioPlot(const std::vector<double> &times,
                         const std::vector<double> &ratioVals,
                         int &selectedEpoch, bool &newed, const int epochCount) {
        const int n = static_cast<int>(times.size());
        const int nEpoch = epochCount > 0 ? epochCount : n;
        if (n <= 0 || !ImPlot::BeginPlot("Ratio 时序", ImVec2(-1, 300))) return;
        ImPlot::SetupAxes("Epoch", "Ratio");
        std::vector<double> tt, rr;
        std::vector<int> kept;
        buildBrokenAxis(times, tt, kept);
        const int nf = static_cast<int>(tt.size());
        alignBrokenY(kept, ratioVals, rr);
        if (newed && nf > 0 && std::isfinite(tt.back()))
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Always);
        newed = false;
        if (nf > 0) {
            // 3.0 阈值横线
            if (tt.front() <= tt.back()) {
                double yt[2] = {tt.front(), tt.back()};
                double yv[2] = {3.0, 3.0};
                ImPlotSpec thr;
                thr.LineColor = ImVec4(1.0f, 0.6f, 0.1f, 0.9f);
                thr.LineWeight = 1.5f;
                ImPlot::PlotLine("阈值 3.0", yt, yv, 2, thr);
            }
            ImPlotSpec rspec;
            rspec.LineColor = ImVec4(0.4f, 0.8f, 1.0f, 1.0f);
            rspec.LineWeight = 1.5f;
            ImPlot::PlotLine("Ratio", tt.data(), rr.data(), nf, rspec);
        }
        if (nf > 0) {
            double sx = selectedEpoch;
            ImPlot::DragLineX(5, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
            int se = static_cast<int>(std::lround(sx));
            if (se < 0) se = 0;
            if (se >= nEpoch) se = nEpoch - 1;
            selectedEpoch = se;
        }
        ImPlot::EndPlot();
    }

    // 逐卫星残差图（码/相位可切换）
    void renderSatResidualPlot(const std::map<SatID, std::vector<double> > &satResT,
                               const std::map<SatID, std::vector<double> > &satResV,
                               const std::map<SatID, std::vector<double> > &phaseT,
                               const std::map<SatID, std::vector<double> > &phaseV,
                               int *resMode,
                               int &selectedEpoch, const int epochCount, bool &newed,
                               const double *codeLo, const double *codeHi,
                               const double *phaseLo, const double *phaseHi,
                               const float plotH) {
        // 切换：码(伪距) / 载波相位。状态由调用方持有（SolveTask::resMode），这里只读写。
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

        // SPP 不做载波相位，相位残差恒空：明说一句，避免误以为图坏了。
        // ⚠️ 必须与切换按钮同行：曾写成独立的一行 TextDisabled，切到相位时整块内容纵向撑高、
        // 容器的固定高度装不下 → 父 Child 长出上下滑条，绘图区被裁出可视范围（图"消失"）。
        if (phase && V.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("(无相位残差：该模式不使用载波相位)");
        }

        // 高度一律取「容器剩余空间」，且不超过调用方给的上限（plotH）：
        // 写死 plotH 会让 绘图区+单选行 的总高超过父 Child，父容器出滑条。
        // 再减一个 ItemSpacing：ImPlot 之后的行距也会计入子窗口内容高度，不减会顶出 3~4px 滑条。
        float ph = ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y;
        if (plotH > 0.0f && plotH < ph) ph = plotH;
        if (ph < 120.0f) ph = 120.0f;

        // 标题由外层 SectionHeader 负责，故关掉 ImPlot 自带标题（省一行高度）；
        // 同时标签固定为 "##residualSeries"，切模式不换 ID，避免切一次重建一次绘图状态。
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
                    // 不许静默丢整颗星：times/vals 理论上逐历元同步 push，长度应相等；
                    // 万一失同步（历史数据、外部注入），按重叠前缀画，尾部缺失只丢点不丢星。
                    const auto n = static_cast<int>(std::min(it->second.size(), vals.size()));
                    if (n <= 0) continue;
                    const double *t = it->second.data();
                    const double *v = vals.data();
                    const std::string base = sat.toString() + suffix;
                    std::vector<double> tt, vv;
                    tt.reserve(n + 4);
                    vv.reserve(n + 4);
                    for (int i = 0; i < n; ++i) {
                        if (!std::isfinite(v[i])) continue;
                        if (i > 0 && std::isfinite(t[i]) && std::isfinite(t[i - 1]) && t[i] - t[i - 1] > 2.5) {
                            tt.push_back(t[i - 1]);
                            vv.push_back(std::numeric_limits<double>::quiet_NaN()); // 断点：前后两段不相连
                        }
                        tt.push_back(t[i]);
                        vv.push_back(v[i]);
                    }
                    if (!tt.empty() && tt.size() == vv.size())
                        ImPlot::PlotLine(base.c_str(), tt.data(), vv.data(), static_cast<int>(tt.size()));
                }
            };

            plotOne(T, V, "");
            double sx = selectedEpoch;
            ImPlot::DragLineX(1, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
            int se = static_cast<int>(std::lround(sx));
            if (se < 0) se = 0;
            if (se >= epochCount) se = epochCount - 1;
            selectedEpoch = se;
            ImPlot::EndPlot();
        }
        newed = false;
    }

    void RenderSkyplot(const std::map<SatID, std::vector<GnssTask::SkyPoint> > &tracks,
                       const std::vector<std::pair<SatID, GnssTask::SkyPoint> > &curPts,
                       const float sizePx) {
        // 天顶图是正方形：宽、高都要受容器约束。只按宽度取尺寸时，若容器高度不足，
        // 正方形会撑出父 Child → 天顶图所在面板出现上下滑条（此前即如此）。
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
                for (size_t i = 1; i < pts.size(); ++i)
                    gaps.push_back(pts[i].epIdx - pts[i - 1].epIdx);
                if (!gaps.empty()) {
                    std::sort(gaps.begin(), gaps.end());
                    gapThr = std::max(2, 3 * gaps[gaps.size() / 2]);
                }
            }
            for (size_t i = 1; i <= pts.size(); ++i) {
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

    // LEO 轨迹（实现：绘制逻辑本体，无 SEH；独立成自由函数以避开 __try 与局部对象展开的 C2712 冲突）
    static void RenderLeo3DImpl(const Eigen::Vector3d &leoPos,
                                const std::vector<GnssTask::TrajPoint> &leoTraj,
                                const std::vector<GnssTask::TrajPoint> &refTraj,
                                const float sizePx) {
        if (leoTraj.size() < 2) {
            ImGui::TextDisabled("(轨迹点不足，无法绘制 3D)");
            return;
        }
        auto fillXYZ = [&](const std::vector<GnssTask::TrajPoint> &traj,
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

    // LEO 轨迹（SEH 包裹，渲染异常不崩溃）
    void RenderLeo3D(const Eigen::Vector3d &leoPos,
                     const std::vector<GnssTask::TrajPoint> &leoTraj,
                     const std::vector<GnssTask::TrajPoint> &refTraj,
                     const float sizePx) {
        __try {
            RenderLeo3DImpl(leoPos, leoTraj, refTraj, sizePx);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "LEO 3D 轨迹渲染异常，已跳过");
        }
    }

    namespace {
        // 章节标题；flag 非空时在标题右侧补一个 显示/隐藏 开关
        void SectionHeader(const char *title, bool *flag) {
            ImGui::SeparatorText(title);
            if (flag) {
                const float bw = ImGui::CalcTextSize("显示").x
                                 + ImGui::GetStyle().FramePadding.x * 2.0f
                                 + ImGui::GetStyle().ItemSpacing.x;
                ImGui::SameLine(ImGui::GetContentRegionAvail().x - bw);
                ImGui::PushID(title);
                if (ImGui::SmallButton(*flag ? "隐藏" : "显示"))
                    *flag = !*flag;
                ImGui::PopID();
            }
        }

        // 可见性兜底：LEO 走 renderToggles，SPP/PPP 走 PosTabView 的静态开关
        bool vis(const bool *flag, const bool fallback) {
            return flag ? *flag : fallback;
        }

        // 顶部状态行 + 历元导航（首/末/前后 + 滑块 + 历元信息）
        void renderStatusAndEpochNav(const PosTabView &v) {
            // 状态行
            if (!v.statusText.empty())
                ImGui::TextColored(v.statusColor, "%s", v.statusText.c_str());
            else if (v.epochCount > 0)
                ImGui::TextColored(ImVec4(0.3f, 1, 0.3f, 1), "处理完成: 共 %d 个历元", v.epochCount);

            // 历元导航：先 Clamp，防御解算线程/上帧把 selectedEpoch 改出当前范围。
            if (v.selectedEpoch && v.epochCount > 0) {
                int &se = *v.selectedEpoch;
                if (se < 0) se = 0;
                if (se >= v.epochCount) se = v.epochCount - 1;
            }

            // 历元导航（箭头 + 滑块 + 信息）
            if (v.epochCount > 0 && v.selectedEpoch) {
                if (ImGui::Button("<<")) *v.selectedEpoch = 0;
                ImGui::SameLine();
                if (ImGui::Button("<")) { if (*v.selectedEpoch > 0) (*v.selectedEpoch)--; }
                ImGui::SameLine();
                ImGui::PushItemWidth(150);
                int ep = *v.selectedEpoch + 1;
                if (ImGui::SliderInt("##es", &ep, 1, v.epochCount)) *v.selectedEpoch = ep - 1;
                ImGui::PopItemWidth();
                ImGui::SameLine();
                if (ImGui::Button(">")) { if (*v.selectedEpoch < v.epochCount - 1) (*v.selectedEpoch)++; }
                ImGui::SameLine();
                if (ImGui::Button(">>")) *v.selectedEpoch = v.epochCount - 1;
                ImGui::SameLine();
                if (!v.epochInfo.empty())
                    ImGui::TextDisabled("| %s", v.epochInfo.c_str());
                if (v.showFixRate) {
                    ImGui::SameLine();
                    ImGui::Text("| %s", v.fixRateText.c_str());
                }
            }
        }

        // 参考真值 ECEF 输入（含剪贴板坐标解析）
        // 注意：用户一旦手动修改，即视为「提供了参考真值」(userRefProvided=true)，
        // 后续解算完成时的「均值确认」不再覆盖（见 GuiSolverPanel 的 maybeConfirmRef）。
        void renderRefECEFInput(const PosTabView &v) {
            if (v.refECEF && v.showRefEnu && v.showRefInput) {
                ImGui::Text("参考真值:");
                ImGui::SameLine();
                ImGui::PushItemWidth(200);
                bool changed = false;
                if (ImGui::InputDouble("X/m", &(*v.refECEF)[0])) changed = true;
                ImGui::SameLine();
                if (ImGui::InputDouble("Y/m", &(*v.refECEF)[1])) changed = true;
                ImGui::SameLine();
                if (ImGui::InputDouble("Z/m", &(*v.refECEF)[2])) changed = true;
                ImGui::SameLine();
                if (ImGui::Button("剪贴板")) {
                    if (const char *c = ImGui::GetClipboardText()) {
                        double val[3] = {};
                        const char *p = c;
                        int cnt = 0;
                        while (*p && cnt < 3) {
                            while (*p && !(*p == '-' || (*p >= '0' && *p <= '9'))) p++;
                            if (!*p) break;
                            val[cnt] = strtod(p, const_cast<char **>(&p));
                            cnt++;
                        }
                        if (cnt >= 1) (*v.refECEF)[0] = val[0];
                        if (cnt >= 2) (*v.refECEF)[1] = val[1];
                        if (cnt >= 3) (*v.refECEF)[2] = val[2];
                        changed = (cnt >= 1);
                    }
                }
                if (changed) {
                    if (v.userRefProvided) *v.userRefProvided = true;
                    if (v.onRefChanged) v.onRefChanged();
                }
            }
        }

        // 选中卫星的观测详情表（伪距 / 载波 / Doppler / 强度，按频点分行）
        void renderObsDetailTable(const PosTabView &v) {
            if (v.showObsDetail && v.selectedSatIdx && *v.selectedSatIdx >= 0 &&
                *v.selectedSatIdx < static_cast<int>(v.satRows.size())) {
                const int si = *v.selectedSatIdx;
                ImGui::SeparatorText(("观测详情 (" + v.satRows[si].sat.toString() + ")").c_str());
                if (ImGui::BeginTable(
                    "##od", 5, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                    ImGui::TableSetupColumn("频", ImGuiTableColumnFlags_WidthFixed, 60);
                    ImGui::TableSetupColumn("伪距(m)");
                    ImGui::TableSetupColumn("载波(cyc)");
                    ImGui::TableSetupColumn("Doppler(Hz)");
                    ImGui::TableSetupColumn("强度");
                    ImGui::TableHeadersRow();
                    std::set<std::string> freqs;
                    for (auto &[t, vv]: v.detailObs) if (t.size() > 1) freqs.insert(t.substr(1));
                    if (freqs.empty()) freqs.insert("?");
                    for (auto &f: freqs) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        std::string l = f;
                        if (v.detailSystem == 'G') l = "L" + f;
                        else if (v.detailSystem == 'C') l = "B" + f;
                        ImGui::Text("%s", l.c_str());
                        auto show = [&](const char *p) {
                            if (const auto it = v.detailObs.find(std::string(p) + f); it != v.detailObs.end())
                                ImGui::Text(
                                    "%.3f", it->second);
                            else ImGui::TextDisabled("-");
                        };
                        ImGui::TableSetColumnIndex(1);
                        show("C");
                        ImGui::TableSetColumnIndex(2);
                        show("L");
                        ImGui::TableSetColumnIndex(3);
                        show("D");
                        ImGui::TableSetColumnIndex(4);
                        show("S");
                    }
                    ImGui::EndTable();
                }
            }
        }

        // 右侧面板三表
        void renderPosTables(const PosTabView &v) {
            ImGui::SeparatorText("位置");
            // 表1：坐标表（数据 | X | Y | Z | 长度 | σ）
            if (ImGui::BeginTable("##coord", 6, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("数据");
                ImGui::TableSetupColumn("X/m", ImGuiTableColumnFlags_WidthFixed, 92);
                ImGui::TableSetupColumn("Y/m", ImGuiTableColumnFlags_WidthFixed, 92);
                ImGui::TableSetupColumn("Z/m", ImGuiTableColumnFlags_WidthFixed, 92);
                ImGui::TableSetupColumn("长度/m", ImGuiTableColumnFlags_WidthFixed, 78);
                ImGui::TableSetupColumn("σ/m", ImGuiTableColumnFlags_WidthFixed, 78);
                ImGui::TableHeadersRow();
                auto row = [&](const char *label, const Eigen::Vector3d &p,
                               const char *lenTxt, const char *sigTxt) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", label);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.4f", p[0]);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.4f", p[1]);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.4f", p[2]);
                    ImGui::TableSetColumnIndex(4);
                    if (lenTxt && *lenTxt) ImGui::Text("%s", lenTxt);
                    else ImGui::TextDisabled("-");
                    ImGui::TableSetColumnIndex(5);
                    if (sigTxt && *sigTxt) ImGui::Text("%s", sigTxt);
                    else ImGui::TextDisabled("-");
                };
                char b1[64], b2[64];
                snprintf(b1, sizeof b1, "%.3f", v.sigmaFloatDisp);
                row("坐标-float", v.xyzFloatDisp, "", b1);
                if (v.hasFixed) {
                    snprintf(b1, sizeof b1, "%.3f", v.sigmaFixedDisp);
                    row("坐标-fixed", v.xyzFixedDisp, "", b1);
                }
                if (v.hasBaseline) {
                    snprintf(b1, sizeof b1, "%.4f", v.baseFloatDisp.norm());
                    row("基线-float", v.baseFloatDisp, b1, "");
                    if (v.hasFixed) {
                        snprintf(b1, sizeof b1, "%.4f", v.baseFixedDisp.norm());
                        row("基线-fixed", v.baseFixedDisp, b1, "");
                    }
                }
                if (v.hasVelDisp) {
                    snprintf(b1, sizeof b1, "%.4f", v.velDisp.norm());
                    snprintf(b2, sizeof b2, "%.3f", v.sigmaVDisp);
                    row("速度", v.velDisp, b1, b2);
                }
                ImGui::EndTable();
            }

            // 表2：参考 + ENU/RTN（无表头；仅在有固定算法时；LEO 无参考轨道不显示）
            if (v.showRefTable) {
                ImGui::SeparatorText("参考");
                if (ImGui::BeginTable("##ref", 5, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                    ImGui::TableSetupColumn("数据");
                    if (v.isLeoRtn) {
                        ImGui::TableSetupColumn("R/m", ImGuiTableColumnFlags_WidthFixed, 92);
                        ImGui::TableSetupColumn("T/m", ImGuiTableColumnFlags_WidthFixed, 92);
                        ImGui::TableSetupColumn("N/m", ImGuiTableColumnFlags_WidthFixed, 92);
                    } else {
                        ImGui::TableSetupColumn("X/m", ImGuiTableColumnFlags_WidthFixed, 92);
                        ImGui::TableSetupColumn("Y/m", ImGuiTableColumnFlags_WidthFixed, 92);
                        ImGui::TableSetupColumn("Z/m", ImGuiTableColumnFlags_WidthFixed, 92);
                    }
                    ImGui::TableSetupColumn("RMS/m", ImGuiTableColumnFlags_WidthFixed, 92);
                    ImGui::TableHeadersRow();
                    auto rrow = [&](const char *label, const Eigen::Vector3d &p, const double rms) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%s", label);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%.4f", p[0]);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%.4f", p[1]);
                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%.4f", p[2]);
                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%.4f", rms);
                    };
                    if (v.isLeoRtn) {
                        rrow("参考", v.refDisp, v.rtn3dDisp);
                        rrow("RTN", v.rtnDisp, v.rtn3dDisp);
                    } else {
                        rrow("参考", v.refDisp, v.abs3dDisp);
                        if (v.hasFixed) {
                            rrow("ENU-fixed", v.enuFixedDisp, v.enuFixed3d);
                            rrow("ENU-float", v.enuFloatDisp, v.enuFloat3d);
                        } else {
                            rrow("ENU", v.enuFloatDisp, v.enuFloat3d);
                        }
                    }
                    ImGui::EndTable();
                }
            }

            // 表3：DOP + 卫星数
            {
                ImGui::SeparatorText("其他");
                if (const int nC = v.isRtk ? 5 : 6;
                    ImGui::BeginTable("##dop", nC,
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                    ImGui::TableSetupColumn("PDOP");
                    ImGui::TableSetupColumn("GDOP");
                    ImGui::TableSetupColumn("HDOP");
                    ImGui::TableSetupColumn("VDOP");
                    if (!v.isRtk) ImGui::TableSetupColumn("TDOP");
                    ImGui::TableSetupColumn("卫星数");
                    ImGui::TableHeadersRow();
                    // 行2：数值
                    ImGui::TableNextRow();
                    int ci = 0;
                    auto set = [&](const char *fmt, const double val) {
                        ImGui::TableSetColumnIndex(ci++);
                        ImGui::Text(fmt, val);
                    };
                    set("%.2f", v.pdop);
                    set("%.2f", v.gdop);
                    set("%.2f", v.hdop);
                    set("%.2f", v.vdop);
                    if (!v.isRtk) set("%.2f", v.tdop);
                    ImGui::TableSetColumnIndex(ci++);
                    ImGui::Text("%d/%d", v.numSatsResult, v.numObs);
                    // 行3/行4：解状态、ratio 作为 2 列表头 + 数值行（仅固定/浮点区分模式）
                    if (v.showSolveStatus) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("解状态");
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("ratio值");
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextColored(v.solveStatusColor, "%s", v.solStatShort.c_str());
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%.2f", v.solRatio);
                    }
                    ImGui::EndTable();
                }
            }
        }

        // 左右并排：卫星概览表（含选中星观测详情） | 位置面板（含导出 CSV）
        void renderSatelliteAndPositionPanels(const PosTabView &v, const float th) {
            {
                bool *stTg = v.renderToggles ? &v.renderToggles->satTable : nullptr;
                bool *ppTg = v.renderToggles ? &v.renderToggles->posPanel : nullptr;
                const bool stVis = vis(stTg, v.showSatTable);
                const bool ppVis = vis(ppTg, v.showPosPanel); //NOLINT
                if (stVis || ppVis || v.renderToggles) {
                    if (ImGui::BeginTable("##ms", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        ImGui::TableSetupColumn("L", ImGuiTableColumnFlags_WidthStretch, 0.6f);
                        ImGui::TableSetupColumn("R", ImGuiTableColumnFlags_WidthStretch, 0.4f);
                        ImGui::TableNextRow(ImGuiTableRowFlags_None, th);

                        ImGui::TableSetColumnIndex(0);
                        ImGui::BeginChild("##sat");
                        {
                            SectionHeader("卫星概览", stTg);
                            if (stVis) {
                                if (v.hasSatRows)
                                    renderSatelliteTable(v.satRows, *v.selectedSatIdx);

                                renderObsDetailTable(v);
                            }
                        }
                        ImGui::EndChild();

                        ImGui::TableSetColumnIndex(1);
                        ImGui::BeginChild("##res");
                        {
                            SectionHeader("位置 (WGS84)", ppTg);
                            if (ppVis) {
                                if (v.solved) {
                                    ImGui::BeginChild("##restbl", ImVec2(0, -52), ImGuiChildFlags_Borders);
                                    renderPosTables(v);
                                    ImGui::EndChild();
                                    if (v.onExportCsv) {
                                        const bool canExport = v.epochCount > 0;
                                        if (!canExport) ImGui::BeginDisabled();
                                        if (ImGui::Button("导出 CSV", ImVec2(-FLT_MIN, 40))) {
                                            v.onExportCsv();
                                        }
                                        if (!canExport) ImGui::EndDisabled();
                                    }
                                } else {
                                    ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1),
                                                       "%s", v.noSolveMsg.empty() ? "无定位解" : v.noSolveMsg.c_str());
                                }
                            } else {
                                ImGui::TextDisabled("(已隐藏)");
                            }
                        }
                        ImGui::EndChild();
                        ImGui::EndTable();
                    }
                }
            }
        }


        // E-N 平面图：尺寸完全由所在单元格决定（可拉伸），中心标记可拖动以搬移参考点。
        // fixedH > 0 用于竖向堆叠场景（此时没有表格行高可借用）；0 = 用尽父级剩余空间。
        void renderENPlanView(const PosTabView &v, const float fixedH = 0.0f) {
            const auto &enu_e = v.enu_e;
            const auto &enu_n = v.enu_n;
            const auto &enu_solved = v.enu_solved;
            if (enu_e.empty() || enu_n.empty() || enu_solved.empty()) return;
            // ⚠️ 尺寸 (0,0) = 用尽父容器剩余空间：宽随表格列宽、高随行高。
            // 旧实现写死 sizePx×sizePx 正方形，拖宽列只会多出空白区域（右侧"空了"）。
            if (!ImGui::BeginChild("##enplan", ImVec2(0, fixedH), ImGuiChildFlags_FrameStyle)) {
                ImGui::EndChild();
                return;
            }
            if (ImPlot::BeginPlot("E-N 平面图", ImVec2(-1, -1), ImPlotFlags_NoLegend | ImPlotFlags_NoBoxSelect)) {
                // 仅取已解算历元
                std::vector<double> xe, xn;
                const int n = static_cast<int>(enu_e.size());
                xe.reserve(n);
                xn.reserve(n);
                for (int i = 0; i < n; ++i) {
                    if (enu_solved[i] < 0.5) continue;
                    const double e = enu_e[i], nn = enu_n[i];
                    if (!std::isfinite(e) || !std::isfinite(nn)) continue;
                    xe.push_back(e);
                    xn.push_back(nn);
                }
                const int m = static_cast<int>(xe.size());
                // 对称等距坐标范围（1:1 显示比例由数据范围保证，不依赖控件是否正方形）
                double r = 1e-6;
                for (int i = 0; i < m; ++i)
                    r = std::max(r, std::max(std::abs(xe[i]), std::abs(xn[i])));
                r *= 1.15;
                ImPlot::SetupAxes("E (m) ", "N (m) ");
                ImPlot::SetupAxisLimits(ImAxis_X1, -r, r, ImPlotCond_Once);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -r, r, ImPlotCond_Once);
                // 轨迹连线（淡）
                if (m > 1) {
                    ImPlotSpec ls;
                    ls.LineColor = ImVec4(0.5f, 0.7f, 0.9f, 0.35f);
                    ls.LineWeight = 1.0f;
                    ImPlot::PlotLine("轨迹", xe.data(), xn.data(), m, ls);
                }
                // 逐历元描点
                if (m > 0) {
                    ImPlotSpec ps;
                    ps.Marker = ImPlotMarker_Circle;
                    ps.MarkerSize = 2.5f;
                    ps.LineColor = ImVec4(0.2f, 0.9f, 0.4f, 0.9f);
                    ImPlot::PlotScatter("解算轨迹", xe.data(), xn.data(), m, ps);
                }
                // —— 中心点（参考点）拖拽：拖动即把参考坐标搬到松手处 ——
                // ENU 坐标本来就是"相对参考点"的，所以标记平时钉在原点；拖动时暂时离开原点，
                // 松手后把 refECEF 平移 (ΔE,ΔN)，下一帧轨迹整体平移、标记回到原点。
                // 状态跨帧保存（ImPlot 拖拽工具本身就跨帧）：只有一块 E-N 图，用函数内静态量即可。
                static double cx = 0.0, cy = 0.0; // 待定中心（当前 ENU 系，m）
                static bool wasHeld = false;
                bool held = false;
                ImPlot::DragPoint(0, &cx, &cy, ImVec4(0.95f, 0.6f, 0.1f, 1.0f), 7.0f,
                                  ImPlotDragToolFlags_NoFit, nullptr, nullptr, &held);
                // 十字线跟随待定中心，明确"搬到哪"
                ImPlot::DragLineX(99, &cx, ImVec4(0.95f, 0.6f, 0.1f, 0.55f), 1.0f,
                                  ImPlotDragToolFlags_NoInputs);
                ImPlot::DragLineY(99, &cy, ImVec4(0.95f, 0.6f, 0.1f, 0.55f), 1.0f,
                                  ImPlotDragToolFlags_NoInputs);
                if (!held) {
                    if (wasHeld && v.refECEF && (std::abs(cx) > 1e-9 || std::abs(cy) > 1e-9)) {
                        // 水平位移（E,N）→ ECEF 增量：老参考点的 ENU 基做反变换，三个分量都会变。
                        const Eigen::Vector3d oldRef = *v.refECEF;
                        *v.refECEF = ENUToXYZ(Eigen::Vector3d(cx, cy, 0.0), oldRef);
                        // 与手输参考坐标同语义：用户已指定参考真值，解算结束的"均值确认"不得覆盖
                        if (v.userRefProvided) *v.userRefProvided = true;
                        if (v.onRefChanged) v.onRefChanged(); // 重算 ENU 序列
                    }
                    cx = 0.0; // 未拖动 / 已提交：标记回到原点
                    cy = 0.0;
                }
                wasHeld = held;
                ImPlot::EndPlot();
            }
            ImGui::EndChild();
        }

        void renderEnuConvergence(const PosTabView &v) {
            if (v.epochCount > 0 && !v.times.empty() && !v.show3d &&
                !v.enu_e.empty() && !v.enu_n.empty() && !v.enu_u.empty()) {
                SectionHeader("ENU 收敛轨迹 / E-N 平面图", nullptr);
                // 行高取本区域剩余高度（视口内自适应，窗口拉高图就跟着长），不再写死 380。
                const ImGuiStyle &style = ImGui::GetStyle();
                float rowH = ImGui::GetContentRegionAvail().y - style.ItemSpacing.y;
                rowH = std::max(280.0f, std::min(rowH, 720.0f));
                bool newed = v.newed;
                if (!v.showEnPlan) {
                    renderENUPlot(v.times, v.enu_e, v.enu_n, v.enu_u,
                                  *v.selectedEpoch, newed, v.fixedY,
                                  v.hasFloatSeries, v.float_e, v.float_n, v.float_u, v.epochCount, rowH);
                    return;
                }
                const float availX = ImGui::GetContentRegionAvail().x;
                // 左：ENU 时间序列；右：E-N 平面图。两列都是 Stretch + Resizable，
                // 分隔条可以自由拖拽（旧实现右列 WidthFixed + 内部正方形，拉宽只会留空白）。
                const float planRatio = std::clamp(380.0f / std::max(availX, 1.0f), 0.22f, 0.6f);
                if (availX >= 640.0f) {
                    if (ImGui::BeginTable("##enuRow", 2,
                                          ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        ImGui::TableSetupColumn("series", ImGuiTableColumnFlags_WidthStretch, 1.0f - planRatio);
                        ImGui::TableSetupColumn("plan", ImGuiTableColumnFlags_WidthStretch, planRatio);
                        ImGui::TableNextRow(ImGuiTableRowFlags_None, rowH);
                        ImGui::TableSetColumnIndex(0);
                        renderENUPlot(v.times, v.enu_e, v.enu_n, v.enu_u,
                                      *v.selectedEpoch, newed, v.fixedY,
                                      v.hasFloatSeries, v.float_e, v.float_n, v.float_u, v.epochCount, rowH);
                        ImGui::TableSetColumnIndex(1);
                        renderENPlanView(v);
                        ImGui::EndTable();
                    }
                } else {
                    // 宽度不足时竖向堆叠：序列在上，平面图在下（给固定高度，避免抢走整页）
                    renderENUPlot(v.times, v.enu_e, v.enu_n, v.enu_u,
                                  *v.selectedEpoch, newed, v.fixedY,
                                  v.hasFloatSeries, v.float_e, v.float_n, v.float_u, v.epochCount,
                                  std::max(220.0f, rowH * 0.55f));
                    renderENPlanView(v, std::min(rowH, 380.0f));
                }
            }
        }

        // Ratio 时序（RTK/LEO/PPP）
        void renderRatioPlotSection(const PosTabView &v) {
            if (!v.showRatio || v.epochCount == 0 || v.times.empty() || v.ratioVals.empty()) return;
            SectionHeader("Ratio 时序 (阈值 3.0)", nullptr);
            bool newed = v.newed;
            renderRatioPlot(v.times, v.ratioVals, *v.selectedEpoch, newed, v.epochCount);
        }

        // 天顶图 / LEO 3D 轨迹 与 逐星残差 并排
        void renderSkyResidualRow(const PosTabView &v) {
            // 3D 卫星坐标图 / 天顶图 + 逐星残差（并排；标题右侧带开关）
            {
                bool *d3Tg = v.renderToggles ? &v.renderToggles->plot3d : nullptr;
                bool *resTg = v.renderToggles ? &v.renderToggles->residual : nullptr;
                // LEO: plot3d 开关控制左列；SPP/PPP 无开关，天顶图恒显示
                const bool left = vis(d3Tg, true);
                // ⚠️ 判「有没有残差可画」必须与码/相位模式无关：切到相位时伪距那两张表是空的
                // （快照只深拷当前显示的那一份），曾用 !v.resT.empty() 判断 → 相位模式下
                // 整块被当成「已隐藏」，图直接消失。
                const bool right = vis(resTg, v.showResidual) && v.hasResidualData;
                if (left || right) {
                    const float rowH = std::max(280.0f, std::min(ImGui::GetContentRegionAvail().x * 0.30f, 480.0f));
                    // LEO 模式恒开两列（保证隐藏图的开关仍可点）；SPP/PPP 按实际列数
                    if (ImGui::BeginTable("##skyres", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        ImGui::TableSetupColumn("Sky", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                        ImGui::TableSetupColumn("Res", ImGuiTableColumnFlags_WidthStretch, 0.67f);
                        ImGui::TableNextRow();
                        int ci = 0;
                        // 左列
                        ImGui::TableSetColumnIndex(ci++);
                        if (ImGui::BeginChild("##leftpanel", ImVec2(0, rowH), ImGuiChildFlags_FrameStyle)) {
                            SectionHeader(v.renderToggles ? "LEO 轨迹 (ECEF XYZ)" : "天顶图", d3Tg);
                            if (left) {
                                // 高度取容器剩余空间：写死 rowH 会连标题行一起算进总高 → 溢出出滑条
                                float fillH = ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y;
                                if (fillH < 120.0f) fillH = 120.0f;
                                if (v.show3d)
                                    RenderLeo3D(v.leoPos3d, v.leoTraj, v.leoRefTraj, fillH);
                                else
                                    RenderSkyplot(v.skyTracks, v.curSkyPts, 460.0f); // SPP/PPP 走天顶图
                            } else {
                                ImGui::TextDisabled("(已隐藏)");
                            }
                        }
                        ImGui::EndChild();
                        // 右列（有残差或 LEO 模式时渲染，保证开关可点）
                        if (right || v.renderToggles) {
                            ImGui::TableSetColumnIndex(ci++); //NOLINT
                            if (ImGui::BeginChild("##respanel", ImVec2(0, rowH), ImGuiChildFlags_FrameStyle)) {
                                SectionHeader(v.resMode && *v.resMode == 1 ? "相位残差序列" : "伪距残差序列", resTg);
                                if (right) {
                                    const double ylo = v.robustRes ? v.resYlo : -8.0;
                                    const double yhi = v.robustRes ? v.resYhi : 8.0;
                                    const double plo = v.robustResPhase ? v.resPhaseYlo : -0.5;
                                    const double phi = v.robustResPhase ? v.resPhaseYhi : 0.5;
                                    bool rnewed = v.newed;
                                    renderSatResidualPlot(v.resT, v.resV,
                                                          v.resPhaseT, v.resPhaseV, v.resMode,
                                                          *v.selectedEpoch, v.epochCount, rnewed,
                                                          &ylo, &yhi, &plo, &phi, 0.0f); // 0 = 填满容器
                                } else {
                                    ImGui::TextDisabled("(已隐藏)");
                                }
                            }
                            ImGui::EndChild();
                        }
                        ImGui::EndTable();
                    }
                }
            }
        }

        // DOP 五曲线叠图（P·G·H·V·T 共享时间轴），未解算历元以 NaN 断点，中途断点不连线。
        void renderDopPlot(const PosTabView &v, const bool newed) {
            if (ImPlot::BeginPlot("DOP", ImVec2(-1, 350))) {
                ImPlot::SetupAxes("Epoch", "DOP");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                std::vector<double> tt;
                std::vector<int> kept;
                buildBrokenAxis(v.times, tt, kept); // 自适应断点：相邻间隔过大插 NaN
                const int n = static_cast<int>(tt.size());
                if (n > 0 && newed && std::isfinite(tt.back()))
                    ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Always);
                auto plotOne = [&](const char *label, const std::vector<double> &src) {
                    if (static_cast<int>(src.size()) != static_cast<int>(v.times.size())) return;
                    std::vector<double> yy;
                    alignBrokenY(kept, src, yy);
                    ImPlot::PlotLine(label, tt.data(), yy.data(), n);
                };
                plotOne("PDOP", v.pdops);
                plotOne("GDOP", v.gdops);
                plotOne("HDOP", v.hdops);
                plotOne("VDOP", v.vdops);
                if (!v.isRtk)
                    plotOne("TDOP", v.tdops);
                if (n > 0) {
                    double sx = v.selectedEpoch ? static_cast<double>(*v.selectedEpoch) : 0.0;
                    ImPlot::DragLineX(3, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
                    if (v.selectedEpoch) {
                        int se = static_cast<int>(std::lround(sx));
                        if (se < 0) se = 0;
                        if (v.epochCount > 0 && se >= v.epochCount) se = v.epochCount - 1;
                        *v.selectedEpoch = se;
                    }
                }
                ImPlot::EndPlot();
            }
        }

        // σP / σV / DOP 时间序列（动态列）
        void renderSigmaDopRow(const PosTabView &v) {
            // σP / σV / DOP（最右列：P·G·H·V·T 五曲线叠图）
            if (v.showSigmaDop && v.epochCount > 0 && !v.times.empty()) {
                SectionHeader("σ / 精度 (P·V·DOP)", nullptr);
                const bool newed = v.newed; // ENU 之后已置 false（与 SPP 一致：X 轴自动缩放）
                // 动态列：只包含启用且非空的序列
                struct Col {
                    const char *name;
                    const std::vector<double> *data; // 单序列列使用
                    bool isDop; // DOP 叠图列
                };
                std::vector<Col> cols;
                if (!v.sigmaPs.empty()) cols.push_back({"SigmaP", &v.sigmaPs, false});
                if (v.showSigmaV && !v.sigmaVs.empty()) cols.push_back({"SigmaV", &v.sigmaVs, false});
                const bool hasDop = v.showPdop && !v.pdops.empty() && !v.gdops.empty()
                                    && !v.hdops.empty() && !v.vdops.empty() && !v.tdops.empty();
                if (hasDop) cols.push_back({"DOP", nullptr, true});
                if (!cols.empty()) {
                    if (ImGui::BeginTable("##bp3", static_cast<int>(cols.size()),
                                          ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        for (int c = 0; c < static_cast<int>(cols.size()); ++c)
                            ImGui::TableSetupColumn(cols[c].name, ImGuiTableColumnFlags_WidthStretch,
                                                    1.0f / static_cast<float>(cols.size()));
                        ImGui::TableNextRow();
                        for (int k = 0; k < static_cast<int>(cols.size()); ++k) {
                            ImGui::TableSetColumnIndex(k);
                            if (cols[k].isDop) {
                                renderDopPlot(v, newed);
                                continue;
                            }
                            const bool sigPWithFloat = cols[k].name == std::string("SigmaP")
                                                       && v.hasFloatSeries && !v.sigmaPFloatVals.empty();
                            if (ImPlot::BeginPlot(cols[k].name, ImVec2(-1, 350),
                                                  sigPWithFloat ? ImPlotFlags_None : ImPlotFlags_NoLegend)) {
                                ImPlot::SetupAxes("Epoch", "m");
                                ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
                                std::vector<double> tt;
                                std::vector<int> kept;
                                buildBrokenAxis(v.times, tt, kept); // 自适应断点：中途断点不连线
                                const int nSigma = static_cast<int>(tt.size());
                                std::vector<double> yy;
                                alignBrokenY(kept, *cols[k].data, yy);
                                if (nSigma > 0 && newed && std::isfinite(tt.back()))
                                    ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Always);
                                if (nSigma > 0) {
                                    ImPlot::PlotLine(cols[k].name, tt.data(), yy.data(), nSigma);
                                    if (sigPWithFloat) {
                                        std::vector<double> ff;
                                        ff.reserve(nSigma);
                                        alignBrokenY(kept, v.sigmaPFloatVals, ff);
                                        ImPlotSpec fspec;
                                        fspec.LineColor = ImVec4(0.75f, 0.75f, 0.75f, 0.35f);
                                        fspec.LineWeight = 1.0f;
                                        ImPlot::PlotLine("SigmaP(浮点)", tt.data(), ff.data(), nSigma, fspec);
                                    }
                                }
                                if (nSigma > 0) {
                                    double sx = v.selectedEpoch ? static_cast<double>(*v.selectedEpoch) : 0.0;
                                    ImPlot::DragLineX(3, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
                                    if (v.selectedEpoch) {
                                        int se = static_cast<int>(std::lround(sx));
                                        if (se < 0) se = 0;
                                        if (v.epochCount > 0 && se >= v.epochCount) se = v.epochCount - 1;
                                        *v.selectedEpoch = se;
                                    }
                                }
                                ImPlot::EndPlot();
                            }
                        }
                        ImGui::EndTable();
                    }
                }
            }
        }

        // LEO 专用：相对参考轨道的 R/T/N/3D 偏差轨迹
        void renderRtnDeviation(const PosTabView &v) {
            // LEO 专用：相对参考轨道的 R/T/N/3D 偏差轨迹
            {
                bool *rtnTg = v.renderToggles && v.hasRef ? &v.renderToggles->rtn : nullptr;
                bool rtnVis = v.renderToggles ? (v.hasRef && v.renderToggles->rtn) : v.showRtn; //NOLINT
                if (rtnTg || (rtnVis && !v.rtnTimes.empty())) {
                    SectionHeader("R/T/N/3D 偏差", rtnTg);
                    if (rtnVis && !v.rtnTimes.empty()) {
                        if (ImPlot::BeginPlot("R/T/N/3D 偏差", ImVec2(-1, 380))) {
                            ImPlot::SetupAxes("Epoch", "偏差 (m)");
                            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                            // 断点感知：相邻有限点间隔过大时插入 NaN 断点（自适应阈值，兼容抽样后的小数秒时间轴）
                            std::vector<double> rt, rr, rrt, rrn, rd3;
                            std::vector<int> kept;
                            buildBrokenAxis(v.rtnTimes, rt, kept);
                            alignBrokenY(kept, v.rtnR, rr);
                            alignBrokenY(kept, v.rtnT, rrt);
                            alignBrokenY(kept, v.rtnN, rrn);
                            alignBrokenY(kept, v.rtnD3, rd3);
                            const int nRtn = static_cast<int>(rt.size());
                            if (nRtn > 0 && v.newed)
                                ImPlot::SetupAxisLimits(ImAxis_X1, 0, rt.back(), ImPlotCond_Once);
                            if (nRtn > 0) {
                                const int dn = static_cast<int>(rt.size());
                                ImPlot::PlotLine("R", rt.data(), rr.data(), dn);
                                ImPlot::PlotLine("T", rt.data(), rrt.data(), dn);
                                ImPlot::PlotLine("N", rt.data(), rrn.data(), dn);
                                ImPlot::PlotLine("3D", rt.data(), rd3.data(), dn);
                            }
                            // 与其他时间序列图一致的可拖拽时间游标（与 selectedEpoch 双向同步）
                            double sx = v.selectedEpoch ? static_cast<double>(*v.selectedEpoch) : 0.0;
                            ImPlot::DragLineX(2, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
                            if (v.selectedEpoch) {
                                int se = static_cast<int>(std::lround(sx));
                                if (se < 0) se = 0;
                                if (v.epochCount > 0 && se >= v.epochCount) se = v.epochCount - 1;
                                *v.selectedEpoch = se;
                            }
                            ImPlot::EndPlot();
                        }
                    } else if (rtnTg) {
                        ImGui::TextDisabled("(已隐藏)");
                    }
                }
            }
        }

        // LEO 专用：逐历元伪距 / 相位 RMS
        void renderRmsRow(const PosTabView &v) {
            // LEO 专用：逐历元伪距/相位 RMS
            {
                bool *rmsTg = v.renderToggles ? &v.renderToggles->rms : nullptr;
                bool rmsVis = v.renderToggles ? v.renderToggles->rms : v.showRms; //NOLINT
                if (rmsTg || (rmsVis && !v.times.empty() && !v.rmsCode.empty())) {
                    SectionHeader("伪距/相位 RMS", rmsTg);
                    if (rmsVis && !v.times.empty() && !v.rmsCode.empty()) {
                        if (ImPlot::BeginPlot("伪距/相位 RMS", ImVec2(-1, 320))) {
                            ImPlot::SetupAxes("Epoch", "RMS (m)");
                            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                            std::vector<double> tt, rc, rp;
                            std::vector<int> kept;
                            buildBrokenAxis(v.times, tt, kept);
                            const bool okRms = !v.rmsCode.empty() && !v.rmsPhase.empty() &&
                                               static_cast<int>(v.rmsCode.size()) == static_cast<int>(v.times.size()) &&
                                               static_cast<int>(v.rmsPhase.size()) == static_cast<int>(v.times.size());
                            if (okRms) {
                                alignBrokenY(kept, v.rmsCode, rc);
                                alignBrokenY(kept, v.rmsPhase, rp);
                            }
                            const int nRms = static_cast<int>(tt.size());
                            if (nRms > 0 && std::isfinite(tt.back()))
                                ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Once);
                            if (okRms) {
                                const int dn = static_cast<int>(tt.size());
                                ImPlot::PlotLine("Code RMS", tt.data(), rc.data(), dn);
                                ImPlot::PlotLine("Phase RMS", tt.data(), rp.data(), dn);
                            }
                            // 与其他时间序列图一致的可拖拽时间游标（与 selectedEpoch 双向同步）
                            if (nRms > 0) {
                                double sx = v.selectedEpoch ? static_cast<double>(*v.selectedEpoch) : 0.0;
                                ImPlot::DragLineX(4, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
                                if (v.selectedEpoch) {
                                    int se = static_cast<int>(std::lround(sx));
                                    if (se < 0) se = 0;
                                    if (v.epochCount > 0 && se >= v.epochCount) se = v.epochCount - 1;
                                    *v.selectedEpoch = se;
                                }
                            }
                            ImPlot::EndPlot();
                        }
                    } else if (rmsTg) {
                        ImGui::TextDisabled("(已隐藏)");
                    }
                }
            }
        }
    } // namespace (anonymous)

    void RenderPositioningTab(const PosTabView &v) {
        renderStatusAndEpochNav(v);
        renderRefECEFInput(v);

        ImGui::Separator();

        if (v.epochCount == 0) {
            if (!v.busy)
                ImGui::TextDisabled("无解算历元（未解出任何历元，请检查观测文件与产品是否匹配）。");
            return;
        }

        const float availY = ImGui::GetContentRegionAvail().y;
        constexpr float plotH = 380;
        float th = availY - plotH - ImGui::GetStyle().ItemSpacing.y;
        if (th < 200) th = 200;

        renderSatelliteAndPositionPanels(v, th);
        renderEnuConvergence(v);
        renderRatioPlotSection(v);
        renderSkyResidualRow(v);
        renderSigmaDopRow(v);
        renderRtnDeviation(v);
        renderRmsRow(v);
    }
}
