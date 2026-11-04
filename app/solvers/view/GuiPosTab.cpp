#include "GuiPosTab.h"

#include "CoordConvert.h"
#include "StringUtils.h"
#include <algorithm>
#include <cstdio>
#include <cfloat>
#include <optional>
#include <set>

#include "implot.h"
#include "ui/GuiColors.h"

namespace Solvers::Charts {
    namespace {

        void sectionHeader(const char *title, bool *flag) {
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

        bool vis(const bool *flag, const bool fallback) {
            return flag ? *flag : fallback;
        }


        void renderStatusAndEpochNav(const PosTabView &v) {

            if (!v.statusText.empty())
                ImGui::TextColored(v.statusColor, "%s", v.statusText.c_str());
            else if (v.epochCount > 0)
                ImGui::TextColored(UI::kColPass, "处理完成: 共 %d 个历元", v.epochCount);


            if (v.selectedEpoch && v.epochCount > 0) {
                int &se = *v.selectedEpoch;
                if (se < 0) se = 0;
                if (se >= v.epochCount) se = v.epochCount - 1;
            }


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

        // ENU 序列中已解算且有限点的均值；无有效点时返回空。
        std::optional<Eigen::Vector3d> meanSolvedENU(const PosTabView &v) {
            double me = 0.0, mn = 0.0, mu = 0.0;
            int m = 0;
            const int n = static_cast<int>(v.enu_e.size());
            for (int i = 0; i < n; ++i) {
                if (v.enu_solved[i] < 0.5) continue;
                if (!std::isfinite(v.enu_e[i]) || !std::isfinite(v.enu_n[i]) || !std::isfinite(v.enu_u[i])) continue;
                me += v.enu_e[i];
                mn += v.enu_n[i];
                mu += v.enu_u[i];
                ++m;
            }
            if (m == 0) return std::nullopt;
            return Eigen::Vector3d(me / m, mn / m, mu / m);
        }

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
                        const int cnt = Str::scanDoubles(c, val, 3);
                        if (cnt >= 1) (*v.refECEF)[0] = val[0];
                        if (cnt >= 2) (*v.refECEF)[1] = val[1];
                        if (cnt >= 3) (*v.refECEF)[2] = val[2];
                        changed = cnt >= 1;
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("取均值")) {
                    if (const auto mean = meanSolvedENU(v)) {
                        *v.refECEF = Coord::ENUToXYZ(*mean, *v.refECEF);
                        if (v.onRefChanged) v.onRefChanged();
                    }
                }
                if (changed) {
                    if (v.onRefChanged) v.onRefChanged();
                }
            }
        }


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


        void renderPosTables(const PosTabView &v) {
            ImGui::SeparatorText("位置");

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
                std::snprintf(b1, sizeof b1, "%.3f", v.sigmaFloatDisp);
                row("坐标-float", v.xyzFloatDisp, "", b1);
                if (v.hasFixed) {
                    std::snprintf(b1, sizeof b1, "%.3f", v.sigmaFixedDisp);
                    row("坐标-fixed", v.xyzFixedDisp, "", b1);
                }
                if (v.hasBaseline) {
                    std::snprintf(b1, sizeof b1, "%.4f", v.baseFloatDisp.norm());
                    row("基线-float", v.baseFloatDisp, b1, "");
                    if (v.hasFixed) {
                        std::snprintf(b1, sizeof b1, "%.4f", v.baseFixedDisp.norm());
                        row("基线-fixed", v.baseFixedDisp, b1, "");
                    }
                }
                if (v.hasVelDisp) {
                    std::snprintf(b1, sizeof b1, "%.4f", v.velDisp.norm());
                    std::snprintf(b2, sizeof b2, "%.3f", v.sigmaVDisp);
                    row("速度", v.velDisp, b1, b2);
                }
                ImGui::EndTable();
            }


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


        void renderSatelliteAndPositionPanels(const PosTabView &v, const float th) {
            {
                bool *stTg = v.renderToggles ? &v.renderToggles->satTable : nullptr;
                bool *ppTg = v.renderToggles ? &v.renderToggles->posPanel : nullptr;
                const bool stVis = vis(stTg, v.showSatTable);
                const bool ppVis = vis(ppTg, v.showPosPanel);
                if (stVis || ppVis || v.renderToggles) {
                    if (ImGui::BeginTable("##ms", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        ImGui::TableSetupColumn("L", ImGuiTableColumnFlags_WidthStretch, 0.6f);
                        ImGui::TableSetupColumn("R", ImGuiTableColumnFlags_WidthStretch, 0.4f);
                        ImGui::TableNextRow(ImGuiTableRowFlags_None, th);

                        ImGui::TableSetColumnIndex(0);
                        ImGui::BeginChild("##sat");
                        {
                            sectionHeader("卫星概览", stTg);
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
                            sectionHeader("位置 (WGS84)", ppTg);
                            if (ppVis) {
                                if (v.solved) {
                                    ImGui::BeginChild("##restbl", ImVec2(0, -52), ImGuiChildFlags_Borders);
                                    renderPosTables(v);
                                    ImGui::EndChild();
                                    if (v.onExportCSV) {
                                        const bool canExport = v.epochCount > 0;
                                        if (!canExport) ImGui::BeginDisabled();
                                        if (ImGui::Button("导出 CSV", ImVec2(-FLT_MIN, 40))) {
                                            v.onExportCSV();
                                        }
                                        if (!canExport) ImGui::EndDisabled();
                                    }
                                } else {
                                    ImGui::TextColored(UI::kColFail,
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


        void renderENPlanView(const PosTabView &v, const float fixedH = 0.0f) {
            const auto &enu_e = v.enu_e;
            const auto &enu_n = v.enu_n;
            const auto &enu_solved = v.enu_solved;
            if (enu_e.empty() || enu_n.empty() || enu_solved.empty()) return;
            const ImGuiStyle &style = ImGui::GetStyle();
            const ImVec2 enAvail = ImGui::GetContentRegionAvail();
            float enW = std::max(160.0f, enAvail.x);
            float enH = fixedH > 0.0f ? fixedH : std::max(220.0f, enAvail.y - style.ItemSpacing.y);
            enH = std::min(enH, 900.0f);
            if (!ImGui::BeginChild("##enplan", ImVec2(enW, enH), ImGuiChildFlags_FrameStyle)) {
                ImGui::EndChild();
                return;
            }
            if (ImPlot::BeginPlot("E-N 平面图", ImVec2(-1, -1), ImPlotFlags_NoLegend | ImPlotFlags_NoBoxSelect)) {

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

                double r = 1e-6;
                for (int i = 0; i < m; ++i)
                    r = std::max(r, std::max(std::abs(xe[i]), std::abs(xn[i])));
                r *= 1.15;
                ImPlot::SetupAxes("E (m) ", "N (m) ");
                ImPlot::SetupAxisLimits(ImAxis_X1, -r, r, ImPlotCond_Once);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -r, r, ImPlotCond_Once);

                if (m > 1) {
                    ImPlotSpec ls;
                    ls.LineColor = ImVec4(0.5f, 0.7f, 0.9f, 0.35f);
                    ls.LineWeight = 1.0f;
                    ImPlot::PlotLine("轨迹", xe.data(), xn.data(), m, ls);
                }

                if (m > 0) {
                    ImPlotSpec ps;
                    ps.Marker = ImPlotMarker_Circle;
                    ps.MarkerSize = 2.5f;
                    ps.LineColor = ImVec4(0.2f, 0.9f, 0.4f, 0.9f);
                    ImPlot::PlotScatter("解算轨迹", xe.data(), xn.data(), m, ps);
                }

                static double cx = 0.0, cy = 0.0;
                static bool wasHeld = false;
                bool held = false;
                ImPlot::DragPoint(0, &cx, &cy, ImVec4(0.95f, 0.6f, 0.1f, 1.0f), 7.0f,
                                  ImPlotDragToolFlags_NoFit, nullptr, nullptr, &held);

                ImPlot::DragLineX(99, &cx, ImVec4(0.95f, 0.6f, 0.1f, 0.55f), 1.0f,
                                  ImPlotDragToolFlags_NoInputs);
                ImPlot::DragLineY(99, &cy, ImVec4(0.95f, 0.6f, 0.1f, 0.55f), 1.0f,
                                  ImPlotDragToolFlags_NoInputs);
                if (!held) {
                    if (wasHeld && v.refECEF && (std::abs(cx) > 1e-9 || std::abs(cy) > 1e-9)) {

                        const Eigen::Vector3d oldRef = *v.refECEF;
                        *v.refECEF = Coord::ENUToXYZ(Eigen::Vector3d(cx, cy, 0.0), oldRef);
                        if (v.onRefChanged) v.onRefChanged();
                    }
                    cx = 0.0;
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
                sectionHeader("ENU 收敛轨迹 / E-N 平面图", nullptr);

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

                        renderENPlanView(v, rowH);
                        ImGui::EndTable();
                    }
                } else {

                    renderENUPlot(v.times, v.enu_e, v.enu_n, v.enu_u,
                                  *v.selectedEpoch, newed, v.fixedY,
                                  v.hasFloatSeries, v.float_e, v.float_n, v.float_u, v.epochCount,
                                  std::max(220.0f, rowH * 0.55f));
                    renderENPlanView(v, std::min(rowH, 380.0f));
                }
            }
        }


        void renderRatioPlotSection(const PosTabView &v) {
            if (!v.showRatio || v.epochCount == 0 || v.times.empty() || v.ratioVals.empty()) return;
            sectionHeader("Ratio 时序 (阈值 3.0)", nullptr);
            bool newed = v.newed;
            renderRatioPlot(v.times, v.ratioVals, *v.selectedEpoch, newed, v.epochCount);
        }


        void renderSkyResidualRow(const PosTabView &v) {

            {
                bool *d3Tg = v.renderToggles ? &v.renderToggles->plot3d : nullptr;
                bool *resTg = v.renderToggles ? &v.renderToggles->residual : nullptr;
                const bool left = vis(d3Tg, true);
                if (const bool right = vis(resTg, v.showResidual) && v.hasResidualData; left || right) {
                    const float rowH = std::max(280.0f, std::min(ImGui::GetContentRegionAvail().x * 0.30f, 480.0f));
                    if (ImGui::BeginTable("##skyres", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        ImGui::TableSetupColumn("Sky", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                        ImGui::TableSetupColumn("Res", ImGuiTableColumnFlags_WidthStretch, 0.67f);
                        ImGui::TableNextRow();
                        int ci = 0;

                        ImGui::TableSetColumnIndex(ci++);
                        if (ImGui::BeginChild("##leftpanel", ImVec2(0, rowH), ImGuiChildFlags_FrameStyle)) {
                            sectionHeader(v.renderToggles ? "LEO 轨迹 (ECEF XYZ)" : "天顶图", d3Tg);
                            if (left) {
                                float fillH = ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y;
                                if (fillH < 120.0f) fillH = 120.0f;
                                if (v.show3d)
                                    renderLeo3D(v.leoPos3d, v.leoTraj, v.leoRefTraj, fillH);
                                else
                                    renderSkyplot(v.skyTracks, v.curSkyPts, 460.0f);
                            } else {
                                ImGui::TextDisabled("(已隐藏)");
                            }
                        }
                        ImGui::EndChild();

                        if (right || v.renderToggles) {
                            ImGui::TableSetColumnIndex(ci++); //NOLINT
                            if (ImGui::BeginChild("##respanel", ImVec2(0, rowH), ImGuiChildFlags_FrameStyle)) {
                                sectionHeader(v.resMode && *v.resMode == 1 ? "相位残差序列" : "伪距残差序列", resTg);
                                if (right) {
                                    const double ylo = v.robustRes ? v.resYlo : -8.0;
                                    const double yhi = v.robustRes ? v.resYhi : 8.0;
                                    const double plo = v.robustResPhase ? v.resPhaseYlo : -0.5;
                                    const double phi = v.robustResPhase ? v.resPhaseYhi : 0.5;
                                    bool rnewed = v.newed;
                                    renderSatResidualPlot(v.resT, v.resV,
                                                          v.resPhaseT, v.resPhaseV, v.resMode,
                                                          *v.selectedEpoch, v.epochCount, rnewed,
                                                          &ylo, &yhi, &plo, &phi, 0.0f);
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


        void renderDopPlot(const PosTabView &v, const bool newed) {
            if (ImPlot::BeginPlot("DOP", ImVec2(-1, 350))) {
                ImPlot::SetupAxes("Epoch", "DOP");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                const BrokenSeries br(v.times);
                const int n = static_cast<int>(br.t.size());
                xAxisFollow(br.t, newed);
                auto plotOne = [&](const char *label, const std::vector<double> &src) {
                    if (static_cast<int>(src.size()) != static_cast<int>(v.times.size())) return;
                    const std::vector<double> yy = br.align(src);
                    ImPlot::PlotLine(label, br.t.data(), yy.data(), n);
                };
                plotOne("PDOP", v.pdops);
                plotOne("GDOP", v.gdops);
                plotOne("HDOP", v.hdops);
                plotOne("VDOP", v.vdops);
                if (!v.isRtk)
                    plotOne("TDOP", v.tdops);
                if (n > 0)
                    epochCursor(3, v.selectedEpoch, v.epochCount);
                ImPlot::EndPlot();
            }
        }


        void renderSigmaDopRow(const PosTabView &v) {

            if (v.showSigmaDop && v.epochCount > 0 && !v.times.empty()) {
                sectionHeader("σ / 精度 (P·V·DOP)", nullptr);
                const bool newed = v.newed;

                struct Col {
                    const char *name;
                    const std::vector<double> *data;
                    bool isDop;
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
                                const BrokenSeries br(v.times);
                                const int nSigma = static_cast<int>(br.t.size());
                                const std::vector<double> yy = br.align(*cols[k].data);
                                xAxisFollow(br.t, newed);
                                if (nSigma > 0) {
                                    ImPlot::PlotLine(cols[k].name, br.t.data(), yy.data(), nSigma);
                                    if (sigPWithFloat) {
                                        const std::vector<double> ff = br.align(v.sigmaPFloatVals);
                                        ImPlotSpec fspec;
                                        fspec.LineColor = UI::kColFloat;
                                        fspec.LineWeight = 1.0f;
                                        ImPlot::PlotLine("SigmaP(浮点)", br.t.data(), ff.data(), nSigma, fspec);
                                    }
                                }
                                if (nSigma > 0)
                                    epochCursor(3, v.selectedEpoch, v.epochCount);
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
            {
                bool *rtnTg = v.renderToggles && v.hasRef ? &v.renderToggles->rtn : nullptr;
                bool rtnVis = v.renderToggles ? (v.hasRef && v.renderToggles->rtn) : v.showRtn; //NOLINT
                if (rtnTg || (rtnVis && !v.rtnTimes.empty())) {
                    sectionHeader("R/T/N/3D 偏差", rtnTg);
                    if (rtnVis && !v.rtnTimes.empty()) {
                        if (ImPlot::BeginPlot("R/T/N/3D 偏差", ImVec2(-1, 380))) {
                            ImPlot::SetupAxes("Epoch", "偏差 (m)");
                            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                            const BrokenSeries br(v.rtnTimes);
                            const std::vector<double> rr = br.align(v.rtnR), rrt = br.align(v.rtnT),
                                                      rrn = br.align(v.rtnN), rd3 = br.align(v.rtnD3);
                            const int nRtn = static_cast<int>(br.t.size());
                            xAxisFollow(br.t, v.newed, ImPlotCond_Once);
                            if (nRtn > 0) {
                                ImPlot::PlotLine("R", br.t.data(), rr.data(), nRtn);
                                ImPlot::PlotLine("T", br.t.data(), rrt.data(), nRtn);
                                ImPlot::PlotLine("N", br.t.data(), rrn.data(), nRtn);
                                ImPlot::PlotLine("3D", br.t.data(), rd3.data(), nRtn);
                            }
                            epochCursor(2, v.selectedEpoch, v.epochCount);
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
            {
                bool *rmsTg = v.renderToggles ? &v.renderToggles->rms : nullptr;
                bool rmsVis = v.renderToggles ? v.renderToggles->rms : v.showRms; //NOLINT
                if (rmsTg || (rmsVis && !v.times.empty() && !v.rmsCode.empty())) {
                    sectionHeader("伪距/相位 RMS", rmsTg);
                    if (rmsVis && !v.times.empty() && !v.rmsCode.empty()) {
                        if (ImPlot::BeginPlot("伪距/相位 RMS", ImVec2(-1, 320))) {
                            ImPlot::SetupAxes("Epoch", "RMS (m)");
                            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                            const BrokenSeries br(v.times);
                            const bool okRms = !v.rmsCode.empty() && !v.rmsPhase.empty() &&
                                               static_cast<int>(v.rmsCode.size()) == static_cast<int>(v.times.size()) &&
                                               static_cast<int>(v.rmsPhase.size()) == static_cast<int>(v.times.size());
                            const int nRms = static_cast<int>(br.t.size());
                            xAxisFollow(br.t, true, ImPlotCond_Once);
                            if (okRms) {
                                const std::vector<double> rc = br.align(v.rmsCode), rp = br.align(v.rmsPhase);
                                ImPlot::PlotLine("Code RMS", br.t.data(), rc.data(), nRms);
                                ImPlot::PlotLine("Phase RMS", br.t.data(), rp.data(), nRms);
                            }
                            if (nRms > 0)
                                epochCursor(4, v.selectedEpoch, v.epochCount);
                            ImPlot::EndPlot();
                        }
                    } else if (rmsTg) {
                        ImGui::TextDisabled("(已隐藏)");
                    }
                }
            }
        }
    }

    void renderPositioningTab(const PosTabView &v) {
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
