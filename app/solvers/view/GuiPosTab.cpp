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

        void sectionHeader(const char *title) {
            ImGui::SeparatorText(title);
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
            const int n = static_cast<int>(v.p.enu_e.size());
            for (int i = 0; i < n; ++i) {
                if (v.p.solvedFlags[i] < 0.5) continue;
                if (!std::isfinite(v.p.enu_e[i]) || !std::isfinite(v.p.enu_n[i]) || !std::isfinite(v.p.enu_u[i])) continue;
                me += v.p.enu_e[i];
                mn += v.p.enu_n[i];
                mu += v.p.enu_u[i];
                ++m;
            }
            if (m == 0) return std::nullopt;
            return Eigen::Vector3d(me / m, mn / m, mu / m);
        }

        void renderRefECEFInput(const PosTabView &v) {
            if (v.refECEF && v.showRefEnu) {
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
            if (!ImGui::BeginTable("##ms", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV))
                return;
            ImGui::TableSetupColumn("L", ImGuiTableColumnFlags_WidthStretch, 0.6f);
            ImGui::TableSetupColumn("R", ImGuiTableColumnFlags_WidthStretch, 0.4f);
            ImGui::TableNextRow(ImGuiTableRowFlags_None, th);

            ImGui::TableSetColumnIndex(0);
            if (ImGui::BeginChild("##sat")) {
                sectionHeader("卫星概览");
                if (v.hasSatRows) renderSatelliteTable(v.satRows, *v.selectedSatIdx);
                renderObsDetailTable(v);
            }
            ImGui::EndChild();

            ImGui::TableSetColumnIndex(1);
            if (ImGui::BeginChild("##res")) {
                sectionHeader("位置 (WGS84)");
                if (v.solved) {
                    if (ImGui::BeginChild("##restbl", ImVec2(0, -52), ImGuiChildFlags_Borders)) renderPosTables(v);
                    ImGui::EndChild();
                    if (v.onExportCSV) {
                        const bool canExport = v.epochCount > 0;
                        if (!canExport) ImGui::BeginDisabled();
                        if (ImGui::Button("导出 CSV", ImVec2(-FLT_MIN, 40))) v.onExportCSV();
                        if (!canExport) ImGui::EndDisabled();
                    }
                } else {
                    ImGui::TextColored(UI::kColFail, "%s",
                                       v.noSolveMsg.empty() ? "无定位解" : v.noSolveMsg.c_str());
                }
            }
            ImGui::EndChild();
            ImGui::EndTable();
        }


        void renderENPlanView(const PosTabView &v, const float fixedH = 0.0f) {
            const auto &xe = v.s.planE;
            const auto &xn = v.s.planN;
            if (v.p.enu_e.empty() || v.p.enu_n.empty()) return;
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
                const int m = static_cast<int>(xe.size());
                const double r = v.s.planR;
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

                // 当前历元（时间游标所指）高亮：蓝色大点，叠加在轨迹之上，便于实时/回放时一眼定位。
                if (v.selectedEpoch && *v.selectedEpoch >= 0 &&
                    *v.selectedEpoch < static_cast<int>(v.p.enu_e.size()) &&
                    *v.selectedEpoch < static_cast<int>(v.p.solvedFlags.size()) &&
                    v.p.solvedFlags[*v.selectedEpoch] > 0.5) {
                    const double ce = v.p.enu_e[*v.selectedEpoch];
                    const double cn = *v.selectedEpoch < static_cast<int>(v.p.enu_n.size())
                                         ? v.p.enu_n[*v.selectedEpoch] : 0.0;
                    if (std::isfinite(ce) && std::isfinite(cn)) {
                        ImPlotSpec hs;
                        hs.Marker = ImPlotMarker_Circle;
                        hs.MarkerSize = 7.5f;
                        hs.LineColor = ImVec4(0.15f, 0.45f, 1.0f, 1.0f);
                        ImPlot::PlotScatter("##curEpoch", &ce, &cn, 1, hs);
                    }
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
            if (v.epochCount > 0 && !v.s.axis.kept.empty() && !v.show3d &&
                !v.p.enu_e.empty() && !v.p.enu_n.empty() && !v.p.enu_u.empty()) {
                sectionHeader("ENU 收敛轨迹 / E-N 平面图");

                const ImGuiStyle &style = ImGui::GetStyle();
                float rowH = ImGui::GetContentRegionAvail().y - style.ItemSpacing.y;
                rowH = std::max(280.0f, std::min(rowH, 720.0f));
                bool newed = v.s.newed;
                if (!v.showEnPlan) {
                    renderENUPlot(v.s.axis, v.p.enu_e, v.p.enu_n, v.p.enu_u,
                                  *v.selectedEpoch, newed,
                                  v.hasFloatSeries, v.p.float_e, v.p.float_n, v.p.float_u, v.epochCount, rowH);
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
                        renderENUPlot(v.s.axis, v.p.enu_e, v.p.enu_n, v.p.enu_u,
                                      *v.selectedEpoch, newed,
                                      v.hasFloatSeries, v.p.float_e, v.p.float_n, v.p.float_u, v.epochCount, rowH);
                        ImGui::TableSetColumnIndex(1);

                        renderENPlanView(v, rowH);
                        ImGui::EndTable();
                    }
                } else {

                    renderENUPlot(v.s.axis, v.p.enu_e, v.p.enu_n, v.p.enu_u,
                                  *v.selectedEpoch, newed,
                                  v.hasFloatSeries, v.p.float_e, v.p.float_n, v.p.float_u, v.epochCount,
                                  std::max(220.0f, rowH * 0.55f));
                    renderENPlanView(v, std::min(rowH, 380.0f));
                }
            }
        }


        void renderSkyResidualRow(const PosTabView &v) {
            const float rowH = std::max(280.0f, std::min(ImGui::GetContentRegionAvail().x * 0.30f, 480.0f));
            if (ImGui::BeginTable("##skyres", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                ImGui::TableSetupColumn("Sky", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                ImGui::TableSetupColumn("Res", ImGuiTableColumnFlags_WidthStretch, 0.67f);
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::BeginChild("##leftpanel", ImVec2(0, rowH), ImGuiChildFlags_FrameStyle)) {
                    sectionHeader("天顶图");
                    float fillH = ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y;
                    if (fillH < 120.0f) fillH = 120.0f;
                    if (v.show3d)
                        renderLeo3D(v.leoPos3d, v.leoTraj, v.leoRefTraj, fillH);
                    else
                        renderSkyplot(v.skyTracks, v.curSkyPts, 460.0f);
                }
                ImGui::EndChild();

                if (v.s.hasResidual) {
                    ImGui::TableSetColumnIndex(1);
                    if (ImGui::BeginChild("##respanel", ImVec2(0, rowH), ImGuiChildFlags_FrameStyle)) {
                        sectionHeader(v.resMode && *v.resMode == 1 ? "相位残差序列" : "伪距残差序列");
                        const double ylo = v.s.hasResRange ? v.s.resYlo : -8.0;
                        const double yhi = v.s.hasResRange ? v.s.resYhi : 8.0;
                        const double plo = v.s.hasResPhaseRange ? v.s.resPhaseYlo : -0.5;
                        const double phi = v.s.hasResPhaseRange ? v.s.resPhaseYhi : 0.5;
                        bool rnewed = v.s.newed;
                        renderSatResidualPlot(v.p.satResVals, v.p.satResPhaseVals,
                                              v.s.resAxisCode, v.s.resAxisPhase, v.resMode,
                                              *v.selectedEpoch, v.epochCount, rnewed,
                                              &ylo, &yhi, &plo, &phi, 0.0f);
                    }
                    ImGui::EndChild();
                }
                ImGui::EndTable();
            }
        }


        void renderDopPlot(const PosTabView &v, const bool newed) {
            if (ImPlot::BeginPlot("DOP", ImVec2(-1, 350))) {
                ImPlot::SetupAxes("Epoch", "DOP");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                const BrokenAxis &br = v.s.axis;
                const int n = static_cast<int>(br.t.size());
                xAxisFollow(br.t, newed);
                auto plotOne = [&](const char *label, const std::vector<double> &src) {
                    if (static_cast<int>(src.size()) != static_cast<int>(v.p.times.size())) return;
                    const std::vector<double> yy = br.align(src);
                    ImPlot::PlotLine(label, br.t.data(), yy.data(), n);
                };
                plotOne("PDOP", v.p.pdops);
                plotOne("GDOP", v.p.gdops);
                plotOne("HDOP", v.p.hdops);
                plotOne("VDOP", v.p.vdops);
                if (!v.isRtk)
                    plotOne("TDOP", v.p.tdops);
                if (n > 0)
                    epochCursor(3, v.selectedEpoch, v.epochCount);
                ImPlot::EndPlot();
            }
        }


        void renderSigmaDopRow(const PosTabView &v) {

            if (v.showSigmaDop && v.epochCount > 0 && !v.s.axis.kept.empty()) {
                sectionHeader("σ / 精度 (P·V·DOP)");
                const bool newed = v.s.newed;

                struct Col {
                    const char *name;
                    const std::vector<double> *data;
                    bool isDop;
                };
                std::vector<Col> cols;
                if (!v.p.sigmaPs.empty()) cols.push_back({"SigmaP", &v.p.sigmaPs, false});
                if (v.showSigmaV && !v.p.sigmaVs.empty()) cols.push_back({"SigmaV", &v.p.sigmaVs, false});
                const bool hasDop = !v.p.pdops.empty() && !v.p.gdops.empty()
                                    && !v.p.hdops.empty() && !v.p.vdops.empty() && !v.p.tdops.empty();
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
                                                       && v.hasFloatSeries && !v.p.sigmaPFloatVals.empty();
                            if (ImPlot::BeginPlot(cols[k].name, ImVec2(-1, 350),
                                                  sigPWithFloat ? ImPlotFlags_None : ImPlotFlags_NoLegend)) {
                                ImPlot::SetupAxes("Epoch", "m");
                                ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
                                const BrokenAxis &br = v.s.axis;
                                const int nSigma = static_cast<int>(br.t.size());
                                const std::vector<double> yy = br.align(*cols[k].data);
                                xAxisFollow(br.t, newed);
                                if (nSigma > 0) {
                                    ImPlot::PlotLine(cols[k].name, br.t.data(), yy.data(), nSigma);
                                    if (sigPWithFloat) {
                                        const std::vector<double> ff = br.align(v.p.sigmaPFloatVals);
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
            if (!v.showRtn || v.rtnTimes.empty()) return;
            sectionHeader("R/T/N/3D 偏差");
            if (ImPlot::BeginPlot("R/T/N/3D 偏差", ImVec2(-1, 380))) {
                ImPlot::SetupAxes("Epoch", "偏差 (m)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                BrokenAxis br;
                br.build(v.rtnTimes);
                const std::vector<double> rr = br.align(v.rtnR), rrt = br.align(v.rtnT),
                                          rrn = br.align(v.rtnN), rd3 = br.align(v.rtnD3);
                const int nRtn = static_cast<int>(br.t.size());
                xAxisFollow(br.t, v.s.newed, ImPlotCond_Once);
                if (nRtn > 0) {
                    ImPlot::PlotLine("R", br.t.data(), rr.data(), nRtn);
                    ImPlot::PlotLine("T", br.t.data(), rrt.data(), nRtn);
                    ImPlot::PlotLine("N", br.t.data(), rrn.data(), nRtn);
                    ImPlot::PlotLine("3D", br.t.data(), rd3.data(), nRtn);
                }
                epochCursor(2, v.selectedEpoch, v.epochCount);
                ImPlot::EndPlot();
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
        renderSkyResidualRow(v);
        renderSigmaDopRow(v);
        renderRtnDeviation(v);
    }
}
