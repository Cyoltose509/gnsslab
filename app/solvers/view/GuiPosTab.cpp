#include "GuiPosTab.h"
#include "GuiHelpers.h"

#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include <algorithm>
#include <cstdio>
#include <cfloat>
#include <optional>
#include <set>

#include "implot.h"
#include "ui/DragBar.h"
#include "ui/GuiColors.h"

namespace Solvers::Charts {
    namespace {

        void sectionHeader(const char *title) {
            ImGui::SeparatorText(title);
        }

        void renderFilterDirection(const PosTabView &v) {
            if (v.filterDirectionText.empty()) return;
            ImGui::TextDisabled("%s", v.filterDirectionText.c_str());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("滤波方向在参数页选择；解算结果、图表与 CSV 都用这一档的解。\n后向先扫完整份文件再从末尾倒序滤波，融合再跑一遍前向并按协方差加权。");
        }

        void renderStatusAndEpochNav(const PosTabView &v) {
            if (!v.statusText.empty())
                ImGui::TextColored(v.statusColor, "%s", v.statusText.c_str());
            else if (v.epochCount > 0)
                ImGui::TextColored(UI::kColPass, "处理完成: 共 %d 个历元", v.epochCount);

            if (v.selectedEpoch && v.epochCount > 0) {
                char tail[128] = {};
                if (v.showFixRate)
                    std::snprintf(tail, sizeof tail, "%s", v.fixRateText.c_str());
                Helpers::renderEpochNav(*v.selectedEpoch, v.epochCount,
                                        v.epochInfo.empty() ? nullptr : v.epochInfo.c_str(),
                                        v.showFixRate ? tail : nullptr);
            }
        }

        // ENU 序列中已出解且有限点的均值；无有效点时返回空。
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
                ImGui::PopItemWidth();
                ImGui::SameLine();
                if (const Helpers::PasteResult p = Helpers::clipboardXYZButton("pos_ref_clip");
                    p.count > 0) {
                    for (int i = 0; i < p.count; i++) (*v.refECEF)[i] = p.xyz[i];
                    changed = true;
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
                    ImGui::TableSetupColumn("强度", ImGuiTableColumnFlags_WidthStretch);
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


        /// 「格式」列的口径：坐标在地心直角↔大地坐标之间换，基线换站心 ENU，速度换 NED。
        enum class RowKind { Coord, Baseline, Velocity };

        constexpr const char *kFmtCoord[] = {"XYZ", "BLH"};
        constexpr const char *kFmtBase[] = {"XYZ", "ENU"};
        constexpr const char *kFmtVel[] = {"XYZ", "NED"};

        /// 各行各记一份显示偏好，跨任务共用（换任务不必重选）。
        struct DispFmt {
            int coordFloat = 0;
            int coordFixed = 0;
            int baseFloat = 0;
            int baseFixed = 0;
            int vel = 0;
        };

        DispFmt g_dispFmt;

        /// 一行三个分量的显示文本：按所选口径换算，单位写在格子里（长度 m、角度 °）。
        void componentText(const Eigen::Vector3d &p, const RowKind kind, const int sel,
                           const Eigen::Vector3d &anchor, std::string (&out)[3]) {
            char buf[64];
            if (kind == RowKind::Coord && sel == 1) {
                const BLH blh = Coord::XYZToBLH(p, Frame::WGS84);
                std::snprintf(buf, sizeof buf, "%.8f°", blh.B() * RAD_TO_DEG);
                out[0] = buf;
                std::snprintf(buf, sizeof buf, "%.8f°", blh.L() * RAD_TO_DEG);
                out[1] = buf;
                std::snprintf(buf, sizeof buf, "%.4fm", blh.H());
                out[2] = buf;
                return;
            }
            // 矢量换站心系只转不挪：给锚点加上矢量再求相对锚点的分量，等于只作用旋转矩阵。
            Eigen::Vector3d c = p;
            if (kind == RowKind::Baseline && sel == 1 && anchor.squaredNorm() > 1.0)
                c = Coord::XYZToENU(XYZ(p + anchor), XYZ(anchor));
            else if (kind == RowKind::Velocity && sel == 1 && anchor.squaredNorm() > 1.0e12)
                c = Coord::XYZToNED(XYZ(p + anchor), XYZ(anchor));
            for (int i = 0; i < 3; ++i) {
                std::snprintf(buf, sizeof buf, "%.4fm", c[i]);
                out[i] = buf;
            }
        }

        void renderPosTables(const PosTabView &v) {
            ImGui::SeparatorText("坐标 / 速度");

            // 分量列 Stretch 吸收剩余宽度：表格始终铺满容器（右侧不留白），也不会围出空列；
            // 其余列按内容自适应（换字体/DPI 不裁字）。
            const float pad = ImGui::GetStyle().CellPadding.x * 2.0f;
            const float wFmt = ImGui::CalcTextSize("XYZ").x + ImGui::GetFrameHeight() + pad;
            if (ImGui::BeginTable("##coord", 7, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders
                                                 | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("数据");
                ImGui::TableSetupColumn("分量1", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("分量2", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("分量3", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("长度/m");
                ImGui::TableSetupColumn("σ/m");
                ImGui::TableSetupColumn("格式", ImGuiTableColumnFlags_WidthFixed, wFmt);
                ImGui::TableHeadersRow();
                auto row = [&](const char *label, const Eigen::Vector3d &p, int &sel, const RowKind kind,
                               const char *const *opts, const int nOpts, const Eigen::Vector3d &anchor,
                               const char *lenTxt, const char *sigTxt) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(label);
                    std::string txt[3];
                    componentText(p, kind, sel, anchor, txt);
                    for (int i = 0; i < 3; ++i) {
                        ImGui::TableSetColumnIndex(1 + i);
                        ImGui::TextUnformatted(txt[i].c_str());
                    }
                    ImGui::TableSetColumnIndex(4);
                    if (lenTxt && *lenTxt) ImGui::TextUnformatted(lenTxt);
                    else ImGui::TextDisabled("-");
                    ImGui::TableSetColumnIndex(5);
                    if (sigTxt && *sigTxt) ImGui::TextUnformatted(sigTxt);
                    else ImGui::TextDisabled("-");
                    ImGui::TableSetColumnIndex(6);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::PushID(label);
                    ImGui::Combo("##fmt", &sel, opts, nOpts);
                    ImGui::PopID();
                };
                char b1[64], b2[64];
                // 速度/基线换站心系要在解算位置上算，取浮点解优先、固定解兜底。
                const Eigen::Vector3d anchor =
                        v.xyzFloatDisp.squaredNorm() > 1.0e12 ? v.xyzFloatDisp : v.xyzFixedDisp;
                std::snprintf(b1, sizeof b1, "%.3f", v.sigmaFloatDisp);
                row("坐标-float", v.xyzFloatDisp, g_dispFmt.coordFloat, RowKind::Coord,
                    kFmtCoord, 2, anchor, "", b1);
                if (v.hasFixed) {
                    std::snprintf(b1, sizeof b1, "%.3f", v.sigmaFixedDisp);
                    row("坐标-fixed", v.xyzFixedDisp, g_dispFmt.coordFixed, RowKind::Coord,
                        kFmtCoord, 2, anchor, "", b1);
                }
                if (v.hasBaseline) {
                    std::snprintf(b1, sizeof b1, "%.4f", v.baseFloatDisp.norm());
                    row("基线-float", v.baseFloatDisp, g_dispFmt.baseFloat, RowKind::Baseline,
                        kFmtBase, 2, v.refDisp, b1, "");
                    if (v.hasFixed) {
                        std::snprintf(b1, sizeof b1, "%.4f", v.baseFixedDisp.norm());
                        row("基线-fixed", v.baseFixedDisp, g_dispFmt.baseFixed, RowKind::Baseline,
                            kFmtBase, 2, v.refDisp, b1, "");
                    }
                }
                const bool velValid = v.velDisp.squaredNorm() > 1e-12;
                std::snprintf(b1, sizeof b1, "%.4f", v.velDisp.norm());
                std::snprintf(b2, sizeof b2, "%.3f", v.sigmaVDisp);
                row("速度", v.velDisp, g_dispFmt.vel, RowKind::Velocity, kFmtVel, 2, anchor,
                    velValid ? b1 : "", velValid && v.sigmaVDisp > 0.0 ? b2 : "");
                ImGui::EndTable();
            }


            if (v.showRefTable) {
                ImGui::SeparatorText("参考");
                if (ImGui::BeginTable("##ref", 5, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders
                                                   | ImGuiTableFlags_RowBg)) {
                    ImGui::TableSetupColumn("数据");
                    if (v.isLeoRtn) {
                        ImGui::TableSetupColumn("R/m", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("T/m", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("N/m", ImGuiTableColumnFlags_WidthStretch);
                    } else {
                        // 行里既有 XYZ 也有 ENU，表头只写「分量」，单位落到格子里。
                        ImGui::TableSetupColumn("分量1", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("分量2", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("分量3", ImGuiTableColumnFlags_WidthStretch);
                    }
                    ImGui::TableSetupColumn("RMS/m");
                    ImGui::TableHeadersRow();
                    auto rrow = [&](const char *label, const Eigen::Vector3d &p, const double rms) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(label);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%.4fm", p[0]);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%.4fm", p[1]);
                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%.4fm", p[2]);
                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%.4fm", rms);
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
                    ImGui::BeginTable("##dop", nC, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                    // 值都短，均匀铺满一行即可。
                    ImGui::TableSetupColumn("PDOP", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("GDOP", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("HDOP", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("VDOP", ImGuiTableColumnFlags_WidthStretch);
                    if (!v.isRtk) ImGui::TableSetupColumn("TDOP", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("卫星数", ImGuiTableColumnFlags_WidthStretch);
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
                    ImGui::EndTable();
                }

                // 解状态/ratio 放同一张表里会塞进 PDOP 那格，文字直接溢出到相邻列。
                if (v.showSolveStatus) {
                    const float wKey = ImGui::CalcTextSize("ratio值").x + pad;
                    if (ImGui::BeginTable("##solstat", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders
                                                         | ImGuiTableFlags_RowBg)) {
                        ImGui::TableSetupColumn("项", ImGuiTableColumnFlags_WidthFixed, wKey);
                        ImGui::TableSetupColumn("值", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted("解状态");
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextColored(v.solveStatusColor, "%s", v.solStatShort.c_str());
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted("ratio值");
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%.2f", v.solRatio);
                        ImGui::EndTable();
                    }
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
                if (v.hasSatRows && v.selectedSatIdx) renderSatelliteTable(v.satRows, *v.selectedSatIdx);
                renderObsDetailTable(v);
            }
            ImGui::EndChild();

            ImGui::TableSetColumnIndex(1);
            if (ImGui::BeginChild("##res")) {
                sectionHeader("解算结果");
                constexpr float btnH = 40.0f;
                const float foot = v.onExport ? btnH + ImGui::GetStyle().ItemSpacing.y : 0.0f;
                if (v.solved) {
                    // 表格按内容定宽，右栏一窄就横向滚动，别把最后一列裁掉。
                    if (ImGui::BeginChild("##restbl", ImVec2(0, -foot), ImGuiChildFlags_Borders,
                                          ImGuiWindowFlags_HorizontalScrollbar))
                        renderPosTables(v);
                    ImGui::EndChild();
                } else {
                    ImGui::TextColored(UI::kColFail, "%s",
                                       v.noSolveMsg.empty() ? "无定位解" : v.noSolveMsg.c_str());
                    // 本历元无解也留着导出入口：别的历元可能解出来了，或者要留一份「全排除」的记录。
                    if (v.onExport) {
                        const float gap = ImGui::GetContentRegionAvail().y - foot;
                        if (gap > 0.0f) ImGui::SetCursorPosY(ImGui::GetCursorPosY() + gap);
                    }
                }
                if (v.onExport) {
                    const bool canExport = v.epochCount > 0;
                    if (!canExport) ImGui::BeginDisabled();
                    if (ImGui::Button("导出…", ImVec2(-FLT_MIN, btnH))) v.onExport();
                    if (!canExport) ImGui::EndDisabled();
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
            if (ImPlot::BeginPlot(v.enPlanTrack ? "E-N 轨迹图" : "E-N 平面图", ImVec2(-1, -1),
                                  ImPlotFlags_NoLegend | ImPlotFlags_NoBoxSelect)) {
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
                    const int cur = *v.selectedEpoch;
                    double ce = 0.0, cn = 0.0;
                    bool valid = false;
                    if (v.enPlanTrack) {
                        const auto &xyz = v.p.ecef;
                        if (cur < static_cast<int>(xyz.size()) && xyz[cur].squaredNorm() > 1e12) {
                            const ENU enu = Coord::XYZToENU(xyz[cur], v.s.planOrigin);
                            ce = enu.E();
                            cn = enu.N();
                            valid = true;
                        }
                    } else {
                        ce = v.p.enu_e[cur];
                        cn = cur < static_cast<int>(v.p.enu_n.size()) ? v.p.enu_n[cur] : 0.0;
                        valid = true;
                    }
                    if (valid && std::isfinite(ce) && std::isfinite(cn)) {
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
                    if (wasHeld && (std::abs(cx) > 1e-9 || std::abs(cy) > 1e-9)) {
                        if (v.enPlanTrack) {
                            // 轨迹图拖中心只挪显示原点，ENU 误差、RTN 与 CSV 一律不动。
                            if (v.onPlanOriginChanged)
                                v.onPlanOriginChanged(Eigen::Vector3d(cx, cy, 0.0));
                        } else if (v.refECEF) {
                            const Eigen::Vector3d oldRef = *v.refECEF;
                            *v.refECEF = Coord::ENUToXYZ(Eigen::Vector3d(cx, cy, 0.0), oldRef);
                            if (v.onRefChanged) v.onRefChanged();
                        }
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
                sectionHeader(v.enPlanTrack ? "ENU 误差序列 / E-N 轨迹图" : "ENU 收敛轨迹 / E-N 平面图");

                const ImGuiStyle &style = ImGui::GetStyle();
                const float defH = std::clamp(ImGui::GetContentRegionAvail().y - style.ItemSpacing.y,
                                              280.0f, 720.0f);
                float &rowH = UI::dragHeight("spp_enu_row", defH);
                bool newed = v.s.newed;
                if (!v.showEnPlan) {
                    renderENUPlot(v.s.axis, v.p.enu_e, v.p.enu_n, v.p.enu_u,
                                  *v.selectedEpoch, newed,
                                  v.hasFloatSeries, v.p.float_e, v.p.float_n, v.p.float_u, v.epochCount, rowH,
                                  v.solvingEpoch);
                    UI::heightGrip("##enuGrip", rowH, 180.0f, 1200.0f);
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
                                      v.hasFloatSeries, v.p.float_e, v.p.float_n, v.p.float_u, v.epochCount, rowH,
                                  v.solvingEpoch);
                        ImGui::TableSetColumnIndex(1);

                        renderENPlanView(v, rowH);
                        ImGui::EndTable();
                    }
                } else {

                    renderENUPlot(v.s.axis, v.p.enu_e, v.p.enu_n, v.p.enu_u,
                                  *v.selectedEpoch, newed,
                                  v.hasFloatSeries, v.p.float_e, v.p.float_n, v.p.float_u, v.epochCount,
                                  std::max(220.0f, rowH * 0.55f), v.solvingEpoch);
                    renderENPlanView(v, std::min(rowH, 380.0f));
                }
                UI::heightGrip("##enuGrip", rowH, 180.0f, 1200.0f);
            }
        }


        void renderSkyResidualRow(const PosTabView &v) {
            float &rowH = UI::dragHeight(
                "spp_sky_row",
                std::max(280.0f, std::min(ImGui::GetContentRegionAvail().x * 0.30f, 480.0f)));
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
                        renderSkyplot(v.skyTracks, v.curSkyPts, std::min(fillH, 640.0f));
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
                                              &ylo, &yhi, &plo, &phi, 0.0f, v.solvingEpoch);
                    }
                    ImGui::EndChild();
                }
                ImGui::EndTable();
            }
            UI::heightGrip("##skyGrip", rowH, 200.0f, 1200.0f);
        }


        void renderDopPlot(const PosTabView &v, const bool newed, const float plotH) {
            if (ImPlot::BeginPlot("DOP", ImVec2(-1, plotH))) {
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
                if (n > 0) {
                    solveCursor(v.solvingEpoch);
                    epochCursor(3, v.selectedEpoch, v.epochCount);
                }
                ImPlot::EndPlot();
            }
        }


        void renderSigmaDopRow(const PosTabView &v) {

            if (v.showSigmaDop && v.epochCount > 0 && !v.s.axis.kept.empty()) {
                sectionHeader("σ / 精度 (P·V·DOP)");
                const bool newed = v.s.newed;
                float &ph = UI::dragHeight("spp_sigma_row", 350.0f);

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
                        ImGui::TableNextRow(ImGuiTableRowFlags_None, ph);
                        for (int k = 0; k < static_cast<int>(cols.size()); ++k) {
                            ImGui::TableSetColumnIndex(k);
                            if (cols[k].isDop) {
                                renderDopPlot(v, newed, ph);
                                continue;
                            }
                            const bool sigPWithFloat = cols[k].name == std::string("SigmaP")
                                                       && v.hasFloatSeries && !v.p.sigmaPFloatVals.empty();
                            if (ImPlot::BeginPlot(cols[k].name, ImVec2(-1, ph),
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
                                if (nSigma > 0) {
                                    solveCursor(v.solvingEpoch);
                                    epochCursor(3, v.selectedEpoch, v.epochCount);
                                }
                                ImPlot::EndPlot();
                            }
                        }
                        ImGui::EndTable();
                        UI::heightGrip("##sigmaGrip", ph, 160.0f, 1200.0f);
                    }
                }
            }
        }

        // LEO 专用：相对参考轨道的 R/T/N/3D 偏差轨迹
        void renderRtnDeviation(const PosTabView &v) {
            if (!v.showRtn || v.rtnTimes.empty()) return;
            sectionHeader("R/T/N/3D 偏差");
            float &ph = UI::dragHeight("spp_rtn_row", 380.0f);
            if (ImPlot::BeginPlot("R/T/N/3D 偏差", ImVec2(-1, ph))) {
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
                    solveCursor(v.solvingEpoch);
                }
                epochCursor(2, v.selectedEpoch, v.epochCount);
                ImPlot::EndPlot();
            }
            UI::heightGrip("##rtnGrip", ph, 160.0f, 1200.0f);
        }

    }

    void renderPositioningTab(const PosTabView &v) {
        renderStatusAndEpochNav(v);
        renderRefECEFInput(v);
        renderFilterDirection(v);

        ImGui::Separator();

        if (v.epochCount == 0) {
            if (!v.busy)
                ImGui::TextDisabled("无解算历元。");
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
