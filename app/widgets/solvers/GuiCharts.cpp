#define NOMINMAX
#include "GuiCharts.h"

#include "Const.h"
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <cfloat>
#include <limits>
#include <set>
#include <excpt.h>

#include "implot.h"
#include "implot3d.h"

namespace GuiCharts {
    ImU32 getSatCol(const char s) {
        switch (s) {
            case 'G': return IM_COL32(80, 220, 120, 255);
            case 'C': return IM_COL32(255, 130, 60, 255);
            case 'R': return IM_COL32(80, 200, 255, 255);
            case 'E': return IM_COL32(240, 220, 70, 255);
            default: return IM_COL32(220, 220, 220, 255);
        }
    }

    // ----------------------------------------------------------
    // 卫星数据列表
    // ----------------------------------------------------------
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
        // 各处理器可能给不同行赋不同 extra 列数；取最大并做边界保护，防止 vector 越界
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
                const char *hdr = (k < rows.front().extra.size()) ? rows.front().extra[k].first.c_str() : "?";
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

    // ----------------------------------------------------------
    // ENU 收敛轨迹图
    // ----------------------------------------------------------
    void renderENUPlot(const std::vector<double> &times,
                       const std::vector<double> &enu_e,
                       const std::vector<double> &enu_n,
                       const std::vector<double> &enu_u,
                       int &selectedEpoch, bool &newed, const bool fixedY) {
        const int n = static_cast<int>(times.size());
        if (n <= 0 || !ImPlot::BeginPlot("ENU 偏差", ImVec2(-1, 380))) return;
        ImPlot::SetupAxes("Epoch", "E/N/U (m)");
        if (fixedY) ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
        // 过滤非有限值，避免 ImPlot SetupAxisLimits / PlotLine 断言
        std::vector<double> tt, ee, nn, uu;
        tt.reserve(n);
        ee.reserve(n);
        nn.reserve(n);
        uu.reserve(n);
        for (int i = 0; i < n; ++i) {
            if (i >= static_cast<int>(enu_e.size()) || i >= static_cast<int>(enu_n.size()) ||
                i >= static_cast<int>(enu_u.size()))
                continue;
            if (!std::isfinite(times[i]) || !std::isfinite(enu_e[i]) ||
                !std::isfinite(enu_n[i]) || !std::isfinite(enu_u[i]))
                continue;
            tt.push_back(times[i]);
            ee.push_back(enu_e[i]);
            nn.push_back(enu_n[i]);
            uu.push_back(enu_u[i]);
        }
        const int nf = static_cast<int>(tt.size());
        // PPP 初始误差可达百米量级，收敛后亚米，故不固定 Y 轴（按需自动缩放）
        if (newed && nf > 0 && std::isfinite(tt.back()))
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Always);
        newed = false;
        if (nf > 0) {
            ImPlot::PlotLine("E", tt.data(), ee.data(), nf);
            ImPlot::PlotLine("N", tt.data(), nn.data(), nf);
            ImPlot::PlotLine("U", tt.data(), uu.data(), nf);
        }
        if (nf > 0) {
            auto sx = static_cast<double>(selectedEpoch);
            ImPlot::DragLineX(0, &sx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f, ImPlotDragToolFlags_NoFit);
            int se = static_cast<int>(std::lround(sx));
            if (se < 0) se = 0;
            if (se >= nf) se = nf - 1;
            selectedEpoch = se;
        }
        ImPlot::EndPlot();
    }

    // ----------------------------------------------------------
    // 逐卫星残差图
    // ----------------------------------------------------------
    void renderSatResidualPlot(const std::map<SatID, std::vector<double> > &satResT,
                               const std::map<SatID, std::vector<double> > &satResV,
                               int &selectedEpoch, const int epochCount, bool &newed,
                               const double *yLo, const double *yHi, const float plotH) {
        if (ImPlot::BeginPlot("逐卫星残差", ImVec2(-1, plotH))) {
            ImPlot::SetupAxes("Epoch", "残差 (m)");
            {
                ImPlot::SetupAxisLimits(ImAxis_X1, 0, std::max(10.0, static_cast<double>(epochCount)), ImPlotCond_Once);
                if (yLo && yHi && std::isfinite(*yLo) && std::isfinite(*yHi) && *yHi > *yLo)
                    ImPlot::SetupAxisLimits(ImAxis_Y1, *yLo, *yHi, ImPlotCond_Once);
            }
            for (auto &[sat, vals]: satResV) {
                auto it = satResT.find(sat);
                if (it == satResT.end() || vals.empty()) continue;
                // 防御：时间戳数量必须不少于残差数量，否则 t[i] 越界
                if (it->second.size() < vals.size()) continue;
                const double *t = it->second.data();
                const double *v = vals.data();
                const auto n = static_cast<int>(vals.size());
                const std::string base = sat.toString();
                std::vector<double> tt, vv;
                tt.reserve(n + 4);
                vv.reserve(n + 4);
                for (int i = 0; i < n; ++i) {
                    // 过滤非有限值，防止 ImPlot/assert 崩溃
                    if (!std::isfinite(v[i])) continue;
                    if (i > 0 && std::isfinite(t[i]) && std::isfinite(t[i - 1]) && (t[i] - t[i - 1]) > 2.5) {
                        tt.push_back(t[i - 1]);
                        vv.push_back(std::numeric_limits<double>::quiet_NaN()); // 断点：前后两段不相连
                    }
                    tt.push_back(t[i]);
                    vv.push_back(v[i]);
                }
                if (!tt.empty() && tt.size() == vv.size())
                    ImPlot::PlotLine(base.c_str(), tt.data(), vv.data(), static_cast<int>(tt.size()));
            }
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

    void RenderSkyplot(const std::map<SatID, std::vector<SkyPoint> > &tracks,
                       const std::vector<std::pair<SatID, SkyPoint> > &curPts,
                       const float sizePx) {
        const float availW = ImGui::GetContentRegionAvail().x;
        const float sizeW = std::max(220.0f, std::min(availW, sizePx));
        constexpr float pad = 30.0f;
        const float R = std::max(70.0f, (sizeW - 2.0f * pad) / 2.0f);
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
            for (size_t i = 1; i <= pts.size(); ++i) {
                if (i == pts.size() || pts[i].epIdx - pts[i - 1].epIdx > 2) {
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

    // LEO 轨迹：直接用 ImPlot3D 画 ECEF 原始 XYZ（不换算、不依赖参考轨道）。
    // 参考轨道轨迹仅在存在时叠加；无参考轨道时只画 LEO 自身轨迹。
    void RenderLeo3D(const Eigen::Vector3d &leoPos,
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
            for (const auto &tp: traj) {
                const double x = tp.pos.x(), y = tp.pos.y(), z = tp.pos.z();
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                if (tp.pos.norm() < 1e3) continue;
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
                double px[1] = {leoPos.x()}, py[1] = {leoPos.y()}, pz[1] = {leoPos.z()};
                ImPlot3D::PlotScatter("当前位置", px, py, pz, 1);
            }
            ImPlot3D::EndPlot();
        }
    }

    // SEH 兜底（**刻意保留**）：ImPlot3D 在极端/并发数据下可能触发断言。
    // 与"包住整帧渲染"的宽范围 SEH 不同，这里只包住单个第三方调用，
    // 且刻意隔离在这个不含 C++ 析构的函数里（SEH 与 C++ 析构混用本身就有问题）；
    // 捕获后跳过本图，避免破坏外层 TabItem/Table/Child 栈。
    void RenderLeo3DSeh(const Eigen::Vector3d &leoPos,
                        const std::vector<TrajPoint> &leoTraj,
                        const std::vector<TrajPoint> &refTraj,
                        const float sizePx) {
        __try {
            RenderLeo3D(leoPos, leoTraj, refTraj, sizePx);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "LEO 3D 轨迹渲染异常，已跳过");
        }
    }

    // ------------------------------------------------------------------
    // RenderPositioningTab 的渲染主体按职责拆分为下列无状态辅助函数；
    // 所有状态均来自 PosTabView（只读），不新增任何 static 可变状态。
    // ------------------------------------------------------------------
    namespace {
        // 章节标题；flag 非空时在标题右侧补一个 显示/隐藏 开关
        static void SectionHeader(const char *title, bool *flag) {
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
        static bool vis(const bool *flag, const bool fallback) {
            return flag ? *flag : fallback;
        }

        // 顶部状态行 + 历元导航（首/末/前后 + 滑块 + 历元信息）
        static void renderStatusAndEpochNav(const PosTabView &v) {
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
            }
        }

        // 参考真值 ECEF 输入（含剪贴板坐标解析）
        static void renderRefEcefInput(const PosTabView &v) {
            if (v.refECEF && v.showRefEnu && v.showRefInput) {
                ImGui::Text("参考真值:");
                ImGui::SameLine();
                ImGui::PushItemWidth(200);
                if (ImGui::InputDouble("X/m", &(*v.refECEF)[0])) { if (v.onRefChanged) v.onRefChanged(); }
                ImGui::SameLine();
                if (ImGui::InputDouble("Y/m", &(*v.refECEF)[1])) { if (v.onRefChanged) v.onRefChanged(); }
                ImGui::SameLine();
                if (ImGui::InputDouble("Z/m", &(*v.refECEF)[2])) { if (v.onRefChanged) v.onRefChanged(); }
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
                        if (v.onRefChanged) v.onRefChanged();
                    }
                }
            }
        }

        // 选中卫星的观测详情表（伪距 / 载波 / Doppler / 强度，按频点分行）
        static void renderObsDetailTable(const PosTabView &v) {
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

        // 左右并排：卫星概览表（含选中星观测详情） | 位置面板（含导出 CSV）
        static void renderSatelliteAndPositionPanels(const PosTabView &v, const float th) {
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
                                    ImGui::Text("ECEF: (%.4f, %.4f, %.4f)", v.xyz[0], v.xyz[1], v.xyz[2]);
                                    if (v.showRefEnu) {
                                        ImGui::Text("  REF: (%.4f, %.4f, %.4f)", v.refECEF ? (*v.refECEF)[0] : 0,
                                                    v.refECEF ? (*v.refECEF)[1] : 0, v.refECEF ? (*v.refECEF)[2] : 0);
                                        ImGui::Text("  ENU: (%.4f, %.4f, %.4f)", v.enu[0], v.enu[1], v.enu[2]);
                                    }
                                    ImGui::Text("  BLH: (%.8f, %.8f, %.4f)", v.blh[0] * RAD_TO_DEG, v.blh[1] * RAD_TO_DEG, v.blh[2]);
                                    ImGui::Text("  σP: %.3f", v.sigmaP);
                                    if (v.showZtd)
                                        ImGui::Text("  ZTD: %.4f m", v.ztd);
                                    if (v.showVelDop) {
                                        if (v.showVel) {
                                            ImGui::SeparatorText("速度");
                                            ImGui::Text("  (%.4f, %.4f, %.4f) m/s", v.vel[0], v.vel[1], v.vel[2]);
                                        }
                                        ImGui::SeparatorText("DOP");
                                        ImGui::Text("  PDOP:%.2f GDOP:%.2f HDOP:%.2f VDOP:%.2f TDOP:%.2f",
                                                    v.pdop, v.gdop, v.hdop, v.vdop, v.tdop);
                                        ImGui::Text("  卫星: %d/%d", v.numSatsResult, v.numObs);
                                    }
                                } else {
                                    ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1),
                                                       "%s", v.noSolveMsg.empty() ? "无定位解" : v.noSolveMsg.c_str());
                                }
                                ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 50);
                                if (v.onExportCsv) {
                                    const bool canExport = v.epochCount > 0;
                                    if (!canExport) ImGui::BeginDisabled();
                                    if (ImGui::Button("导出 CSV", ImVec2(-FLT_MIN, 40))) {
                                        v.onExportCsv();
                                    }
                                    if (!canExport) ImGui::EndDisabled();
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

        // ENU 收敛轨迹（LEO 的 show3d 模式下隐藏）
        static void renderEnuConvergence(const PosTabView &v) {
            // ENU 收敛轨迹：LEO 是运动目标（ENU 相对固定测站无意义），show3d 模式(LEO)下隐藏；
            // SPP/PPP 静态站才显示 ENU 收敛。
            if (v.epochCount > 0 && !v.times.empty() && !v.show3d &&
                !v.enu_e.empty() && !v.enu_n.empty() && !v.enu_u.empty()) {
                SectionHeader("ENU 收敛轨迹", nullptr);
                bool newed = v.newed;
                renderENUPlot(v.times, v.enu_e, v.enu_n, v.enu_u,
                              *v.selectedEpoch, newed, v.fixedY);
            }
        }

        // 天顶图 / LEO 3D 轨迹 与 逐星残差 并排
        static void renderSkyResidualRow(const PosTabView &v) {
            // 3D 卫星坐标图 / 天顶图 + 逐星残差（并排；标题右侧带开关）
            {
                bool *d3Tg = v.renderToggles ? &v.renderToggles->plot3d : nullptr;
                bool *resTg = v.renderToggles ? &v.renderToggles->residual : nullptr;
                // LEO: plot3d 开关控制左列；SPP/PPP 无开关，天顶图恒显示
                const bool left = vis(d3Tg, true);
                const bool right = vis(resTg, v.showResidual) && !v.resT.empty(); //NOLINT
                if (left || right || v.renderToggles) {
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
                                if (v.show3d)
                                    RenderLeo3DSeh(v.leoPos3d, v.leoTraj, v.leoRefTraj, rowH);
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
                                SectionHeader("逐星残差", resTg);
                                if (right) {
                                    const double ylo = v.robustRes ? v.resYlo : -8.0;
                                    const double yhi = v.robustRes ? v.resYhi : 8.0;
                                    bool rnewed = v.newed;
                                    renderSatResidualPlot(v.resT, v.resV,
                                                          *v.selectedEpoch, v.epochCount, rnewed,
                                                          &ylo, &yhi, rowH);
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

        // σP / σV / PDOP 时间序列（动态列）
        static void renderSigmaDopRow(const PosTabView &v) {
            // σP / σV / PDOP
            if (v.showSigmaDop && v.epochCount > 0 && !v.times.empty()) {
                SectionHeader("σ / 精度 (P·V·PDOP)", nullptr);
                const bool newed = v.newed; // ENU 之后已置 false（与 SPP 一致：X 轴自动缩放）
                // 动态列：只包含启用且非空的序列
                struct Col {
                    const char *name;
                    const std::vector<double> *data;
                };
                std::vector<Col> cols;
                if (!v.sigmaPs.empty()) cols.push_back({"SigmaP", &v.sigmaPs});
                if (v.showSigmaV && !v.sigmaVs.empty()) cols.push_back({"SigmaV", &v.sigmaVs});
                if (v.showPdop && !v.pdops.empty()) cols.push_back({"PDOP", &v.pdops});
                if (!cols.empty()) {
                    if (ImGui::BeginTable("##bp3", static_cast<int>(cols.size()),
                                          ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                        for (int c = 0; c < static_cast<int>(cols.size()); ++c)
                            ImGui::TableSetupColumn(cols[c].name, ImGuiTableColumnFlags_WidthStretch,
                                                    1.0f / static_cast<float>(cols.size()));
                        ImGui::TableNextRow();
                        for (int k = 0; k < static_cast<int>(cols.size()); ++k) {
                            ImGui::TableSetColumnIndex(k);
                            if (ImPlot::BeginPlot(cols[k].name, ImVec2(-1, 350), ImPlotFlags_NoLegend)) {
                                ImPlot::SetupAxes("Epoch", "m");
                                ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
                                // 过滤非有限值，避免 ImPlot 断言
                                const int nSigmaRaw = static_cast<int>(v.times.size());
                                const bool okSigmaRaw = (nSigmaRaw > 0 &&
                                                         static_cast<int>(cols[k].data->size()) == nSigmaRaw);
                                std::vector<double> tt, yy;
                                if (okSigmaRaw) {
                                    tt.reserve(nSigmaRaw);
                                    yy.reserve(nSigmaRaw);
                                    for (int i = 0; i < nSigmaRaw; ++i) {
                                        if (!std::isfinite(v.times[i]) || !std::isfinite((*cols[k].data)[i])) continue;
                                        tt.push_back(v.times[i]);
                                        yy.push_back((*cols[k].data)[i]);
                                    }
                                }
                                const int nSigma = static_cast<int>(tt.size());
                                if (nSigma > 0 && newed && std::isfinite(tt.back()))
                                    ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Always);
                                if (nSigma > 0)
                                    ImPlot::PlotLine(cols[k].name, tt.data(), yy.data(), nSigma);
                                // 与其他时间序列图一致的可拖拽时间游标（与 selectedEpoch 双向同步）
                                if (nSigma > 0) {
                                    double sx = (v.selectedEpoch ? static_cast<double>(*v.selectedEpoch) : 0.0);
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
        static void renderRtnDeviation(const PosTabView &v) {
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
                            // 防御：SolverThread 每帧 push 多组向量，快照后仍要求长度一致；
                            // 同时过滤非有限值，避免 ImPlot 断言/崩溃。
                            const auto nRtnRaw = static_cast<int>(v.rtnTimes.size());
                            std::vector<double> rt, rr, rrt, rrn, rd3;
                            rt.reserve(v.rtnTimes.size());
                            rr.reserve(v.rtnTimes.size());
                            rrt.reserve(v.rtnTimes.size());
                            rrn.reserve(v.rtnTimes.size());
                            rd3.reserve(v.rtnTimes.size());
                            for (int i = 0; i < nRtnRaw; ++i) {
                                if (!std::isfinite(v.rtnTimes[i])) continue;
                                if (!std::isfinite(v.rtnR[i]) || !std::isfinite(v.rtnT[i]) ||
                                    !std::isfinite(v.rtnN[i]) || !std::isfinite(v.rtnD3[i]))
                                    continue;
                                rt.push_back(v.rtnTimes[i]);
                                rr.push_back(v.rtnR[i]);
                                rrt.push_back(v.rtnT[i]);
                                rrn.push_back(v.rtnN[i]);
                                rd3.push_back(v.rtnD3[i]);
                            }
                            const auto nRtn = static_cast<int>(rt.size());
                            if (nRtn > 0 && v.newed)
                                ImPlot::SetupAxisLimits(ImAxis_X1, 0, rt.back(), ImPlotCond_Once);
                            if (nRtn > 0) {
                                ImPlot::PlotLine("R", rt.data(), rr.data(), nRtn);
                                ImPlot::PlotLine("T", rt.data(), rrt.data(), nRtn);
                                ImPlot::PlotLine("N", rt.data(), rrn.data(), nRtn);
                                ImPlot::PlotLine("3D", rt.data(), rd3.data(), nRtn);
                            }
                            // 与其他时间序列图一致的可拖拽时间游标（与 selectedEpoch 双向同步）
                            double sx = (v.selectedEpoch ? static_cast<double>(*v.selectedEpoch) : 0.0);
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
        static void renderRmsRow(const PosTabView &v) {
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
                            const int nRmsRaw = static_cast<int>(v.times.size());
                            const bool okRmsRaw = (nRmsRaw > 0 &&
                                                   static_cast<int>(v.rmsCode.size()) == nRmsRaw &&
                                                   static_cast<int>(v.rmsPhase.size()) == nRmsRaw);
                            std::vector<double> tt, rc, rp;
                            if (okRmsRaw) {
                                tt.reserve(nRmsRaw);
                                rc.reserve(nRmsRaw);
                                rp.reserve(nRmsRaw);
                                for (int i = 0; i < nRmsRaw; ++i) {
                                    if (!std::isfinite(v.times[i]) || !std::isfinite(v.rmsCode[i]) ||
                                        !std::isfinite(v.rmsPhase[i]))
                                        continue;
                                    tt.push_back(v.times[i]);
                                    rc.push_back(v.rmsCode[i]);
                                    rp.push_back(v.rmsPhase[i]);
                                }
                            }
                            const int nRms = static_cast<int>(tt.size());
                            if (nRms > 0 && std::isfinite(tt.back()))
                                ImPlot::SetupAxisLimits(ImAxis_X1, 0, tt.back(), ImPlotCond_Once);
                            if (nRms > 0) {
                                ImPlot::PlotLine("Code RMS", tt.data(), rc.data(), nRms);
                                ImPlot::PlotLine("Phase RMS", tt.data(), rp.data(), nRms);
                            }
                            // 与其他时间序列图一致的可拖拽时间游标（与 selectedEpoch 双向同步）
                            if (nRms > 0) {
                                double sx = (v.selectedEpoch ? static_cast<double>(*v.selectedEpoch) : 0.0);
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
        renderRefEcefInput(v);

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
        renderRmsRow(v);
    }
}
