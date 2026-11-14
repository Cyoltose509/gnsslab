#include "solvers/ins/Task.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "core/geom/CoordStruct.h"
#include "imgui.h"
#include "implot.h"
#include "solvers/view/GuiCharts.h"
#include "solvers/view/GuiExport.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiSolverPanel.h"
#include "ui/DragBar.h"
#include "ui/GuiColors.h"

namespace Solvers::INS {
    namespace {
        constexpr std::size_t kMaxPts = 4000;
        constexpr float kPlanH = 340.0f;   // E-N 平面图默认高度，明显高于序列图
        constexpr float kSeriesH = 190.0f;
        constexpr float kChartMinH = 90.0f;
        constexpr float kChartMaxH = 1200.0f;

        bool followTail = true;

        struct Line {
            const char *name;
            const std::vector<double> *y;
            ImVec4 color;
        };

        struct XYLine {
            const char *name;
            const std::vector<double> *x;
            const std::vector<double> *y;
            ImVec4 color;
        };

        std::size_t strideOf(const std::size_t n) {
            return n > kMaxPts ? (n + kMaxPts - 1) / kMaxPts : 1;
        }

        void appendPoint(Curves &c, const std::size_t idx, const NavState &s,
                         const double lat0, const double lon0) {
            double nn = 0.0, ee = 0.0;
            Coord::LLHToLocalNE(s.llh[0], s.llh[1], lat0, lon0, nn, ee);
            const Euler e = s.qbn.toEuler();
            c.epoch.push_back(static_cast<double>(idx));
            c.e.push_back(ee);
            c.n.push_back(nn);
            c.h.push_back(s.llh[2]);
            c.vN.push_back(s.vned[0]);
            c.vE.push_back(s.vned[1]);
            c.vD.push_back(s.vned[2]);
            c.vNorm.push_back(s.vned.norm());
            c.roll.push_back(e.roll * RAD_TO_DEG);
            c.pitch.push_back(e.pitch * RAD_TO_DEG);
            c.heading.push_back(e.yaw * RAD_TO_DEG);
        }

        ImPlotSpec lineSpec(const ImVec4 &c) {
            ImPlotSpec s;
            s.LineColor = c;
            s.LineWeight = 1.5f;
            return s;
        }

        void drawTimePlot(const char *title, const char *ylabel, const char *key,
                          const std::vector<double> &epoch, const Line *lines, const int count,
                          int &sel, const int epochCount) {
            if (epoch.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            float &h = UI::dragHeight(key, kSeriesH);
            if (ImPlot::BeginPlot(title, ImVec2(-1.0f, h))) {
                ImPlot::SetupAxes("Epoch", ylabel);
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                for (int i = 0; i < count; ++i) {
                    if (!lines[i].y || lines[i].y->size() != epoch.size()) continue;
                    ImPlot::PlotLine(lines[i].name, epoch.data(), lines[i].y->data(),
                                     static_cast<int>(epoch.size()), lineSpec(lines[i].color));
                }
                Charts::epochCursor(0, &sel, epochCount);
                ImPlot::EndPlot();
            }
            ImGui::PushID(key);
            UI::heightGrip("##grip", h, kChartMinH, kChartMaxH);
            ImGui::PopID();
        }

        void drawPlanPlot(const XYLine *lines, const int count, const double curE, const double curN) {
            if (count <= 0 || !lines[0].x || lines[0].x->empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            float &h = UI::dragHeight("ins_track", kPlanH);
            if (ImPlot::BeginPlot("E-N 平面轨迹###ins_track", ImVec2(-1.0f, h), ImPlotFlags_Equal)) {
                ImPlot::SetupAxes("E (m)", "N (m)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                for (int i = 0; i < count; ++i) {
                    if (!lines[i].x || !lines[i].y || lines[i].x->size() != lines[i].y->size()) continue;
                    ImPlot::PlotLine(lines[i].name, lines[i].x->data(), lines[i].y->data(),
                                     static_cast<int>(lines[i].x->size()), lineSpec(lines[i].color));
                }
                ImPlotSpec ps;
                ps.Marker = ImPlotMarker_Circle;
                ps.MarkerSize = 7.0f;
                ps.LineColor = UI::kColInfo;
                ImPlot::PlotScatter("当前历元", &curE, &curN, 1, ps);
                ImPlot::EndPlot();
            }
            ImGui::PushID("ins_track");
            UI::heightGrip("##grip", h, 160.0f, kChartMaxH);
            ImGui::PopID();
        }

        void renderCharts(const CurveCache &cc, const double curE, const double curN,
                          int &sel, const int n, int &shownSolution) {
            ImGui::SeparatorText("导航曲线");
            ImGui::RadioButton("自由解", &shownSolution, 0);
            ImGui::SameLine();
            ImGui::RadioButton("零速修正", &shownSolution, 1);

            const Curves &a = cc.freeSol;
            const Curves &b = cc.zuptSol;
            const bool showFree = shownSolution != 1;
            const bool showZupt = shownSolution != 0;

            XYLine track[2];
            int trackCount = 0;
            if (showFree) track[trackCount++] = {"自由解", &a.e, &a.n, UI::kColX};
            if (showZupt) track[trackCount++] = {"零速修正", &b.e, &b.n, UI::kColPass};
            drawPlanPlot(track, trackCount, curE, curN);

            {
                Line ls[2];
                int cnt = 0;
                if (showFree) ls[cnt++] = {"自由解 h", &a.h, UI::kColX};
                if (showZupt) ls[cnt++] = {"零速修正 h", &b.h, UI::kColPass};
                drawTimePlot("高程###ins_h", "h (m)", "ins_h", a.epoch, ls, cnt, sel, n);
            }
            {
                Line ls[2];
                int cnt = 0;
                if (showFree) ls[cnt++] = {"自由解 |v|", &a.vNorm, UI::kColX};
                if (showZupt) ls[cnt++] = {"零速修正 |v|", &b.vNorm, UI::kColPass};
                drawTimePlot("速度模###ins_vnorm", "|v| (m/s)", "ins_vnorm", a.epoch, ls, cnt, sel, n);
            }

            const Curves &c = showZupt && !showFree ? b : a;
            const Line vel[] = {
                {"vN", &c.vN, UI::kColX}, {"vE", &c.vE, UI::kColY}, {"vD", &c.vD, UI::kColZ}
            };
            drawTimePlot("速度三分量###ins_v", "v (m/s)", "ins_vel", c.epoch, vel, 3, sel, n);
            const Line att[] = {
                {"横滚", &c.roll, UI::kColX}, {"俯仰", &c.pitch, UI::kColY},
                {"航向", &c.heading, UI::kColZ}
            };
            drawTimePlot("姿态###ins_att", "角度 (°)", "ins_att", c.epoch, att, 3, sel, n);
        }

        /// 结果页要用的标量元信息：nav 在解算中一直在长，不能整份拷给界面。
        struct Meta {
            std::size_t total = 0;
            std::size_t zuptCount = 0;
            double t0 = 0.0;
            std::string modelName;
            std::string scaleNote;
            ImuScaleSource scaleSource = ImuScaleSource::Record;
            ImuScale scale;
            AlignInfo align;
            Eigen::Vector3d accelBias = Eigen::Vector3d::Zero();
            bool accelBiasApplied = false;
        };

        void renderInfo(const Meta &m, const NavState &s, const bool applied, const int sel,
                        const bool zuptView, const std::function<void()> &onExport) {
            const XYZ xyz = Coord::BLHToXYZ(BLH(s.llh[0], s.llh[1], s.llh[2]), Frame::WGS84);
            const Euler e = s.qbn.toEuler();

            ImGui::SeparatorText(zuptView ? "当前历元解算信息（零速修正解）" : "当前历元解算信息（自由解）");
            if (ImGui::BeginTable("##ins_state", 4,
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("数据");
                ImGui::TableSetupColumn("分量 1");
                ImGui::TableSetupColumn("分量 2");
                ImGui::TableSetupColumn("分量 3");
                ImGui::TableHeadersRow();
                auto row = [](const char *label, const double v1, const double v2,
                              const double v3, const char *fmt) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", label);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text(fmt, v1);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text(fmt, v2);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text(fmt, v3);
                };
                row("ECEF (m)", xyz[0], xyz[1], xyz[2], "%.4f");
                row("BLH (°/°/m)", s.llh[0] * RAD_TO_DEG, s.llh[1] * RAD_TO_DEG, s.llh[2], "%.7f");
                row("速度 NED (m/s)", s.vned[0], s.vned[1], s.vned[2], "%.4f");
                row("姿态 (°)", e.roll * RAD_TO_DEG, e.pitch * RAD_TO_DEG, e.yaw * RAD_TO_DEG, "%.4f");
                ImGui::EndTable();
            }

            ImGui::SeparatorText("历元状态");
            if (ImGui::BeginTable("##ins_ep", 5,
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("t (s)");
                ImGui::TableSetupColumn("Wk");
                ImGui::TableSetupColumn("SOW (s)");
                ImGui::TableSetupColumn("|v| (m/s)");
                ImGui::TableSetupColumn("ZUPT");
                ImGui::TableHeadersRow();
                ImGui::TableNextRow();
                const unsigned wk = static_cast<unsigned>(s.t / FULL_WEEK);
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%.3f", s.t - m.t0);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%u", wk);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.3f", s.t - wk * FULL_WEEK);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.4f", s.vned.norm());
                ImGui::TableSetColumnIndex(4);
                ImGui::TextColored(applied ? UI::kColPass : UI::kColMute,
                                   applied ? "生效" : "未生效");
                ImGui::EndTable();
            }

            ImGui::SeparatorText("解算设置");
            ImGui::Text("IMU 型号：%s | 共 %zu 个历元 | 当前第 %d 个",
                        m.modelName.empty() ? "未识别" : m.modelName.c_str(), m.total, sel + 1);
            ImGui::Text("零速修正累计生效 %zu 次", m.zuptCount);
            ImGui::Text("当量：%.3e (m/s)/LSB, %.3e rad/LSB @ %.0f Hz",
                        m.scale.accelMpsPerLsb, m.scale.gyroRadPerLsb, m.scale.rateHz);
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(m.scaleSource == ImuScaleSource::Record ? UI::kColInfo : UI::kColWarn,
                               "当量来源：%s", m.scaleNote.c_str());
            if (m.align.used) {
                ImGui::Text("初始姿态：开头静止段 0–%.1f s 自动粗对准 → roll %+.4f° pitch %+.4f° yaw %+.4f°",
                            m.align.spanSec, m.align.rollDeg, m.align.pitchDeg, m.align.headingDeg);
                ImGui::TextColored(m.align.headingReliable ? UI::kColInfo : UI::kColWarn,
                                   "航向跨窗散布 %.2f°（%s）", m.align.yawSpreadDeg,
                                   m.align.headingReliable ? "自洽" : "低精度 IMU 航向不可靠");
            }
            if (m.accelBiasApplied) {
                ImGui::Text("静置段加计零偏：%+.3e / %+.3e / %+.3e m/s²（已扣除）",
                            m.accelBias[0], m.accelBias[1], m.accelBias[2]);
            }
            ImGui::PopTextWrapPos();

            if (ImGui::Button("导出...", ImVec2(-FLT_MIN, 40))) onExport();
            ImGui::TextDisabled("导出主列跟随上面选中的解；Zupt* 附加列恒为零速修正解。");
        }
    }

    void CurveCache::sync(const Result &r, const double lat0, const double lon0) {
        const std::size_t n = r.nav.size();
        const std::size_t st = strideOf(n);
        if (st != stride) {
            freeSol = Curves{};
            zuptSol = Curves{};
            stride = st;
            next = 0;
        }
        std::size_t i = next;
        for (; i < n; i += stride) {
            appendPoint(freeSol, i, r.nav[i], lat0, lon0);
            if (i < r.navZupt.size()) appendPoint(zuptSol, i, r.navZupt[i], lat0, lon0);
        }
        next = i;
    }

    void Task::renderResult() {
        const bool isDone = done.load();
        const bool hasError = errorFlag();
        if (hasError) ImGui::TextColored(UI::kColFail, "错误：%s", errorMessage().c_str());

        Meta m;
        const auto snap = snapshot();
        if (snap) {
            std::lock_guard lk(resMutex);
            m.total = snap->nav.size();
            m.zuptCount = snap->zuptCount;
            m.modelName = snap->modelName;
            m.scaleNote = snap->scaleNote;
            m.scaleSource = snap->scaleSource;
            m.scale = snap->scale;
            m.align = snap->align;
            m.accelBias = snap->accelBias;
            m.accelBiasApplied = snap->accelBiasApplied;
            if (m.total > 0) {
                const double lat0 = snap->nav.front().llh[0];
                const double lon0 = snap->nav.front().llh[1];
                curves.sync(*snap, lat0, lon0);
            }
        }

        if (!isDone && !hasError) {
            if (m.total > 0) {
                ImGui::TextColored(UI::kColInfo, "正在处理 [%s]：正在解算第 %zu 个历元",
                                   fileName.c_str(), m.total);
            } else if (aligning.load()) {
                const double rate = m.scale.rateHz > 0.0 ? m.scale.rateHz : 200.0;
                ImGui::TextColored(UI::kColInfo, "正在读取开头静止段做粗对准 [%s]：%.1f s",
                                   fileName.c_str(), static_cast<double>(headSamples.load()) / rate);
            } else {
                ImGui::TextColored(UI::kColInfo, "正在读取 IMU 日志 [%s] …", fileName.c_str());
            }
            ImGui::SameLine();
            if (ImGui::Button("停止解算")) cancel();
        } else if (!hasError && m.total > 0) {
            ImGui::TextColored(UI::kColPass, "处理完成：共 %zu 个历元", m.total);
        }

        if (m.total == 0) {
            if (!hasError && isDone) ImGui::TextDisabled("没有解算结果");
            return;
        }

        const int n = static_cast<int>(m.total);
        int sel = view.selectedEpoch;
        if (followTail || sel < 0 || sel >= n) sel = n - 1;

        m.t0 = 0.0;
        NavState cur, curZ;
        bool applied = false, hasZupt = false;
        {
            std::lock_guard lk(resMutex);
            m.t0 = snap->nav.front().t;
            cur = snap->nav[sel];
            hasZupt = sel < static_cast<int>(snap->navZupt.size());
            if (hasZupt) curZ = snap->navZupt[sel];
            if (sel < static_cast<int>(snap->zuptApplied.size())) applied = snap->zuptApplied[sel] != 0;
        }

        char info[128];
        const unsigned wk = static_cast<unsigned>(cur.t / FULL_WEEK);
        std::snprintf(info, sizeof info, "t = %.3f s | Wk %u SOW %.3f",
                      cur.t - m.t0, wk, cur.t - wk * FULL_WEEK);
        Helpers::renderEpochNav(sel, n, info);
        followTail = sel >= n - 1;
        ImGui::Separator();

        const NavState &curShown = shownSolution == 1 && hasZupt ? curZ : cur;
        double ce = 0.0, cn = 0.0;
        {
            std::lock_guard lk(resMutex);
            const double lat0 = snap->nav.front().llh[0];
            const double lon0 = snap->nav.front().llh[1];
            Coord::LLHToLocalNE(curShown.llh[0], curShown.llh[1], lat0, lon0, cn, ce);
        }

        const float rowH = std::max(320.0f, ImGui::GetContentRegionAvail().y
                                           - ImGui::GetStyle().ItemSpacing.y);
        if (ImGui::BeginTable("##ins_main", 2,
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn("曲线", ImGuiTableColumnFlags_WidthStretch, 0.6f);
            ImGui::TableSetupColumn("当前历元", ImGuiTableColumnFlags_WidthStretch, 0.4f);
            ImGui::TableNextRow(ImGuiTableRowFlags_None, rowH);
            ImGui::TableSetColumnIndex(0);
            if (ImGui::BeginChild("##ins_charts"))
                renderCharts(curves, ce, cn, sel, n, shownSolution);
            ImGui::EndChild();
            ImGui::TableSetColumnIndex(1);
            if (ImGui::BeginChild("##ins_info"))
                renderInfo(m, curShown, applied, sel, shownSolution == 1,
                           [this] { ExportUI::open(shared_from_this()); });
            ImGui::EndChild();
            ImGui::EndTable();
        }
        view.selectedEpoch = sel;
        ExportUI::render();
    }
}  // namespace Solvers::INS
