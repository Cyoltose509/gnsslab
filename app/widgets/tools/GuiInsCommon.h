#pragma once


#include "GuiInsPanel.h"
#include "GuiHelpers.h"

#include "Const.h"
#include "core/Csv.h"

#include "implot.h"
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace Tools::InsPanel::insui {
        template<class Reader, class ReadFn, class OnSample, class OnBatch>
        void pumpImu(Reader &r, ReadFn readOne, const std::atomic<bool> &stop,
                     OnSample onSample, OnBatch onBatch) {
            size_t last = 0, batchMark = 0;
            while (!stop) {
                const bool more = readOne();
                const ImuLog &L = r.getImuLog();
                for (size_t i = last; i < L.size(); ++i) onSample(L[i]);
                last = L.size();
                if (!more) break;
                if (L.size() - batchMark >= 2048) {
                    onBatch(static_cast<long long>(L.size()));
                    batchMark = L.size();
                }
            }
            onBatch(static_cast<long long>(r.getImuLog().size()));
        }

        template<class Reader, class OnSample, class OnBatch>
        void pumpImuReader(Reader &r, const std::atomic<bool> &stop,
                           OnSample onSample, OnBatch onBatch) {
            ImuSample t;
            pumpImu(r, [&] { return r.getNextImu(t); }, stop, onSample, onBatch);
        }

        constexpr float kChartHeight = 360.0f;
        constexpr float kFileChartH = 220.0f;
        constexpr ImVec4 kColX(0.90f, 0.30f, 0.30f, 1.00f);
        constexpr ImVec4 kColY(0.30f, 0.90f, 0.30f, 1.00f);
        constexpr ImVec4 kColZ(0.30f, 0.50f, 1.00f, 1.00f);
        constexpr ImVec4 kColPass(0.35f, 0.85f, 0.45f, 1.00f);
        constexpr ImVec4 kColFail(0.95f, 0.40f, 0.35f, 1.00f);

        inline ImPlotSpec lineSpec(const ImVec4 &c) {
            ImPlotSpec s;
            s.LineColor = c;
            s.LineWeight = 1.5f;
            return s;
        }

        inline void toElapsed(const std::vector<double> &tow, std::vector<double> &out) {
            out.clear();
            if (tow.empty()) return;
            out.reserve(tow.size());
            const double t0 = tow.front();
            for (const double t: tow) out.push_back(t - t0);
        }

        inline void fitXToLatest(const bool newData, const double xmax) {
            if (newData && xmax > 0.0 && std::isfinite(xmax))
                ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, xmax, ImPlotCond_Always);
        }

        template<class Task, class BuildFn>
        void exportCsvDialog(Task &task, const std::string &defaultName, const std::string &displayName,
                             BuildFn build) {
            std::wstring out;
            if (!Solvers::Helpers::saveCSVDialog(defaultName, out)) return;
            Csv::Writer w;
            build(w);
            if (!w.save(out, true)) {
                task.setError(displayName +
                              " 导出失败：无法写入文件（目录不存在、无写权限，或被 Excel 等程序占用）");
                return;
            }
            LOG_INFO << "[导出] " << displayName << " → " << std::filesystem::path(out).string();
        }

        inline void drawAccelChart(const char *title, const PerFileChart &c, const bool chartNew) {
            if (c.t.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(c.t.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kFileChartH))) {
                ImPlot::SetupAxes("时间 t (s)", "加速度计 (LSB)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, c.t.back());
                ImPlot::PlotLine("ax", c.t.data(), c.ax.data(), n, lineSpec(kColX));
                ImPlot::PlotLine("ay", c.t.data(), c.ay.data(), n, lineSpec(kColY));
                ImPlot::PlotLine("az", c.t.data(), c.az.data(), n, lineSpec(kColZ));
                ImPlot::EndPlot();
            }
        }

        inline void drawGyroChart(const char *title, const PerFileChart &c, const bool chartNew) {
            if (c.t.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(c.t.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kFileChartH))) {
                ImPlot::SetupAxes("时间 t (s)", "陀螺仪 (LSB)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, c.t.back());
                ImPlot::PlotLine("gx", c.t.data(), c.gx.data(), n, lineSpec(kColX));
                ImPlot::PlotLine("gy", c.t.data(), c.gy.data(), n, lineSpec(kColY));
                ImPlot::PlotLine("gz", c.t.data(), c.gz.data(), n, lineSpec(kColZ));
                ImPlot::EndPlot();
            }
        }

        // 补偿结果显示：画三段补偿序列（t 应为相对时间，单位自定）
        inline void drawCompChart(const char *title, const std::vector<double> &t,
                                  const std::vector<double> &a, const std::vector<double> &b,
                                  const std::vector<double> &c, const bool chartNew) {
            if (t.empty() || a.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(t.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kFileChartH))) {
                ImPlot::SetupAxes("时间 t (s)", "值");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                const double xmax = t.back() - t.front();
                fitXToLatest(chartNew, xmax);
                ImPlot::PlotLine("X", t.data(), a.data(), n, lineSpec(kColX));
                ImPlot::PlotLine("Y", t.data(), b.data(), n, lineSpec(kColY));
                ImPlot::PlotLine("Z", t.data(), c.data(), n, lineSpec(kColZ));
                ImPlot::EndPlot();
            }
        }

        inline void drawScaleBars(const char *title, const bool ready, const double v[3], const ImVec4 &fill,
                           const char *fmt = "%.1f") {
            if (ImPlot::BeginPlot(title, ImVec2(-1, kChartHeight))) {
                if (ready) {
                    ImPlot::SetupAxes("轴", "值", ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
                } else {
                    ImPlot::SetupAxes("轴", "值", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, -1.0, 1.0, ImPlotCond_Always);
                }
                ImPlot::SetupAxisLimits(ImAxis_X1, -0.6, 2.6, ImPlotCond_Always);
                constexpr double xs[3] = {0, 1, 2};
                const char *xl[3] = {"X", "Y", "Z"};
                ImPlot::SetupAxisTicks(ImAxis_X1, xs, 3, xl, false);
                if (ready) {
                    ImPlotSpec bs;
                    bs.FillColor = fill;
                    bs.LineColor = ImVec4(0.55f, 0.75f, 1.00f, 1.0f);
                    bs.LineWeight = 1.0f;
                    ImPlot::PlotBars("值", xs, v, 3, 0.5, bs);
                    for (int k = 0; k < 3; ++k) {
                        char buf[32];
                        std::snprintf(buf, sizeof(buf), fmt, v[k]);
                        ImPlot::PlotText(buf, xs[k], v[k]);
                    }
                } else {
                    ImPlot::PlotText("计算中", 1.0, 0.0);
                }
                ImPlot::EndPlot();
            }
        }

        template<class F1, class F2>
        void twoCol(const F1 &left, const F2 &right) {
            ImGui::Columns(2, nullptr, false);
            left();
            ImGui::NextColumn();
            right();
            ImGui::Columns(1);
        }

        inline void vec3Row(const char *label, const double x, const double y, const double z, const char *fmt = "%.3f") {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text(fmt, x);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text(fmt, y);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text(fmt, z);
        }

        inline void steadyStats(const std::vector<AttitudeSample> &s,
                         double &ym, double &ysd, double &pm, double &psd, double &rm, double &rsd) {
            ym = ysd = pm = psd = rm = rsd = 0.0;
            if (s.empty()) return;
            const size_t lo = s.size() * 2 / 3;
            double ny = 0, np = 0, nr = 0;
            for (size_t i = lo; i < s.size(); ++i) {
                ny += s[i].yaw;
                np += s[i].pitch;
                nr += s[i].roll;
            }
            const auto m = static_cast<double>(s.size() - lo);
            if (m <= 0) return;
            ny /= m;
            np /= m;
            nr /= m;
            double vy = 0, vp = 0, vr = 0;
            for (size_t i = lo; i < s.size(); ++i) {
                vy += (s[i].yaw - ny) * (s[i].yaw - ny);
                vp += (s[i].pitch - np) * (s[i].pitch - np);
                vr += (s[i].roll - nr) * (s[i].roll - nr);
            }
            ym = ny * RAD_TO_DEG;
            ysd = std::sqrt(vy / m) * RAD_TO_DEG;
            pm = np * RAD_TO_DEG;
            psd = std::sqrt(vp / m) * RAD_TO_DEG;
            rm = nr * RAD_TO_DEG;
            rsd = std::sqrt(vr / m) * RAD_TO_DEG;
        }
}
