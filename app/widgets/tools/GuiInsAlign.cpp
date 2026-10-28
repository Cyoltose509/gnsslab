#include "GuiInsPanel.h"
#include "GuiInsCommon.h"

#include "GuiHelpers.h"
#include "SolverTask.h"
#include "core/PathMemory.h"
#include "io/AsciiLogReader.h"
#include "io/OEM7Reader.h"
#include "ins/Aligner.h"
#include "core/InsMath.h"
#include "Const.h"
#include "util/StringUtils.h"

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace Tools::InsPanel {
    using namespace insui;

    StaticAlignTask::StaticAlignTask() {
        mode = GnssTask::SolverMode::Ins;
        fileName = "静态解析粗对准";
    }

    void StaticAlignTask::startAlign() {
        {
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.setNum("ins_g", g);
            cfg.setNum("ins_lat", latDeg);
            cfg.setNum("ins_we", earthRate);
            cfg.set("ins_win", std::to_string(win));
        }
        if (alignPath.empty()) {
            setError("请先选择对准段日志文件 (ASCII .ASC)");
            return;
        }
        if (g <= 0.0) {
            setError("重力加速度 g 必须为正数");
            return;
        }
        state = State::Running;
        loading = true;
        if (worker.joinable()) worker.join();
        stop = false;
        hasResult.store(false);
        sampleCount.store(0);
        chartDirty.store(false);
        {
            std::lock_guard lk(resultMutex);
            series.clear();
            plotPts.clear();
        }
        std::shared_ptr<SolverTaskBase> self = shared_from_this();
        worker = std::thread([self] { dynamic_cast<StaticAlignTask *>(self.get())->alignWorker(); });
    }

    void StaticAlignTask::alignWorker() {
        SiteParams sp;
        sp.g = g;
        sp.latDeg = latDeg;
        sp.earthRate = earthRate;
        Aligner::AlignStream st;
        st.init(win, sp);

        // 整段均值用累计和（与 Aligner::solve 的 mean 口径一致，无需另存原始 log）。
        Eigen::Vector3d sumAcc = Eigen::Vector3d::Zero(), sumGyr = Eigen::Vector3d::Zero();
        long long count = 0;
        std::vector<AttitudeSample> seriesLocal;
        seriesLocal.reserve(1 << 16);
        OnlineDecimator<AttitudeSample> dec;

        auto onSample = [&](const ImuSample &s) {
            AttitudeSample a;
            st.step(s, a);
            sumAcc += Eigen::Vector3d(s.ax, s.ay, s.az);
            sumGyr += Eigen::Vector3d(s.gx, s.gy, s.gz);
            ++count;
            if (st.count < static_cast<size_t>(win)) return;
            seriesLocal.push_back(a);
            dec.push(a);
        };
        auto onBatch = [&](const long long n) {
            sampleCount.store(n);
            std::lock_guard lk(resultMutex);
            plotPts = dec.buf;
            chartDirty.store(true);
            if (count > 0) {
                const Eigen::Vector3d accm = sumAcc / static_cast<double>(count);
                const Eigen::Vector3d gyrm = sumGyr / static_cast<double>(count);
                const Eigen::Matrix3d C = alignDCM(-accm, gyrm, sp);
                DCMToEuler(C, att.roll, att.pitch, att.yaw);
                alignQ = Aligner::diagnoseMeans(accm, gyrm, sp);
                alignQValid = true;
            }
        };

        if (endsWithIC(alignPath, ".asc")) {
            AsciiLogReader r;
            if (!r.open(alignPath)) {
                setError("无法打开 ASCII 日志: " + alignPath);
                loading = false;
                state = State::Done;
                return;
            }
            pumpImuReader(r, stop, onSample, onBatch);
        } else {
            OEM7Reader r;
            if (!r.open(alignPath)) {
                setError("无法打开 OEM7 二进制日志: " + alignPath);
                loading = false;
                state = State::Done;
                return;
            }
            pumpImuReader(r, stop, onSample, onBatch);
        }

        if (stop) {
            loading = false;
            state = State::Done;
            return;
        }
        if (seriesLocal.empty()) {
            setError("未在该文件中解析到 IMU 历元: " + alignPath);
            loading = false;
            state = State::Done;
            return;
        }

        Attitude a{};
        AlignQuality aq;
        {
            const Eigen::Vector3d acc = sumAcc / static_cast<double>(count);
            const Eigen::Vector3d gyr = sumGyr / static_cast<double>(count);
            const Eigen::Matrix3d C = alignDCM(-acc, gyr, sp);
            DCMToEuler(C, a.roll, a.pitch, a.yaw);
            aq = Aligner::diagnoseMeans(acc, gyr, sp);
        }
        double ym, ysd, pm, psd, rm, rsd;
        steadyStats(seriesLocal, ym, ysd, pm, psd, rm, rsd);

        {
            std::lock_guard lk(resultMutex);
            att = a;
            alignQ = aq;
            alignQValid = true;
            series = std::move(seriesLocal);
            plotPts = dec.buf;
            steadyY = ym;
            steadyYsd = ysd;
            steadyP = pm;
            steadyPsd = psd;
            steadyR = rm;
            steadyRsd = rsd;
        }
        chartDirty.store(true);
        hasResult.store(true);
        loading = false;
        state = State::Done;
    }

    void StaticAlignTask::renderConfigPanel() {
        Solvers::Helpers::beginConfigBody("静态解析粗对准配置###cfg_", *this);

        if (ImGui::BeginTabBar("##cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                Solvers::Helpers::fileRow("对准段日志", &alignPath, "ins_align", Solvers::Helpers::fIns,
                                          std::size(Solvers::Helpers::fIns), nullptr);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("参数")) {
                ImGui::PushItemWidth(260);
                ImGui::InputDouble("当地重力加速度 g (m/s²)", &g, 0.0, 0.0, "%.6f");
                ImGui::InputDouble("当地纬度 φ (°N)", &latDeg, 0.0, 0.0, "%.6f");
                ImGui::InputDouble("地球自转率 ωe (rad/s)", &earthRate, 0.0, 0.0, "%.12f");
                ImGui::InputInt("滑窗长度 (历元)", &win);
                ImGui::PopItemWidth();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        ImGui::Spacing();
        if (ImGui::Button("开始对准", ImVec2(200, 40))) startAlign();
        ImGui::SameLine();
        Solvers::Helpers::renderCancelButton(this);
        if (errorFlag() && !hasResult)
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", errorMessage().c_str());

        Solvers::Helpers::endConfigBody();
    }

    void StaticAlignTask::renderResult() {
        if (errorFlag()) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "错误: %s", errorMessage().c_str());
            return;
        }
        const bool running = state == State::Running;
        const bool done = hasResult.load();

        if (running) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f),
                               "正在对准 [%s]：已解析 %lld 个历元", fileName.c_str(),
                               static_cast<long long>(sampleCount.load()));
        }

        {
            Attitude m;
            {
                std::lock_guard lk(resultMutex);
                m = att;
            }
            if (ImGui::BeginTable("##align_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("姿态 (deg)");
                ImGui::TableSetupColumn("yaw");
                ImGui::TableSetupColumn("pitch");
                ImGui::TableSetupColumn("roll");
                ImGui::TableHeadersRow();
                vec3Row("整段均值", m.yaw * RAD_TO_DEG, m.pitch * RAD_TO_DEG, m.roll * RAD_TO_DEG, "%.4f");
                if (done) {
                    vec3Row("稳态(后1/3)均值", steadyY, steadyP, steadyR, "%.4f");
                    vec3Row("稳态 1σ", steadyYsd, steadyPsd, steadyRsd, "%.4f");
                } else {
                    for (int r = 0; r < 2; ++r) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(r == 0 ? "稳态(后1/3)均值" : "稳态 1σ");
                        for (int c = 0; c < 3; ++c) {
                            ImGui::TableSetColumnIndex(1 + c);
                            ImGui::TextDisabled("计算中");
                        }
                    }
                }
                ImGui::EndTable();
            }
        }
        ImGui::Spacing();

        {
            AlignQuality aq;
            bool valid = false;
            {
                std::lock_guard lk(resultMutex);
                aq = alignQ;
                valid = alignQValid;
            }
            if (valid && ImGui::BeginTable("##alignq_tbl", 4,
                                           ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                           ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("对准质量");
                ImGui::TableSetupColumn("北");
                ImGui::TableSetupColumn("东");
                ImGui::TableSetupColumn("地");
                ImGui::TableHeadersRow();
                auto qRow = [](const char *label, const double v, const char *fmt) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(label);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text(fmt, v);
                };
                qRow("重力方向残差 (°)", aq.gravityErrDeg, "%.4f");
                qRow("地球自转残差 (rad/s)", aq.earthRateErrRad, "%.9f");
                auto axisRow = [](const char *label, const Eigen::Vector3d &v) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(label);
                    for (int c = 0; c < 3; ++c) {
                        ImGui::TableSetColumnIndex(1 + c);
                        ImGui::Text("%.4f", v(c));
                    }
                };
                axisRow("机体 X 轴 → NED", aq.axisX);
                axisRow("机体 Y 轴 → NED", aq.axisY);
                axisRow("机体 Z 轴 → NED", aq.axisZ);
                ImGui::EndTable();
            }
        }
        ImGui::Spacing();
        if (ImGui::Button("导出逐历元姿态 CSV", ImVec2(220, 0))) exportAlignSeries();
        ImGui::SameLine();
        if (ImGui::Button("导出汇总 CSV", ImVec2(150, 0))) exportAlignSummary();
        ImGui::SameLine();
        if (ImGui::Button("导出每秒均值 CSV", ImVec2(200, 0))) exportAlignPerSec();


        ImGui::Spacing();

        // 抽稀缓冲 → 三条曲线（yaw/pitch/roll 各自相对首样本的历时）
        std::vector<AttitudeSample> pts;
        {
            std::lock_guard lk(resultMutex);
            pts = plotPts; // 抽稀缓冲 ≤~4k
        }
        std::vector<double> px, py, pp, pr;
        if (!pts.empty()) {
            px.reserve(pts.size());
            py.reserve(pts.size());
            pp.reserve(pts.size());
            pr.reserve(pts.size());
            for (const auto &[tow, roll, pitch, yaw]: pts) {
                px.push_back(tow);
                py.push_back(yaw * RAD_TO_DEG);
                pp.push_back(pitch * RAD_TO_DEG);
                pr.push_back(roll * RAD_TO_DEG);
            }
        }
        std::vector<double> pt;
        toElapsed(px, pt); // 横轴：相对首样本的历时（绝对 tow≈1.47e9 会把刻度挤成一串同值）
        const bool chartNew = chartDirty.exchange(false);
        auto drawAtt = [&](const char *title, const std::vector<double> &y, const ImVec4 &col) {
            if (pt.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(pt.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kChartHeight))) {
                ImPlot::SetupAxes("时间 t (s)", "姿态角 (deg)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, pt.back());
                ImPlot::PlotLine("角度", pt.data(), y.data(), n, lineSpec(col));
                ImPlot::EndPlot();
            }
        };
        drawAtt("yaw 随时间", py, kColX);
        drawAtt("pitch 随时间", pp, kColY);
        drawAtt("roll 随时间", pr, kColZ);

        if (running) {
            if (ImGui::Button("停止")) stop = true;
            ImGui::Spacing();
        }


    }

    void StaticAlignTask::exportAlignSeries() {
        const auto s = snapshotSeries();
        if (s.empty()) return;
        exportCsvDialog(*this, "align_series.csv", "align_series.csv", [&](Csv::Writer &w) {
            w.row("tow", "yaw(deg)", "pitch(deg)", "roll(deg)");
            for (const auto &[tow, roll, pitch, yaw]: s)
                w.row(Csv::fx(tow, 3), Csv::fx(yaw * RAD_TO_DEG, 6), Csv::fx(pitch * RAD_TO_DEG, 6),
                      Csv::fx(roll * RAD_TO_DEG, 6));
        });
    }

    void StaticAlignTask::exportAlignSummary() {
        Attitude a{};
        std::vector<AttitudeSample> s;
        double ym = 0, pm = 0, rm = 0;
        {
            std::lock_guard lk(resultMutex);
            a = att;
            s = series;
            ym = steadyY;
            pm = steadyP;
            rm = steadyR;
        }
        if (!hasResult.load() || s.empty()) {
            setError("尚无对准结果，请先点「开始对准」");
            return;
        }
        const auto n = static_cast<double>(s.size());
        double ya = 0, pa = 0, ra = 0;
        for (const auto &p: s) {
            ya += p.yaw;
            pa += p.pitch;
            ra += p.roll;
        }
        ya /= n;
        pa /= n;
        ra /= n;
        double yv = 0, pv = 0, rv = 0;
        for (const auto &p: s) {
            yv += (p.yaw - ya) * (p.yaw - ya);
            pv += (p.pitch - pa) * (p.pitch - pa);
            rv += (p.roll - ra) * (p.roll - ra);
        }
        const double ys = std::sqrt(yv / n), ps = std::sqrt(pv / n), rs = std::sqrt(rv / n);

        exportCsvDialog(*this, "align_summary.csv", "align_summary.csv", [&](Csv::Writer &w) {
            w.row("量", "yaw(deg)", "pitch(deg)", "roll(deg)");
            w.row("整段均值", Csv::fx(a.yaw * RAD_TO_DEG, 4), Csv::fx(a.pitch * RAD_TO_DEG, 4),
                  Csv::fx(a.roll * RAD_TO_DEG, 4));
            w.row("每历元全段均值", Csv::fx(ya * RAD_TO_DEG, 4), Csv::fx(pa * RAD_TO_DEG, 4),
                  Csv::fx(ra * RAD_TO_DEG, 4));
            w.row("每历元全段标准差", Csv::fx(ys * RAD_TO_DEG, 4), Csv::fx(ps * RAD_TO_DEG, 4),
                  Csv::fx(rs * RAD_TO_DEG, 4));
            w.row("每历元稳态(后1/3)均值", Csv::fx(ym, 4), Csv::fx(pm, 4), Csv::fx(rm, 4));
        });
    }

    void StaticAlignTask::exportAlignPerSec() {
        const auto s = snapshotSeries();
        if (s.empty()) return;
        exportCsvDialog(*this, "align_persec.csv", "align_persec.csv", [&](Csv::Writer &w) {
            w.row("sec", "yaw(deg)", "pitch(deg)", "roll(deg)", "n");
            long long sec = 0;
            double sy = 0, sp = 0, sr = 0;
            int n = 0;
            auto flush = [&] {
                if (n <= 0) return;
                w.row(static_cast<long>(sec), Csv::fx(sy / n * RAD_TO_DEG, 6), Csv::fx(sp / n * RAD_TO_DEG, 6),
                      Csv::fx(sr / n * RAD_TO_DEG, 6), n);
                sy = sp = sr = 0;
                n = 0;
            };
            for (const auto &[tow, roll, pitch, yaw]: s) {
                const auto k = static_cast<long long>(std::floor(tow));
                if (n > 0 && k != sec) flush();
                sec = k;
                sy += yaw;
                sp += pitch;
                sr += roll;
                ++n;
            }
            flush();
        });
    }

    std::vector<AttitudeSample> StaticAlignTask::snapshotSeries() {
        std::vector<AttitudeSample> s;
        {
            std::lock_guard lk(resultMutex);
            s = series;
        }
        if (s.empty()) setError("尚无对准序列，请先点「开始对准」");
        return s;
    }
}
