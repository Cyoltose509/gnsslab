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
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace Tools::InsPanel {

    StaticAlignTask::StaticAlignTask() {
        mode = GnssTask::SolverMode::Ins;
        fileName = "静态解析粗对准";
    }

    void StaticAlignTask::startAlign() {
        {
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.setNum("ins_lat", latDeg);
            cfg.setNum("ins_g", g);
            cfg.set("ins_win", std::to_string(win));
        }
        if (alignPath.empty()) {
            setError("请先选择对准段日志文件（ASCII .ASC）。");
            return;
        }
        if (!std::isfinite(latDeg) || std::fabs(latDeg) > 90.0) {
            setError("当地纬度 φ 超出 [-90, 90] 的范围。");
            return;
        }
        if (!std::isfinite(g) || g <= 0.0) {
            setError("当地重力加速度 g 必须为正数。");
            return;
        }
        // 手填当量只对同一份数据有效：换文件或换 g 就作废
        keepManualScaleFor(scaleSf, alignPath + "\n" + std::to_string(g));
        beginRun(*this);
        {
            std::lock_guard lk(resultMutex);
            series.clear();
            plotPts.clear();
            sensorModel.clear();
            scaleNotice = ScaleNotice{};
        }
        launchWorker(*this, [this] { alignWorker(); });
    }

    void StaticAlignTask::alignWorker() {
        SiteParams sp;
        sp.latDeg = latDeg;
        sp.g = g;
        ScaleIdentifier sen(scaleSf); // 当量识别整条流水线（GuiInsCommon.h），与标定同一个对象
        Aligner::AlignStream st;
        st.init(win, sp);

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
            sen.probeSample(s); // 型号缺失时靠它反推当量
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
                alignQ = Aligner::diagnoseMeans(accm, gyrm, sp, sen.currentScale());
                alignQValid = true;
            }
        };

        // 记录自带的型号读到就能显示（不必等整份文件读完）；读不到要等末尾 resolve 才定得下来
        auto onReader = [&](auto &r) -> bool {
            return sen.observe(r, [&](const std::string &label, const ScaleNotice &n) {
                std::lock_guard lk(resultMutex);
                sensorModel = label;
                scaleNotice = n;
            });
        };
        if (!pumpImuFile(alignPath, stop, onReader, onSample, onBatch)) {
            abortRun(*this, sen.openFailure(alignPath));
            return;
        }

        if (stoppedAndEnd(*this)) return;
        if (seriesLocal.empty()) {
            abortRun(*this, "未在该文件中解析到任何 IMU 历元：" + alignPath);
            return;
        }
        sen.offerProbe();
        sen.resolve(g);
        {
            std::lock_guard lk(resultMutex);
            sensorModel = sen.label();
            scaleNotice = sen.notice(true);
        }

        Attitude a{};
        AlignQuality aq;
        {
            const Eigen::Vector3d acc = sumAcc / static_cast<double>(count);
            const Eigen::Vector3d gyr = sumGyr / static_cast<double>(count);
            const Eigen::Matrix3d C = alignDCM(-acc, gyr, sp);
            DCMToEuler(C, a.roll, a.pitch, a.yaw);
            aq = Aligner::diagnoseMeans(acc, gyr, sp, sen.scale());
        }
        double ym, ysd, pm, psd, rm, rsd;
        steadyStats(seriesLocal, ym, ysd, pm, psd, rm, rsd);

        {
            char ang[128];
            std::snprintf(ang, sizeof ang, "yaw=%.4f° pitch=%.4f° roll=%.4f°",
                          a.yaw * RAD_TO_DEG, a.pitch * RAD_TO_DEG, a.roll * RAD_TO_DEG);
            LOG_INFO << "[静态解析粗对准] 完成：" << seriesLocal.size() << " 个历元，当量 "
                     << sen.label() << "，" << ang << "，模长校验" << (aq.hasScale ? "可用" : "不可用");
        }
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
        finishRun(*this);
    }

    void StaticAlignTask::renderConfigPanel() {
        renderInsConfig("静态解析粗对准配置###cfg_", *this, "开始对准",
                        [&] {
                            Solvers::Helpers::fileRow("对准段日志", &alignPath, "ins_align", Solvers::Helpers::fIns,
                                                      std::size(Solvers::Helpers::fIns), nullptr);
                        },
                        [&] {
                            ImGui::InputDouble("当地纬度 φ (°N)", &latDeg, 0.0, 0.0, "%.6f");
                            ImGui::InputDouble("当地重力加速度 g (m/s²)", &g, 0.0, 0.0, "%.6f");
                            ImGui::InputInt("滑窗长度 (历元)", &win);
                        },
                        [this] { startAlign(); });
    }

    void StaticAlignTask::renderResult() {
        const bool running = state == State::Running;
        const bool done = hasResult.load();
        if (renderResultPrologue(*this, true, [this] { startAlign(); })) return;

        if (running) {
            ImGui::TextColored(kColInfo, "正在对准 %s：已解析 %lld 个历元", fileName.c_str(),
                               static_cast<long long>(sampleCount.load()));
        }

        {
            Attitude m;
            {
                std::lock_guard lk(resultMutex);
                m = att;
            }
            ImGui::SeparatorText("姿态角 (°)");
            if (ImGui::BeginTable("##align_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("统计量");
                ImGui::TableSetupColumn("yaw");
                ImGui::TableSetupColumn("pitch");
                ImGui::TableSetupColumn("roll");
                ImGui::TableHeadersRow();
                // 没跑过就该是「没有」，不是三个 0.0000 冒充姿态结果
                auto pendingRow = [](const char *label, const char *pend) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(label);
                    for (int c = 0; c < 3; ++c) {
                        ImGui::TableSetColumnIndex(1 + c);
                        ImGui::TextDisabled("%s", pend);
                    }
                };
                const char *pend = running ? "计算中" : "—";
                if (running || done) {
                    vec3Row("整段均值", m.yaw * RAD_TO_DEG, m.pitch * RAD_TO_DEG, m.roll * RAD_TO_DEG, "%.4f");
                } else {
                    pendingRow("整段均值", "—");
                }
                if (done) {
                    vec3Row("稳态均值（后 1/3）", steadyY, steadyP, steadyR, "%.4f");
                    vec3Row("稳态 1σ", steadyYsd, steadyPsd, steadyRsd, "%.4f");
                } else {
                    pendingRow("稳态均值（后 1/3）", pend);
                    pendingRow("稳态 1σ", pend);
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
            if (valid) {
                ImGui::SeparatorText("姿态矩阵 C（机体轴 → NED）");
                if (ImGui::BeginTable("##alignaxis_tbl", 4,
                                      ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                    ImGui::TableSetupColumn("机体轴");
                    ImGui::TableSetupColumn("北");
                    ImGui::TableSetupColumn("东");
                    ImGui::TableSetupColumn("地");
                    ImGui::TableHeadersRow();
                    auto axisRow = [](const char *label, const Eigen::Vector3d &v) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(label);
                        for (int c = 0; c < 3; ++c) {
                            ImGui::TableSetColumnIndex(1 + c);
                            ImGui::Text("%.4f", v(c));
                        }
                    };
                    axisRow("X", aq.axisX);
                    axisRow("Y", aq.axisY);
                    axisRow("Z", aq.axisZ);
                    ImGui::EndTable();
                }
                ImGui::Spacing();

                ImGui::SeparatorText("对准质量");
                if (ImGui::BeginTable("##alignq_tbl", 4,
                                      ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                    ImGui::TableSetupColumn("检查项");
                    ImGui::TableSetupColumn("实测");
                    ImGui::TableSetupColumn("参考");
                    ImGui::TableSetupColumn("判定/偏差");
                    ImGui::TableHeadersRow();

                    auto row = [](const char *label, const char *meas, const char *ref, const char *note,
                                  const ImVec4 *col) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(label);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted(meas);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::TextUnformatted(ref);
                        ImGui::TableSetColumnIndex(3);
                        if (col) ImGui::TextColored(*col, "%s", note);
                        else ImGui::TextUnformatted(note);
                    };

                    const bool orthoOk = std::fabs(aq.dcmDet - 1.0) < 1e-9 && aq.dcmOrthoErr < 1e-9;
                    const ImVec4 *cOrtho = orthoOk ? &kColPass : &kColFail;

                    char b1[64], b2[64], b3[64];
                    std::snprintf(b1, sizeof b1, "%.12f", aq.dcmDet);
                    row("姿态矩阵 det(C)", b1, "+1", orthoOk ? "单位正交" : "非单位正交", cOrtho);

                    std::snprintf(b2, sizeof b2, "%.3e", aq.dcmOrthoErr);
                    row("正交误差 max|(C^T)C−I|", b2, "0", orthoOk ? "已正交化" : "未正交化", cOrtho);

                    std::snprintf(b3, sizeof b3, "%.4f", aq.earthRateErrDeg);
                    row("ω 与 ω_ie^n 夹角 (°)", b3, "0", "", nullptr);

                    if (aq.hasScale) {
                        char m1[64], r1[64], n1[64];
                        std::snprintf(m1, sizeof m1, "%.6f", aq.accelNorm);
                        std::snprintf(r1, sizeof r1, "g = %.6f", aq.gravityRef);
                        std::snprintf(n1, sizeof n1, "%+.3f %%", 100.0 * (aq.accelNorm / aq.gravityRef - 1.0));
                        row("|f| (m/s²)", m1, r1, n1, nullptr);

                        char m2[64], r2[64], n2[64];
                        std::snprintf(m2, sizeof m2, "%.9f", aq.gyroNorm);
                        std::snprintf(r2, sizeof r2, "Ω = %.9f", aq.gyroRef);
                        std::snprintf(n2, sizeof n2, "%+.3f %%", 100.0 * (aq.gyroNorm / aq.gyroRef - 1.0));
                        row("|ω| (rad/s)", m2, r2, n2, nullptr);
                    } else {
                        // 当量还没定下来（流式阶段）与确实拿不到是两件事，别混成同一句「—」
                        const char *pend = done ? "不可用" : "待定";
                        row("|f| (m/s²)", pend, "g(φ)", "—", nullptr);
                        row("|ω| (rad/s)", pend, "Ω", "—", nullptr);
                    }

                    if (aq.latDerivedValid) {
                        char m3[64], r3[64], n3[64];
                        std::snprintf(m3, sizeof m3, "%.4f", aq.latDerivedDeg);
                        std::snprintf(r3, sizeof r3, "φ = %.4f", latDeg);
                        std::snprintf(n3, sizeof n3, "%+.4f °", aq.latDerivedDeg - latDeg);
                        row("反解纬度 φ (°)", m3, r3, n3, nullptr);
                    } else {
                        row("反解纬度 φ (°)", "不可用", "φ", "重力矢量退化", nullptr);
                    }
                    ImGui::EndTable();
                }
            }
        }
        ImGui::Spacing();
        {
            // 三个导出都要整段序列，读完才有：跑的过程中按钮直接禁掉，而不是点了再报错
            ImGui::BeginDisabled(!done);
            const bool s1 = ImGui::Button("导出逐历元姿态 CSV", ImVec2(220, 0));
            ImGui::SameLine();
            const bool s2 = ImGui::Button("导出汇总 CSV", ImVec2(150, 0));
            ImGui::SameLine();
            const bool s3 = ImGui::Button("导出每秒均值 CSV", ImVec2(200, 0));
            const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
            ImGui::EndDisabled();
            if (!done && hovered) ImGui::SetTooltip("对准完成后才能导出。");
            if (s1) exportAlignSeries();
            if (s2) exportAlignSummary();
            if (s3) exportAlignPerSec();
        }


        ImGui::Spacing();

        std::vector<AttitudeSample> pts;
        {
            std::lock_guard lk(resultMutex);
            pts = plotPts;
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
        toElapsed(px, pt);
        const bool chartNew = chartDirty.exchange(false);
        auto drawAtt = [&](const char *title, const std::vector<double> &y, const ImVec4 &col) {
            if (pt.empty()) {
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(pt.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kChartHeight))) {
                ImPlot::SetupAxes("时间 t (s)", "姿态角 (°)");
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
        const auto s = snapshotSeries();
        if (s.empty()) return;
        Attitude a{};
        double ym = 0, pm = 0, rm = 0;
        AlignQuality aq;
        std::string model;
        {
            std::lock_guard lk(resultMutex);
            a = att;
            ym = steadyY;
            pm = steadyP;
            rm = steadyR;
            aq = alignQ;
            model = sensorModel;
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

            w.endRow();
            w.row("对准质量", "实测", "参考", "判定/偏差");
            const bool orthoOk = std::fabs(aq.dcmDet - 1.0) < 1e-9 && aq.dcmOrthoErr < 1e-9;
            w.row("姿态矩阵 det(C)", Csv::fx(aq.dcmDet, 12), "+1", orthoOk ? "单位正交" : "非单位正交");
            w.row("正交误差 max|C^T C-I|", Csv::sci(aq.dcmOrthoErr, 3), "0",
                  orthoOk ? "已正交化" : "未正交化");
            w.row("ω 与 ω_ie^n 夹角 (deg)", Csv::fx(aq.earthRateErrDeg, 4), "0", "");
            if (aq.hasScale) {
                char ref1[64], dev1[64], ref2[64], dev2[64];
                std::snprintf(ref1, sizeof ref1, "g = %.6f", aq.gravityRef);
                std::snprintf(dev1, sizeof dev1, "%+.3f %%", 100.0 * (aq.accelNorm / aq.gravityRef - 1.0));
                std::snprintf(ref2, sizeof ref2, "Ω = %.9f", aq.gyroRef);
                std::snprintf(dev2, sizeof dev2, "%+.3f %%", 100.0 * (aq.gyroNorm / aq.gyroRef - 1.0));
                w.row("|f| (m/s²)", Csv::fx(aq.accelNorm, 6), ref1, dev1);
                w.row("|ω| (rad/s)", Csv::fx(aq.gyroNorm, 9), ref2, dev2);
            } else {
                w.row("|f| (m/s²)", "-", "g", "-");
                w.row("|ω| (rad/s)", "-", "Ω", "-");
            }
            if (aq.latDerivedValid) {
                char ref3[64], dev3[64];
                std::snprintf(ref3, sizeof ref3, "φ = %.4f", latDeg);
                std::snprintf(dev3, sizeof dev3, "%+.4f °", aq.latDerivedDeg - latDeg);
                w.row("反解纬度 φ̂ (deg)", Csv::fx(aq.latDerivedDeg, 4), ref3, dev3);
            } else {
                w.row("反解纬度 φ̂ (deg)", "不可用", "φ", "重力矢量退化");
            }
            if (!model.empty()) w.row("IMU 型号", model);
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
        if (s.empty()) setError("尚无对准序列，无法导出。请先完成一次对准。");
        return s;
    }
}
