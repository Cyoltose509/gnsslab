#include "Log.h"
#include "GuiInsPanel.h"
#include "GuiInsCommon.h"

#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"
#include "solvers/common/Core.h"
#include "io/novatel/AsciiReader.h"
#include "io/novatel/LogReader.h"
#include "solver/ins/Aligner.h"
#include "core/ins/InsMath.h"
#include "core/Const.h"
#include "StringUtils.h"

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
#include "i18n/I18n.h"

namespace Tools::Ins {

    StaticAlignTask::StaticAlignTask() {
        fileName = "static-align";
        nameKey = "gui.menu.tools.ins_align";
    }

    void StaticAlignTask::startAlign() {
        saveInsParams(par);
        if (alignPath.empty()) {
            setError(I18n::tr("ins.align.err.no_file"));
            return;
        }


        keepManualScaleFor(scaleSf, alignPath + "\n" + std::to_string(par.g));
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
        sp.latDeg = par.latDeg;
        sp.g = par.g;
        ScaleIdentifier sen(scaleSf);
        Aligner::AlignStream st;
        st.init(par.win, sp);

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
            sen.probeSample(s);
            ++count;
            if (st.count < static_cast<std::size_t>(par.win)) return;
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
                att = alignDCM(-accm, gyrm, sp).toEuler();
                alignQ = Aligner::diagnoseMeans(accm, gyrm, sp, sen.res.currentScale());
                alignQValid = true;
            }
        };


        auto onReader = [&](NovAtel::LogReader &r) -> bool {
            return sen.observe(r, [&](const std::string &label, const ScaleNotice &n) {
                std::lock_guard lk(resultMutex);
                sensorModel = label;
                scaleNotice = n;
            });
        };
        if (!pumpImuFile(alignPath, stop, onReader, onSample, onBatch)) {
            abortRun(*this, sen.failureMessage(alignPath));
            return;
        }

        if (stoppedAndEnd(*this)) return;
        if (seriesLocal.empty()) {
            abortRun(*this, I18n::tr("ins.align.err.no_epoch", alignPath));
            return;
        }
        sen.offerProbe();
        sen.resolve(par.g);
        {
            std::lock_guard lk(resultMutex);
            sensorModel = sen.res.describe();
            scaleNotice = sen.res.notice(true);
        }

        Euler a{};
        AlignQuality aq;
        {
            const Eigen::Vector3d acc = sumAcc / static_cast<double>(count);
            const Eigen::Vector3d gyr = sumGyr / static_cast<double>(count);
            a = alignDCM(-acc, gyr, sp).toEuler();
            aq = Aligner::diagnoseMeans(acc, gyr, sp, sen.res.scale);
        }
        double ym, ysd, pm, psd, rm, rsd;
        steadyStats(seriesLocal, ym, ysd, pm, psd, rm, rsd);

        {
            char ang[128];
            std::snprintf(ang, sizeof ang, "yaw=%.4f° pitch=%.4f° roll=%.4f°",
                          a.yaw * RAD_TO_DEG, a.pitch * RAD_TO_DEG, a.roll * RAD_TO_DEG);
            LOG_INFO << "[" << fileName << "] " << I18n::tr("ins.align.log.done", seriesLocal.size(),
                                                           sen.res.describe(), std::string(ang),
                                                           aq.hasScale ? I18n::trc("ins.align.scale_ok")
                                                                       : I18n::trc("ins.align.na"));
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
        renderInsConfig("ins.align.window_title", *this, "ins.align.start",
                        [&] {
                            Solvers::Helpers::fileRow(I18n::trc("ins.align.file_row"), &alignPath, "ins_align",
                                                      FileDialog::fIns(), nullptr);
                        },
                        [&] {
                            renderInsParams(par);
                        },
                        [this] { startAlign(); });
    }

    void StaticAlignTask::renderResult() {
        const bool running = state == State::Running;
        const bool done = hasResult.load();
        if (renderResultPrologue(*this, true, [this] { startAlign(); })) return;

        if (running) {
            ImGui::TextColored(kColInfo, "%s",
                               I18n::tr("ins.align.aligning", displayName(), sampleCount.load()).c_str());
        }

        {
            Euler m;
            {
                std::lock_guard lk(resultMutex);
                m = att;
            }
            ImGui::SeparatorText(I18n::trc("ins.align.att_deg"));
            if (ImGui::BeginTable("##align_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn(I18n::trc("ins.align.col.stat"));
                ImGui::TableSetupColumn("yaw");
                ImGui::TableSetupColumn("pitch");
                ImGui::TableSetupColumn("roll");
                ImGui::TableHeadersRow();

                auto pendingRow = [](const char *label, const char *pend) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(I18n::trc(label));
                    for (int c = 0; c < 3; ++c) {
                        ImGui::TableSetColumnIndex(1 + c);
                        ImGui::TextDisabled("%s", pend);
                    }
                };
                const char *pend = running ? I18n::trc("ins.chart.computing") : "—";
                if (running || done) {
                    vec3Row("ins.align.row.mean", m.yaw * RAD_TO_DEG, m.pitch * RAD_TO_DEG,
                            m.roll * RAD_TO_DEG, "%.4f");
                } else {
                    pendingRow("ins.align.row.mean", "—");
                }
                if (done) {
                    vec3Row("ins.align.row.steady_mean", steadyY, steadyP, steadyR, "%.4f");
                    vec3Row("ins.align.row.steady_sigma", steadyYsd, steadyPsd, steadyRsd,
                            "%.4f");
                } else {
                    pendingRow("ins.align.row.steady_mean", pend);
                    pendingRow("ins.align.row.steady_sigma", pend);
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
                ImGui::SeparatorText(I18n::trc("ins.align.sep.dcm"));
                if (ImGui::BeginTable("##alignaxis_tbl", 4,
                                      ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                    ImGui::TableSetupColumn(I18n::trc("ins.align.col.body_axis"));
                    ImGui::TableSetupColumn(I18n::trc("ins.align.col.north"));
                    ImGui::TableSetupColumn(I18n::trc("ins.align.col.east"));
                    ImGui::TableSetupColumn(I18n::trc("ins.align.col.down"));
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

                ImGui::SeparatorText(I18n::trc("ins.align.sep.quality"));
                if (ImGui::BeginTable("##alignq_tbl", 4,
                                      ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                    ImGui::TableSetupColumn(I18n::trc("ins.calib.col.check"));
                    ImGui::TableSetupColumn(I18n::trc("ins.calib.col.measured"));
                    ImGui::TableSetupColumn(I18n::trc("common.reference"));
                    ImGui::TableSetupColumn(I18n::trc("ins.align.col.verdict_dev"));
                    ImGui::TableHeadersRow();

                    auto row = [](const char *label, const char *meas, const char *ref, const char *note,
                                  const ImVec4 *col) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(I18n::trc(label));
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
                    row("ins.align.row.dcm_det", b1, "+1",
                        orthoOk ? I18n::trc("ins.align.note.ortho")
                                : I18n::trc("ins.align.note.not_ortho"), cOrtho);

                    std::snprintf(b2, sizeof b2, "%.3e", aq.dcmOrthoErr);
                    row("ins.align.row.ortho_err", b2, "0",
                        orthoOk ? I18n::trc("ins.align.note.orthonormalized")
                                : I18n::trc("ins.align.note.not_orthonormalized"), cOrtho);

                    std::snprintf(b3, sizeof b3, "%.4f", aq.earthRateErrDeg);
                    row("ins.align.row.earth_rate", b3, "0", "", nullptr);

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

                        const char *pend = done ? I18n::trc("ins.align.na")
                                                : I18n::trc("ins.align.pending");
                        row("|f| (m/s²)", pend, "g(φ)", "—", nullptr);
                        row("|ω| (rad/s)", pend, "Ω", "—", nullptr);
                    }

                    if (aq.latDerivedValid) {
                        char m3[64], r3[64], n3[64];
                        std::snprintf(m3, sizeof m3, "%.4f", aq.latDerivedDeg);
                        std::snprintf(r3, sizeof r3, "φ = %.4f", par.latDeg);
                        std::snprintf(n3, sizeof n3, "%+.4f °", aq.latDerivedDeg - par.latDeg);
                        row("ins.align.row.lat", m3, r3, n3, nullptr);
                    } else {
                        row("ins.align.row.lat", I18n::trc("ins.align.na"), "φ",
                            I18n::trc("ins.align.note.degenerate"), nullptr);
                    }
                    ImGui::EndTable();
                }
            }
        }
        ImGui::Spacing();
        {

            ImGui::BeginDisabled(!done);
            const bool s1 = ImGui::Button(I18n::trc("ins.align.export.series"), ImVec2(220, 0));
            ImGui::SameLine();
            const bool s2 = ImGui::Button(I18n::trc("ins.align.export.summary"), ImVec2(150, 0));
            ImGui::SameLine();
            const bool s3 = ImGui::Button(I18n::trc("ins.align.export.persec"), ImVec2(200, 0));
            const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
            ImGui::EndDisabled();
            if (!done && hovered) ImGui::SetTooltip("%s", I18n::trc("ins.align.export_hint"));
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
        auto drawATT = [&](const char *title, const std::vector<double> &y, const ImVec4 &col) {
            if (pt.empty()) {
                ImGui::TextDisabled("%s", I18n::trc("ins.no_data"));
                return;
            }
            const int n = static_cast<int>(pt.size());
            if (ImPlot::BeginPlot(I18n::trc(title), ImVec2(-1, kChartHeight))) {
                ImPlot::SetupAxes(I18n::trc("ins.chart.x_time"), I18n::trc("ins.align.att_deg"));
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, pt.back());
                ImPlot::PlotLine(I18n::trc("ins.align.chart.legend_angle"), pt.data(), y.data(), n,
                                 lineSpec(col));
                ImPlot::EndPlot();
            }
        };
        drawATT("ins.align.chart.yaw", py, kColX);
        drawATT("ins.align.chart.pitch", pp, kColY);
        drawATT("ins.align.chart.roll", pr, kColZ);

        if (running) {
            if (ImGui::Button(I18n::trc("ins.calib.stop"))) stop = true;
            ImGui::Spacing();
        }


    }

    void StaticAlignTask::exportAlignSeries() {
        const auto s = snapshotSeries();
        if (s.empty()) return;
        exportCSVDialog(*this, "align_series.csv", "align_series.csv", [&](CSV::Writer &w) {
            w.row("tow", "yaw(deg)", "pitch(deg)", "roll(deg)");
            for (const auto &[tow, roll, pitch, yaw]: s)
                w.row(CSV::fx(tow, 3), CSV::fx(yaw * RAD_TO_DEG, 6), CSV::fx(pitch * RAD_TO_DEG, 6),
                      CSV::fx(roll * RAD_TO_DEG, 6));
        });
    }

    void StaticAlignTask::exportAlignSummary() {
        const auto s = snapshotSeries();
        if (s.empty()) return;
        Euler a{};
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

        exportCSVDialog(*this, "align_summary.csv", "align_summary.csv", [&](CSV::Writer &w) {
            w.row(I18n::trc("ins.calib.col.qty"), "yaw(deg)", "pitch(deg)", "roll(deg)");
            w.row(I18n::trc("ins.align.row.mean"), CSV::fx(a.yaw * RAD_TO_DEG, 4), CSV::fx(a.pitch * RAD_TO_DEG, 4),
                  CSV::fx(a.roll * RAD_TO_DEG, 4));
            w.row(I18n::trc("ins.align.csv.mean_epoch"), CSV::fx(ya * RAD_TO_DEG, 4), CSV::fx(pa * RAD_TO_DEG, 4),
                  CSV::fx(ra * RAD_TO_DEG, 4));
            w.row(I18n::trc("ins.align.csv.std_epoch"), CSV::fx(ys * RAD_TO_DEG, 4), CSV::fx(ps * RAD_TO_DEG, 4),
                  CSV::fx(rs * RAD_TO_DEG, 4));
            w.row(I18n::trc("ins.align.csv.mean_steady"), CSV::fx(ym, 4), CSV::fx(pm, 4), CSV::fx(rm, 4));

            w.blank();
            w.row(I18n::trc("ins.align.sep.quality"), I18n::trc("ins.calib.col.measured"),
                  I18n::trc("common.reference"), I18n::trc("ins.align.col.verdict_dev"));
            const bool orthoOk = std::fabs(aq.dcmDet - 1.0) < 1e-9 && aq.dcmOrthoErr < 1e-9;
            w.row(I18n::trc("ins.align.row.dcm_det"), CSV::fx(aq.dcmDet, 12), "+1",
                  orthoOk ? I18n::trc("ins.align.note.ortho") : I18n::trc("ins.align.note.not_ortho"));
            w.row(I18n::trc("ins.align.csv.ortho_err"), CSV::sci(aq.dcmOrthoErr, 3), "0",
                  orthoOk ? I18n::trc("ins.align.note.orthonormalized") : I18n::trc("ins.align.note.not_orthonormalized"));
            w.row(I18n::trc("ins.align.csv.earth_rate_angle"), CSV::fx(aq.earthRateErrDeg, 4), "0", "");
            if (aq.hasScale) {
                char ref1[64], dev1[64], ref2[64], dev2[64];
                std::snprintf(ref1, sizeof ref1, "g = %.6f", aq.gravityRef);
                std::snprintf(dev1, sizeof dev1, "%+.3f %%", 100.0 * (aq.accelNorm / aq.gravityRef - 1.0));
                std::snprintf(ref2, sizeof ref2, "Ω = %.9f", aq.gyroRef);
                std::snprintf(dev2, sizeof dev2, "%+.3f %%", 100.0 * (aq.gyroNorm / aq.gyroRef - 1.0));
                w.row("|f| (m/s²)", CSV::fx(aq.accelNorm, 6), ref1, dev1);
                w.row("|ω| (rad/s)", CSV::fx(aq.gyroNorm, 9), ref2, dev2);
            } else {
                w.row("|f| (m/s²)", "-", "g", "-");
                w.row("|ω| (rad/s)", "-", "Ω", "-");
            }
            if (aq.latDerivedValid) {
                char ref3[64], dev3[64];
                std::snprintf(ref3, sizeof ref3, "φ = %.4f", par.latDeg);
                std::snprintf(dev3, sizeof dev3, "%+.4f °", aq.latDerivedDeg - par.latDeg);
                w.row(I18n::trc("ins.align.csv.derived_lat"), CSV::fx(aq.latDerivedDeg, 4), ref3, dev3);
            } else {
                w.row(I18n::trc("ins.align.csv.derived_lat"), I18n::trc("ins.align.na"), "φ",
                      I18n::trc("ins.align.note.degenerate"));
            }
            if (!model.empty()) w.row(I18n::trc("IMUModel"), model);
        });
    }

    void StaticAlignTask::exportAlignPerSec() {
        const auto s = snapshotSeries();
        if (s.empty()) return;
        exportCSVDialog(*this, "align_persec.csv", "align_persec.csv", [&](CSV::Writer &w) {
            w.row("sec", "yaw(deg)", "pitch(deg)", "roll(deg)", "n");
            long long sec = 0;
            double sy = 0, sp = 0, sr = 0;
            int n = 0;
            auto flush = [&] {
                if (n <= 0) return;
                w.row(static_cast<long>(sec), CSV::fx(sy / n * RAD_TO_DEG, 6), CSV::fx(sp / n * RAD_TO_DEG, 6),
                      CSV::fx(sr / n * RAD_TO_DEG, 6), n);
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
        if (s.empty()) setError(I18n::tr("ins.align.err.no_series"));
        return s;
    }
}
