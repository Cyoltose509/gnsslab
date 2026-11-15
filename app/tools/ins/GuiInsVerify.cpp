#include "Log.h"
#include "GuiInsPanel.h"
#include "GuiInsCommon.h"

#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"
#include "solvers/common/Core.h"
#include "core/Const.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>
#include "i18n/I18n.h"

namespace Tools::Ins {

    InsVerifyTask::InsVerifyTask() {
        fileName = "ins-verify";
        nameKey = "gui.menu.tools.ins_verify";
    }

    void InsVerifyTask::startVerify() {
        saveInsParams(par);
        if (imuPath.empty()) {
            setError(I18n::tr("ins.verify.err.no_imu"));
            return;
        }
        if (refPath.empty()) {
            setError(I18n::tr("ins.verify.err.no_ref"));
            return;
        }
        beginRun(*this);
        {
            std::lock_guard lk(resultMutex);
            report = ::Ins::VerifyReport{};
            reportValid = false;
        }
        launchWorker(*this, [this] { verifyWorker(); });
    }

    void InsVerifyTask::verifyWorker() {
        ::Ins::VerifyOptions opt;
        opt.startTime = par.startTime;
        opt.init.llh = Eigen::Vector3d(par.latDeg * DEG_TO_RAD, par.lonDeg * DEG_TO_RAD, par.hM);
        opt.init.vned = Eigen::Vector3d(par.vN, par.vE, par.vD);
        opt.init.rollDeg = par.rollDeg;
        opt.init.pitchDeg = par.pitchDeg;
        opt.init.headingDeg = par.headingDeg;
        ::Ins::Verifier v(imuPath, refPath, opt);
        if (!v.ready()) {
            abortRun(*this, v.error());
            return;
        }

        // 循环留在界面侧，和标定/对准的 IMU 泵一样：逐条推进、边算边把报告发布出去，
        // 结果页每帧读 report 快照实时画图。每出 2048 个比对点或首次出点就发布一次。
        std::size_t lastPub = 0;
        bool published = false;
        while (!stop.load() && v.step()) {
            const std::size_t c = v.report.compared;
            sampleCount.store(static_cast<long long>(c));
            if (!published || c - lastPub >= 2048) {
                lastPub = c;
                published = true;
                {
                    std::lock_guard lk(resultMutex);
                    report = v.report;
                    reportValid = true;
                }
                chartDirty.store(true);
            }
        }
        if (!v.error().empty()) {
            abortRun(*this, v.error());
            return;
        }
        if (stop.load()) {
            endRun(*this);
            return;
        }
        v.finish();
        if (v.report.compared == 0) {
            abortRun(*this, I18n::tr("ins.verify.err.no_overlap"));
            return;
        }
        // 末批可能不足 2048 点，收尾再发一次完整报告（含 pass 判定）。
        // 只发一次：report = move(v.report) 会把 series（vector）所有权转移走，
        // 再 move 一次会把已掏空的 v.report 覆盖回来，导致曲线 series 变空。
        {
            std::lock_guard lk(resultMutex);
            report = std::move(v.report);
            reportValid = true;
        }
        chartDirty.store(true);

        char dLat[32], dLon[32], dh[32], dV[32];
        std::snprintf(dLat, sizeof dLat, "%.2e", report.maxLatDeg);
        std::snprintf(dLon, sizeof dLon, "%.2e", report.maxLonDeg);
        std::snprintf(dh, sizeof dh, "%.4f", report.maxHM);
        std::snprintf(dV, sizeof dV, "%.2e", report.maxV);
        LOG_INFO << "[" << fileName << "] "
                 << I18n::tr("ins.verify.log.summary", report.compared, report.imuEpochs, dLat, dLon,
                             dh, dV, report.pass ? I18n::tr("ins.verify.within_tol")
                                                 : I18n::tr("ins.verify.over_tol"));

        finishRun(*this);
    }

    void InsVerifyTask::renderConfigPanel() {
        renderInsConfig("ins.verify.window_title", *this, "ins.verify.start",
                        [&] {
                            Solvers::Helpers::fileRow(I18n::trc("ins.verify.file_imu"), &imuPath,
                                                      "ins_verify_imu", FileDialog::fBin(), nullptr);
                            Solvers::Helpers::fileRow(I18n::trc("ins.verify.file_ref"), &refPath,
                                                      "ins_verify_ref", FileDialog::fBin(), nullptr);
                            ImGui::Spacing();
                            ImGui::TextDisabled("%s", I18n::trc("ins.verify.hint.imu"));
                            ImGui::TextDisabled("%s", I18n::trc("ins.verify.hint.ref"));
                        },
                        [&] { renderInsParams(par); }, [this] { startVerify(); });
    }

    void InsVerifyTask::renderResult() {
        const bool running = state == State::Running;
        const bool done = hasResult.load();
        if (drawResultError(*this)) {
            ImGui::Spacing();
            return;
        }

        if (running) {
            ImGui::TextColored(kColInfo, "%s",
                               I18n::tr("ins.verify.comparing", displayName(), sampleCount.load()).c_str());
            ImGui::SameLine();
            if (ImGui::Button(I18n::trc("ins.verify.stop"))) stop = true;
        }

        ::Ins::VerifyReport rep;
        bool valid = false;
        {
            std::lock_guard lk(resultMutex);
            rep = report;
            valid = reportValid;
        }

        // 进行中只显示实时进度，不下结论（rep.pass 由 finish() 在结束时才置位）；
        // 完成后才显示 pass/fail。图与表只要 valid 就画（与标定/对准一致，边算边画）。
        if (running && valid) {
            char hm[32], vv[32];
            std::snprintf(hm, sizeof hm, "%.4f", rep.maxHorizontalM);
            std::snprintf(vv, sizeof vv, "%.2e", rep.maxV);
            ImGui::TextColored(kColInfo, "%s",
                               I18n::tr("ins.verify.progress", rep.compared, hm, vv).c_str());
        } else if (valid) {
            std::string bad;
            auto mark = [&bad](const bool over, const char *name) {
                if (!over) return;
                if (!bad.empty()) bad += "、";
                bad += name;
            };
            mark(rep.maxLatDeg > rep.tol.latLonDeg, I18n::trc("Latitude"));
            mark(rep.maxLonDeg > rep.tol.latLonDeg, I18n::trc("Longitude"));
            mark(rep.maxHM > rep.tol.hM, I18n::trc("Height"));
            mark(rep.maxV > rep.tol.vMps, I18n::trc("Velocity"));
            mark(rep.maxRollDeg > rep.tol.attDeg, I18n::trc("Roll"));
            mark(rep.maxPitchDeg > rep.tol.attDeg, I18n::trc("Pitch"));
            mark(rep.maxHeadingDeg > rep.tol.attDeg, I18n::trc("Heading"));
            if (bad.empty())
                ImGui::TextColored(UI::kColPass, "%s", I18n::trc("ins.verify.all_within_tol"));
            else ImGui::TextColored(UI::kColFail, "%s",
                                    I18n::tr("ins.verify.over_items", bad).c_str());
        }

        if (!valid) {
            ImGui::TextDisabled("%s", I18n::trc("ins.verify.no_result"));
            return;
        }

        ImGui::SeparatorText(I18n::trc("ins.verify.sep.data"));
        if (ImGui::BeginTable("##verify_sum", 4,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn(I18n::trc("ins.verify.col.item"));
            ImGui::TableSetupColumn(I18n::trc("ins.verify.col.value"));
            ImGui::TableSetupColumn(I18n::trc("ins.verify.col.item"));
            ImGui::TableSetupColumn(I18n::trc("ins.verify.col.value"));
            ImGui::TableHeadersRow();

            auto kv = [](const char *k1, const char *v1, const char *k2, const char *v2) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(I18n::trc(k1));
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(v1);
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(I18n::trc(k2));
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(v2);
            };

            char b1[64], b2[64], b3[64], b4[64], b5[64], b6[64];
            std::snprintf(b1, sizeof b1, "%zu", rep.imuEpochs);
            std::snprintf(b2, sizeof b2, "%zu", rep.refEpochs);
            std::snprintf(b3, sizeof b3, "%zu", rep.compared);
            std::snprintf(b4, sizeof b4, "%.3f s", rep.t0);
            std::snprintf(b5, sizeof b5, "%.3f s", rep.t1);
            std::snprintf(b6, sizeof b6, "%.1f min", (rep.t1 - rep.t0) / 60.0);
            kv("ins.verify.kv.imu_epochs", b1, "ins.verify.kv.ref_epochs", b2);
            kv("ins.verify.kv.compared", b3, "ins.verify.kv.start", b4);
            kv("ins.verify.kv.end", b5, "ins.verify.kv.duration", b6);
            ImGui::EndTable();
        }
        ImGui::Spacing();

        ImGui::SeparatorText(I18n::trc("ins.verify.sep.max_dev"));
        if (ImGui::BeginTable("##verify_err", 4,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn(I18n::trc("ins.calib.col.qty"));
            ImGui::TableSetupColumn(I18n::trc("ins.verify.col.max_dev"));
            ImGui::TableSetupColumn(I18n::trc("Tolerance"));
            ImGui::TableSetupColumn(I18n::trc("ins.verify.col.verdict"));
            ImGui::TableHeadersRow();

            auto row = [](const char *label, const char *val, const char *tol, const bool over) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(I18n::trc(label));
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(val);
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(tol);
                ImGui::TableSetColumnIndex(3);
                ImGui::TextColored(over ? kColFail : kColPass, "%s",
                                   over ? I18n::trc("ins.verify.over") : I18n::trc("ins.verify.pass"));
            };

            auto sci = [](char *buf, const std::size_t n, const double v) {
                std::snprintf(buf, n, "%.2e", v);
            };
            auto fixed = [](char *buf, const std::size_t n, const double v, const char *fmt) {
                std::snprintf(buf, n, fmt, v);
            };

            char v1[64], t1[64], v2[64], t2[64], v3[64], t3[64], v4[64], t4[64];
            char v5[64], t5[64], v6[64], t6[64], v7[64], t7[64];
            sci(v1, sizeof v1, rep.maxLatDeg);
            sci(t1, sizeof t1, rep.tol.latLonDeg);
            row("ins.verify.row.lat", v1, t1, rep.maxLatDeg > rep.tol.latLonDeg);
            sci(v2, sizeof v2, rep.maxLonDeg);
            sci(t2, sizeof t2, rep.tol.latLonDeg);
            row("ins.verify.row.lon", v2, t2, rep.maxLonDeg > rep.tol.latLonDeg);
            fixed(v3, sizeof v3, rep.maxHM, "%.4f");
            fixed(t3, sizeof t3, rep.tol.hM, "%.4f");
            row("common.height_m", v3, t3, rep.maxHM > rep.tol.hM);
            sci(v4, sizeof v4, rep.maxV);
            sci(t4, sizeof t4, rep.tol.vMps);
            row("ins.verify.row.velocity", v4, t4, rep.maxV > rep.tol.vMps);
            sci(v5, sizeof v5, rep.maxRollDeg);
            sci(t5, sizeof t5, rep.tol.attDeg);
            row("export.col.roll", v5, t5, rep.maxRollDeg > rep.tol.attDeg);
            sci(v6, sizeof v6, rep.maxPitchDeg);
            sci(t6, sizeof t6, rep.tol.attDeg);
            row("export.col.pitch", v6, t6, rep.maxPitchDeg > rep.tol.attDeg);
            sci(v7, sizeof v7, rep.maxHeadingDeg);
            sci(t7, sizeof t7, rep.tol.attDeg);
            row("export.col.heading", v7, t7, rep.maxHeadingDeg > rep.tol.attDeg);
            ImGui::EndTable();
        }
        ImGui::TextDisabled("%s", I18n::trc("ins.verify.note.tol"));
        ImGui::TextDisabled("%s", I18n::tr("ins.verify.note.horizontal",
                                           rep.maxHorizontalM).c_str());
        ImGui::Spacing();

        {
            ImGui::BeginDisabled(!done);
            const bool s1 = ImGui::Button(I18n::trc("ins.verify.export.points"), ImVec2(200, 0));
            ImGui::SameLine();
            const bool s2 = ImGui::Button(I18n::trc("ins.align.export.summary"), ImVec2(150, 0));
            const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
            ImGui::EndDisabled();
            if (!done && hovered) ImGui::SetTooltip("%s", I18n::trc("ins.verify.export_hint"));
            if (s1) exportSampleCSV();
            if (s2) exportSummaryCSV();
        }
        ImGui::Spacing();

        std::vector<double> tx, dn, de, du, vn, ve, vd, dr, dp, dh;
        if (!rep.series.empty()) {
            const auto n = rep.series.size();
            tx.reserve(n);
            dn.reserve(n);
            de.reserve(n);
            du.reserve(n);
            vn.reserve(n);
            ve.reserve(n);
            vd.reserve(n);
            dr.reserve(n);
            dp.reserve(n);
            dh.reserve(n);
            for (const auto &s: rep.series) {
                tx.push_back(s.t);
                dn.push_back(s.dN);
                de.push_back(s.dE);
                du.push_back(s.dU);
                vn.push_back(s.dVn);
                ve.push_back(s.dVe);
                vd.push_back(s.dVd);
                dr.push_back(s.dRoll);
                dp.push_back(s.dPitch);
                dh.push_back(s.dHeading);
            }
        }
        const bool chartNew = chartDirty.exchange(false);
        auto drawErrPlot = [&](const char *title, const char *yLabel,
                               const std::vector<double> &a, const std::vector<double> &b,
                               const std::vector<double> &c,
                               const char *n1, const char *n2, const char *n3) {
            if (tx.empty()) {
                ImGui::TextDisabled("%s", I18n::trc("ins.no_data"));
                return;
            }
            const int n = static_cast<int>(tx.size());
            if (ImPlot::BeginPlot(I18n::trc(title), ImVec2(-1, kFileChartH))) {
                ImPlot::SetupAxes(I18n::trc("ins.chart.x_time"), I18n::trc(yLabel));
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, tx.back());
                ImPlot::PlotLine(I18n::trc(n1), tx.data(), a.data(), n, lineSpec(kColX));
                ImPlot::PlotLine(I18n::trc(n2), tx.data(), b.data(), n, lineSpec(kColY));
                ImPlot::PlotLine(I18n::trc(n3), tx.data(), c.data(), n, lineSpec(kColZ));
                ImPlot::EndPlot();
            }
        };
        drawErrPlot("ins.verify.chart.pos", "ins.verify.unit.m", dn, de, du, "ins.verify.chart.north",
                    "ins.verify.chart.east", "ins.verify.chart.up");
        drawErrPlot("ins.verify.chart.vel", "ins.verify.unit.mps", vn, ve, vd, "vN", "vE", "vD");
        drawErrPlot("ins.verify.chart.att", "ins.verify.unit.deg", dr, dp, dh,
                    "Roll", "Pitch", "Heading");
    }

    void InsVerifyTask::exportSampleCSV() {
        std::vector<::Ins::VerifySample> s;
        {
            std::lock_guard lk(resultMutex);
            s = report.series;
        }
        if (s.empty()) {
            setError(I18n::tr("ins.verify.err.no_result"));
            return;
        }
        exportCSVDialog(*this, "ins_verify_samples.csv", "ins_verify_samples.csv", [&](CSV::Writer &w) {
            w.row("t(s)", "dN(m)", "dE(m)", "dU(m)", "dVn(m/s)", "dVe(m/s)", "dVd(m/s)",
                  "dRoll(deg)", "dPitch(deg)", "dHeading(deg)");
            for (const auto &p: s)
                w.row(CSV::fx(p.t, 3), CSV::fx(p.dN, 5), CSV::fx(p.dE, 5), CSV::fx(p.dU, 5),
                      CSV::fx(p.dVn, 7), CSV::fx(p.dVe, 7), CSV::fx(p.dVd, 7),
                      CSV::fx(p.dRoll, 8), CSV::fx(p.dPitch, 8), CSV::fx(p.dHeading, 8));
        });
    }

    void InsVerifyTask::exportSummaryCSV() {
        ::Ins::VerifyReport rep;
        {
            std::lock_guard lk(resultMutex);
            rep = report;
        }
        if (rep.compared == 0) {
            setError(I18n::tr("ins.verify.err.no_result"));
            return;
        }
        exportCSVDialog(*this, "ins_verify_summary.csv", "ins_verify_summary.csv", [&](CSV::Writer &w) {
            w.row(I18n::trc("ins.verify.col.item"), I18n::trc("ins.verify.col.value"),
                  I18n::trc("Tolerance"), I18n::trc("ins.verify.col.verdict"));
            auto line = [&](const char *name, const CSV::FixedNum v, const CSV::FixedNum tol, const bool over) {
                w.row(name, v, tol, over ? I18n::trc("ins.verify.over") : I18n::trc("ins.verify.pass"));
            };
            auto lineSci = [&](const char *name, const double v, const double tol, const bool over) {
                w.row(name, CSV::sci(v, 3), CSV::sci(tol, 1), over ? I18n::trc("ins.verify.over") : I18n::trc("ins.verify.pass"));
            };
            lineSci(I18n::trc("ins.verify.row.lat"), rep.maxLatDeg, rep.tol.latLonDeg,
                    rep.maxLatDeg > rep.tol.latLonDeg);
            lineSci(I18n::trc("ins.verify.row.lon"), rep.maxLonDeg, rep.tol.latLonDeg,
                    rep.maxLonDeg > rep.tol.latLonDeg);
            line(I18n::trc("common.height_m"), CSV::fx(rep.maxHM, 5), CSV::fx(rep.tol.hM, 4), rep.maxHM > rep.tol.hM);
            lineSci(I18n::trc("ins.verify.csv.max_v"), rep.maxV, rep.tol.vMps, rep.maxV > rep.tol.vMps);
            lineSci(I18n::trc("export.col.roll"), rep.maxRollDeg, rep.tol.attDeg, rep.maxRollDeg > rep.tol.attDeg);
            lineSci(I18n::trc("export.col.pitch"), rep.maxPitchDeg, rep.tol.attDeg, rep.maxPitchDeg > rep.tol.attDeg);
            lineSci(I18n::trc("export.col.heading"), rep.maxHeadingDeg, rep.tol.attDeg, rep.maxHeadingDeg > rep.tol.attDeg);
            w.blank();
            w.row(I18n::trc("ins.verify.csv.horizontal_max"), CSV::fx(rep.maxHorizontalM, 4));
            w.row(I18n::trc("ins.verify.kv.imu_epochs"), std::to_string(rep.imuEpochs));
            w.row(I18n::trc("ins.verify.kv.ref_epochs"), std::to_string(rep.refEpochs));
            w.row(I18n::trc("ins.verify.kv.compared"), std::to_string(rep.compared));
            w.row(I18n::trc("ins.verify.csv.t0"), CSV::fx(rep.t0, 3));
            w.row(I18n::trc("ins.verify.csv.t1"), CSV::fx(rep.t1, 3));
            w.row(I18n::trc("common.conclusion"),
                  rep.pass ? I18n::trc("ins.verify.csv.all_pass") : I18n::trc("ins.verify.over_tol"));
        });
    }
}
