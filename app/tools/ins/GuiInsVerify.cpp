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

namespace Tools::Ins {

    InsVerifyTask::InsVerifyTask() {
        fileName = "INS 算法验证";
    }

    void InsVerifyTask::startVerify() {
        saveInsParams(par);
        if (imuPath.empty()) {
            setError("请先选择 IMU 增量文件（IMU.bin）。");
            return;
        }
        if (refPath.empty()) {
            setError("请先选择参考轨迹文件（PureINS.bin）。");
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
            abortRun(*this, "参考轨迹与 IMU 时段没有交集，未产生任何比对点。");
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

        char summary[256];
        std::snprintf(summary, sizeof summary,
                      "比对 %zu 点（%zu 条 IMU 增量）：dLat %.2e° dLon %.2e° dh %.4f m dV %.2e m/s，%s",
                      report.compared, report.imuEpochs, report.maxLatDeg, report.maxLonDeg,
                      report.maxHM, report.maxV, report.pass ? "全部在容差内" : "有超差项");
        LOG_INFO << "[" << fileName << "] " << summary;

        finishRun(*this);
    }

    void InsVerifyTask::renderConfigPanel() {
        renderInsConfig("INS 算法验证配置###cfg_", *this, "开始验证",
                        [&] {
                            Solvers::Helpers::fileRow("IMU 增量文件", &imuPath, "ins_verify_imu", FileDialog::fBin, 2,
                                             nullptr);
                            Solvers::Helpers::fileRow("参考轨迹文件", &refPath, "ins_verify_ref", FileDialog::fBin, 2,
                                             nullptr);
                            ImGui::Spacing();
                            ImGui::TextDisabled("IMU.bin 每条 7 个 double（t, Δθ×3, Δv×3），");
                            ImGui::TextDisabled("PureINS.bin 每条 10 个 double（t, φ, λ, h, v×3, 姿态×3）。");
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
            ImGui::TextColored(kColInfo, "正在比对 [%s]：第 %lld 个历元", fileName.c_str(),
                               static_cast<long long>(sampleCount.load()));
            ImGui::SameLine();
            if (ImGui::Button("停止验证")) stop = true;
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
            ImGui::TextColored(kColInfo, "比对进行中：已比对 %zu 点，最大水平偏差 %.4f m、速度 %.2e m/s。",
                               rep.compared, rep.maxHorizontalM, rep.maxV);
        } else if (valid) {
            std::string bad;
            auto mark = [&bad](const bool over, const char *name) {
                if (!over) return;
                if (!bad.empty()) bad += "、";
                bad += name;
            };
            mark(rep.maxLatDeg > rep.tol.latLonDeg, "纬度");
            mark(rep.maxLonDeg > rep.tol.latLonDeg, "经度");
            mark(rep.maxHM > rep.tol.hM, "高程");
            mark(rep.maxV > rep.tol.vMps, "速度");
            mark(rep.maxRollDeg > rep.tol.attDeg, "横滚");
            mark(rep.maxPitchDeg > rep.tol.attDeg, "俯仰");
            mark(rep.maxHeadingDeg > rep.tol.attDeg, "航向");
            if (bad.empty()) ImGui::TextColored(UI::kColPass, "全部指标在容差内");
            else ImGui::TextColored(UI::kColFail, "超差项：%s", bad.c_str());
        }

        if (!valid) {
            ImGui::TextDisabled("（尚无验证结果）");
            return;
        }

        ImGui::SeparatorText("数据概况");
        if (ImGui::BeginTable("##verify_sum", 4,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("项目");
            ImGui::TableSetupColumn("数值");
            ImGui::TableSetupColumn("项目");
            ImGui::TableSetupColumn("数值");
            ImGui::TableHeadersRow();

            auto kv = [](const char *k1, const char *v1, const char *k2, const char *v2) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(k1);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(v1);
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(k2);
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
            kv("IMU 增量历元", b1, "参考轨迹历元", b2);
            kv("比对点数", b3, "起始时标", b4);
            kv("截止时标", b5, "时长", b6);
            ImGui::EndTable();
        }
        ImGui::Spacing();

        ImGui::SeparatorText("与参考轨迹的最大偏差");
        if (ImGui::BeginTable("##verify_err", 4,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("量");
            ImGui::TableSetupColumn("最大偏差");
            ImGui::TableSetupColumn("容差");
            ImGui::TableSetupColumn("判定");
            ImGui::TableHeadersRow();

            auto row = [](const char *label, const char *val, const char *tol, const bool over) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(val);
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(tol);
                ImGui::TableSetColumnIndex(3);
                ImGui::TextColored(over ? kColFail : kColPass, "%s", over ? "超差" : "通过");
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
            row("纬度最大偏差 (°)", v1, t1, rep.maxLatDeg > rep.tol.latLonDeg);
            sci(v2, sizeof v2, rep.maxLonDeg);
            sci(t2, sizeof t2, rep.tol.latLonDeg);
            row("经度最大偏差 (°)", v2, t2, rep.maxLonDeg > rep.tol.latLonDeg);
            fixed(v3, sizeof v3, rep.maxHM, "%.4f");
            fixed(t3, sizeof t3, rep.tol.hM, "%.4f");
            row("高程 (m)", v3, t3, rep.maxHM > rep.tol.hM);
            sci(v4, sizeof v4, rep.maxV);
            sci(t4, sizeof t4, rep.tol.vMps);
            row("速度三分量最大 (m/s)", v4, t4, rep.maxV > rep.tol.vMps);
            sci(v5, sizeof v5, rep.maxRollDeg);
            sci(t5, sizeof t5, rep.tol.attDeg);
            row("横滚 (°)", v5, t5, rep.maxRollDeg > rep.tol.attDeg);
            sci(v6, sizeof v6, rep.maxPitchDeg);
            sci(t6, sizeof t6, rep.tol.attDeg);
            row("俯仰 (°)", v6, t6, rep.maxPitchDeg > rep.tol.attDeg);
            sci(v7, sizeof v7, rep.maxHeadingDeg);
            sci(t7, sizeof t7, rep.tol.attDeg);
            row("航向 (°)", v7, t7, rep.maxHeadingDeg > rep.tol.attDeg);
            ImGui::EndTable();
        }
        ImGui::TextDisabled("书里写的是「1.0e-7 deg 的量级」「1.0e-5~1.0e-4 m/s 的量级」，这里按量级上界取阈值。");
        ImGui::TextDisabled("水平偏差折成米后最大 %.4f m，仅作直观附注，不参与判定。", rep.maxHorizontalM);
        ImGui::Spacing();

        {
            ImGui::BeginDisabled(!done);
            const bool s1 = ImGui::Button("导出逐点误差 CSV", ImVec2(200, 0));
            ImGui::SameLine();
            const bool s2 = ImGui::Button("导出汇总 CSV", ImVec2(150, 0));
            const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
            ImGui::EndDisabled();
            if (!done && hovered) ImGui::SetTooltip("验证完成后才能导出。");
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
                ImGui::TextDisabled("（无数据）");
                return;
            }
            const int n = static_cast<int>(tx.size());
            if (ImPlot::BeginPlot(title, ImVec2(-1, kFileChartH))) {
                ImPlot::SetupAxes("时间 t (s)", yLabel);
                ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
                fitXToLatest(chartNew, tx.back());
                ImPlot::PlotLine(n1, tx.data(), a.data(), n, lineSpec(kColX));
                ImPlot::PlotLine(n2, tx.data(), b.data(), n, lineSpec(kColY));
                ImPlot::PlotLine(n3, tx.data(), c.data(), n, lineSpec(kColZ));
                ImPlot::EndPlot();
            }
        };
        drawErrPlot("位置误差", "m", dn, de, du, "北 N", "东 E", "天 U");
        drawErrPlot("速度误差", "m/s", vn, ve, vd, "vN", "vE", "vD");
        drawErrPlot("姿态误差", "°", dr, dp, dh, "横滚", "俯仰", "航向");
    }

    void InsVerifyTask::exportSampleCSV() {
        std::vector<::Ins::VerifySample> s;
        {
            std::lock_guard lk(resultMutex);
            s = report.series;
        }
        if (s.empty()) {
            setError("尚无验证结果，无法导出。请先完成一次验证。");
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
            setError("尚无验证结果，无法导出。请先完成一次验证。");
            return;
        }
        exportCSVDialog(*this, "ins_verify_summary.csv", "ins_verify_summary.csv", [&](CSV::Writer &w) {
            w.row("项目", "数值", "容差", "判定");
            auto line = [&](const char *name, const CSV::FixedNum v, const CSV::FixedNum tol, const bool over) {
                w.row(name, v, tol, over ? "超差" : "通过");
            };
            auto lineSci = [&](const char *name, const double v, const double tol, const bool over) {
                w.row(name, CSV::sci(v, 3), CSV::sci(tol, 1), over ? "超差" : "通过");
            };
            lineSci("纬度最大偏差 (°)", rep.maxLatDeg, rep.tol.latLonDeg,
                    rep.maxLatDeg > rep.tol.latLonDeg);
            lineSci("经度最大偏差 (°)", rep.maxLonDeg, rep.tol.latLonDeg,
                    rep.maxLonDeg > rep.tol.latLonDeg);
            line("高程 (m)", CSV::fx(rep.maxHM, 5), CSV::fx(rep.tol.hM, 4), rep.maxHM > rep.tol.hM);
            lineSci("速度 (m/s)", rep.maxV, rep.tol.vMps, rep.maxV > rep.tol.vMps);
            lineSci("横滚 (°)", rep.maxRollDeg, rep.tol.attDeg, rep.maxRollDeg > rep.tol.attDeg);
            lineSci("俯仰 (°)", rep.maxPitchDeg, rep.tol.attDeg, rep.maxPitchDeg > rep.tol.attDeg);
            lineSci("航向 (°)", rep.maxHeadingDeg, rep.tol.attDeg, rep.maxHeadingDeg > rep.tol.attDeg);
            w.blank();
            w.row("水平偏差折米最大值 (m)", CSV::fx(rep.maxHorizontalM, 4));
            w.row("IMU 增量历元", std::to_string(rep.imuEpochs));
            w.row("参考轨迹历元", std::to_string(rep.refEpochs));
            w.row("比对点数", std::to_string(rep.compared));
            w.row("起始时标 (s)", CSV::fx(rep.t0, 3));
            w.row("截止时标 (s)", CSV::fx(rep.t1, 3));
            w.row("结论", rep.pass ? "指导书判据全部满足" : "有超差项");
        });
    }
}
