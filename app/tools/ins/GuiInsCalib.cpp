#include "Log.h"
#include "FileUtils.h"
#include "i18n/I18n.h"
#include "GuiInsPanel.h"
#include "GuiInsCommon.h"

#include "solvers/view/GuiHelpers.h"
#include "ui/FileDialog.h"
#include "ui/Splitter.h"
#include "solvers/common/Core.h"
#include "core/PathMemory.h"
#include "solver/ins/Calibrator.h"
#include "core/ins/InsMath.h"
#include "core/Const.h"
#include "StringUtils.h"

#include <algorithm>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Tools::Ins {
    namespace {
        /// 每个标定量当前选用的单位（下标进 kCalibQty 的 units），只在本面板里用。
        int s_qtyUnit[Q_COUNT] = {0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    }

    SixPosCalibTask::SixPosCalibTask() {
        fileName = "sixpos-calib";
        nameKey = "gui.menu.tools.ins_calib";
        cfgKey = "ins.calib.window_title";
        startKey = "ins.calib.start";
    }

    void SixPosCalibTask::startRun() {
        {
            const auto &cfg = AppConfig::PathMemory::instance();
            saveInsParams(par);
            std::string joined;
            for (std::size_t i = 0; i < filePaths.size(); ++i) {
                if (i) joined += '|';
                joined += filePaths[i];
            }
            cfg.set("ins_cal_files", joined);
        }
        if (filePaths.empty()) {
            setError(I18n::tr("ins.calib.err.no_file"));
            return;
        }
        if (!scale.known()) {
            setError(I18n::tr("ins.err.scale_missing"));
            return;
        }
        scaleText = scaleDet.text; // 后台线程不读界面状态
        beginRun();
        accelReady.store(false);
        segStatic.store(0);
        segRot.store(0);
        dupSkipped.store(0);
        detectDone.store(false);
        {
            std::lock_guard lk(resultMutex);
            currentFile.clear();
            detectSeq.clear();
            accelScaleGp.setZero();
            accelRmsp = 0.0;
            report = CalibReport();

            compSegs.pos.clear();
            compSegs.rot.clear();
            compSel = 0;
        }
        worker = std::thread(&SixPosCalibTask::calibWorker, this);
    }

    void SixPosCalibTask::calibWorker() {

        ImuScale sc = scale;   // 采样率留空时按读到的时标补，跟 INS 解算同一口径
        Calibrator::CalibSegments segs;
        std::vector<std::pair<std::string, std::string> > detSeqLines;
        int dups = 0;

        auto parseOne = [&](const std::string &path, ImuLog &out) -> bool {
            out.clear();
            auto onSample = [&](const ImuSample &s) { out.push_back(s); };
            auto onBatch = [&](const long long n) { sampleCount.store(n); };
            if (!pumpImuFile(path, stop, onSample, onBatch))
                return false;
            return !out.empty();
        };

        std::set<std::string> seenHash;
        int nStatic = 0, nRot = 0;

        for (const auto &path: filePaths) {
            if (stop) break;
            const std::string name = File::baseName(path);
            {
                std::lock_guard lk(resultMutex);
                currentFile = name;
            }
            ImuLog log;
            if (!parseOne(path, log) || log.empty()) {
                if (!stop) detSeqLines.emplace_back(name, I18n::tr("ins.calib.err.parse_failed"));
                continue;
            }
            const std::string hash = File::contentHash(path);
            if (seenHash.count(hash)) {
                ++dups;
                detSeqLines.emplace_back(name, I18n::tr("ins.calib.err.duplicate"));
                continue;
            }
            seenHash.insert(hash);


            auto segsFile = Calibrator::detect(log);
            std::string seq;
            for (const auto &[isStatic, pos, rot, i0, i1]: segsFile) {
                if (i1 <= i0) continue;
                ImuLog sub(log.begin() + i0, log.begin() + i1);
                if (isStatic) {
                    ++nStatic;
                    segs.pos.emplace_back(pos, std::move(sub));
                    seq += std::string(seq.empty() ? "" : " ") + Calibrator::posLabel(pos);
                } else {
                    ++nRot;
                    segs.rot.emplace_back(rot, std::move(sub));
                    seq += std::string(seq.empty() ? "" : " ") + Calibrator::rotLabel(rot);
                }
            }
            if (seq.empty()) seq = I18n::tr("ins.calib.seq.none");
            detSeqLines.emplace_back(name, seq);
        }

        segStatic.store(nStatic);
        segRot.store(nRot);
        dupSkipped.store(dups);
        {
            std::lock_guard lk(resultMutex);
            detectSeq = detSeqLines;
            detectDone.store(true);
        }

        if (stoppedAndEnd()) return;

        if (segs.pos.empty() && segs.rot.empty()) {
            abortRun(I18n::tr("ins.calib.err.no_segment"));
            return;
        }
        CalibReport rep = Calibrator::calibrate(segs, par.turnAngleDeg * DEG_TO_RAD, par.g, sc.rateHz);
        if (!rep.errorMsg.empty()) {
            abortRun(rep.errorMsg);
            return;
        }
        Calibrator::applyNominal(rep, sc);
        if (!rep.errorMsg.empty()) {
            abortRun(rep.errorMsg);
            return;
        }

        {
            std::lock_guard lk(resultMutex);
            compSegs = segs;
            compSel = 0;
            for (int i = 0; i < static_cast<int>(compSegs.pos.size()); ++i)
                if (compSegs.pos[i].first == Calibrator::XUP) {
                    compSel = i;
                    break;
                }
        }

        {
            std::lock_guard lk(resultMutex);
            accelScaleGp = rep.accelScaleG;
            accelRmsp = rep.accelRms;
        }
        accelReady.store(true);

        {

            char ka[96];
            std::snprintf(ka, sizeof ka, "[%.2f %.2f %.2f] LSB/g",
                          rep.accelScaleG(0), rep.accelScaleG(1), rep.accelScaleG(2));
            LOG_INFO << "[" << fileName << "] " << I18n::tr("ins.calib.log.done", segStatic.load(),
                                                           segRot.load(), scaleText,
                                                           std::string(ka), rep.accelRms);
        }
        {
            std::lock_guard lk(resultMutex);
            report = std::move(rep);
        }
        finishRun();
    }

    void SixPosCalibTask::renderFileTab() {
        ImGui::TextUnformatted(I18n::trc("ins.calib.hint.files"));
        ImGui::Spacing();

        if (ImGui::Button(I18n::trc("common.browse"), ImVec2(140, 0))) {
            if (std::vector<std::wstring> wpaths; FileDialog::openMany(wpaths, FileDialog::fIns())) {
                for (auto &wp: wpaths) {
                    std::string p = Str::wideToACP(wp);
                    if (!p.empty() && std::find(filePaths.begin(), filePaths.end(), p) == filePaths.end())
                        filePaths.push_back(p);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(I18n::trc("common.clear"), ImVec2(80, 0)))
            filePaths.clear();

        ImGui::Spacing();
        ImGui::Separator();
        if (filePaths.empty()) {
            ImGui::TextDisabled("%s", I18n::trc("ins.calib.no_files"));
        } else {
            ImGui::TextUnformatted(I18n::tr("ins.calib.selected_files", filePaths.size()).c_str());
            for (std::size_t i = 0; i < filePaths.size(); ++i) {
                ImGui::Bullet();
                ImGui::Text("%s", File::baseName(filePaths[i]).c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton(("x##cal" + std::to_string(i)).c_str())) {
                    filePaths.erase(filePaths.begin() + i);
                    --i;
                }
            }
        }
        ImGui::Spacing();
        ImGui::Separator();
        Solvers::Helpers::renderImuScale(scale, scaleDet,
                                         filePaths.empty() ? std::string() : filePaths.front(), par.g,
                                         "calib_scale");
    }

    void SixPosCalibTask::renderParamTab() {
        renderInsParams(par);
    }

    void SixPosCalibTask::renderResult() {
        const bool running = state == State::Running;
        const bool done = hasResult.load();
        const char *kPending = running ? I18n::trc("ins.chart.computing") : "—";
        const bool blocked = renderResultPrologue();
        const bool detReady = detectDone.load();
        if (detReady) {
            ImGui::Spacing();
            ImGui::TextUnformatted(I18n::tr("ins.calib.detected", segStatic.load(), segRot.load(),
                                            dupSkipped.load(), sampleCount.load()).c_str());
        }
        if (detReady &&
            ImGui::Button(I18n::trc("ins.calib.export_detect"), ImVec2(200, 0)))
            exportDetectCSV();
        if (blocked) return;
        if (running) {
            std::string label;
            {
                std::lock_guard lk(resultMutex);
                label = currentFile;
            }
            ImGui::TextColored(kColInfo, "%s",
                               I18n::tr("ins.calib.parsing",
                                        label.empty() ? std::string("—") : label,
                                        segStatic.load(), segRot.load(), dupSkipped.load(),
                                        sampleCount.load()).c_str());
            if (ImGui::Button(I18n::trc("ins.calib.stop"))) stop = true;
            ImGui::Spacing();
        }

        CalibReport rep;
        bool aReady = false, hResult = false;
        Eigen::Vector3d accS;
        double arm = 0.0;
        {
            std::lock_guard lk(resultMutex);
            rep = report;
            aReady = accelReady.load();
            hResult = hasResult.load();
            accS = accelScaleGp;
            arm = accelRmsp;
        }
        const bool chartNew = chartDirty.exchange(false);

        if (ImGui::BeginTable("##cal_tbl", 5,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn(I18n::trc("ins.calib.col.qty"),
                                    ImGuiTableColumnFlags_WidthStretch, 3.4f);
            ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn(I18n::trc("ins.calib.col.unit"),
                                    ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableHeadersRow();

            auto unitCombo = [](const int q) {
                const CalibQty &def = kCalibQty[q];
                ImGui::PushID(q);
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##unit", unitText(def.units[s_qtyUnit[q]].ui))) {
                    for (int i = 0; i < def.nUnits; ++i) {
                        ImGui::PushID(i);
                        if (ImGui::Selectable(unitText(def.units[i].ui), i == s_qtyUnit[q]))
                            s_qtyUnit[q] = i;
                        ImGui::PopID();
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopID();
            };

            auto qtyRow = [&](const int q, const bool ready) {
                const CalibQty &def = kCalibQty[q];
                const CalibUnit &u = def.units[s_qtyUnit[q]];
                const Eigen::Vector3d v = calibQtyValue(def, u, rep, par.g);
                const bool withSigma = def.sigmaQty >= 0;
                Eigen::Vector3d sg = Eigen::Vector3d::Zero();
                int d = 0;
                if (withSigma) {
                    sg = calibQtyValue(kCalibQty[def.sigmaQty], u, rep, par.g);
                    for (int c = 0; c < 3; ++c) d = std::max(d, pmDecimals(sg(c)));
                }
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(I18n::trc(def.uiKey));
                for (int c = 0; c < 3; ++c) {
                    ImGui::TableSetColumnIndex(1 + c);
                    if (!ready) {
                        ImGui::TextDisabled("%s", kPending);
                        continue;
                    }
                    char b[64];
                    if (withSigma) std::snprintf(b, sizeof b, "%.*f ± %.*f", d, v(c), d, sg(c));
                    else std::snprintf(b, sizeof b, u.uiFmt, v(c));
                    ImGui::TextUnformatted(b);
                }
                ImGui::TableSetColumnIndex(4);
                if (def.nUnits > 1) unitCombo(q);
                else ImGui::TextDisabled("%s", unitText(def.units[0].ui));
            };

            for (const int q: kCalibUiRows) {
                const bool accelSide = q == Q_ACC_SCALE || q == Q_ACC_BIAS || q == Q_ACC_DS;
                qtyRow(q, accelSide ? aReady : hResult);
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        if (aReady) {
            ImGui::Spacing();
            ImGui::SeparatorText(I18n::trc("ins.calib.sep.na"));
            if (ImGui::BeginTable("##cross_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("");
                ImGui::TableSetupColumn(I18n::trc("ins.calib.in_x"));
                ImGui::TableSetupColumn(I18n::trc("ins.calib.in_y"));
                ImGui::TableSetupColumn(I18n::trc("ins.calib.in_z"));
                ImGui::TableHeadersRow();
                static const char *kOut[3] = {"ins.calib.out_x", "ins.calib.out_y", "ins.calib.out_z"};
                for (int i = 0; i < 3; ++i) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(I18n::trc(kOut[i]));
                    for (int j = 0; j < 3; ++j) {
                        ImGui::TableSetColumnIndex(1 + j);
                        char b[48];
                        std::snprintf(b, sizeof(b), "%.7f", rep.accelCross(i, j));
                        ImGui::TextUnformatted(b);
                    }
                }
                ImGui::EndTable();
            }
        }
        if (hResult) {
            ImGui::Spacing();
            ImGui::SeparatorText(I18n::trc("ins.calib.sep.ng"));
            if (ImGui::BeginTable("##gcross_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("");
                ImGui::TableSetupColumn(I18n::trc("ins.calib.in_x"));
                ImGui::TableSetupColumn(I18n::trc("ins.calib.in_y"));
                ImGui::TableSetupColumn(I18n::trc("ins.calib.in_z"));
                ImGui::TableHeadersRow();
                static const char *kOutG[3] = {"ins.calib.out_x", "ins.calib.out_y", "ins.calib.out_z"};
                for (int i = 0; i < 3; ++i) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(I18n::trc(kOutG[i]));
                    for (int j = 0; j < 3; ++j) {
                        ImGui::TableSetColumnIndex(1 + j);
                        char b[48];
                        std::snprintf(b, sizeof(b), "%.7f", rep.gyroCross(i, j));
                        ImGui::TextUnformatted(b);
                    }
                }
                ImGui::EndTable();
            }
        }

        {
            const double meanG = accS.mean();
            const double spreadG = accS.maxCoeff() - accS.minCoeff();
            const double maxds = rep.gyroDeltaS.cwiseAbs().maxCoeff();
            ImGui::Spacing();
            ImGui::SeparatorText(I18n::trc("ins.calib.sep.quality"));
            if (ImGui::BeginTable("##qc_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn(I18n::trc("ins.calib.col.check"),
                                        ImGuiTableColumnFlags_WidthStretch, 1.9f);
                ImGui::TableSetupColumn(I18n::trc("Criterion"),
                                        ImGuiTableColumnFlags_WidthStretch, 1.6f);
                ImGui::TableSetupColumn(I18n::trc("ins.calib.col.measured"),
                                        ImGuiTableColumnFlags_WidthStretch, 2.6f);
                ImGui::TableSetupColumn(I18n::trc("common.conclusion"),
                                        ImGuiTableColumnFlags_WidthStretch, 0.8f);
                ImGui::TableHeadersRow();
                auto qcRow = [kPending](const char *item, const char *cond, const bool ready, const char *val, const bool ok) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(item);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextDisabled("%s", cond);
                    ImGui::TableSetColumnIndex(2);
                    if (ready) ImGui::TextUnformatted(val);
                    else ImGui::TextDisabled("%s", kPending);
                    ImGui::TableSetColumnIndex(3);
                    if (!ready) ImGui::TextDisabled("—");
                    else
                        ImGui::TextColored(ok ? kColPass : kColFail, "%s",
                                           ok ? I18n::trc("ins.calib.pass")
                                              : I18n::trc("ins.calib.over"));
                };
                char v[128];
                const bool okG = aReady && meanG > 1.0 && spreadG / meanG < 0.05;
                std::snprintf(v, sizeof(v), "%.2f %%", meanG > 0.0 ? spreadG / meanG * 100.0 : 0.0);
                qcRow(I18n::trc("ins.calib.qc.acc_consistency"), "(max−min)/mean < 5 %", aReady, v, okG);

                const bool okRms = aReady && arm < 0.02 * meanG;
                std::snprintf(v, sizeof(v), "%.2f LSB", arm);
                qcRow(I18n::trc("ins.calib.qc.rms"), "RMS < 2 % · mean K_a", aReady, v, okRms);

                const bool okDs = hResult && maxds < 0.05;
                std::snprintf(v, sizeof(v), "%.3f %%", maxds * 100.0);
                qcRow(I18n::trc("ins.calib.qc.gyro_ds"), "max|δs_g| < 5 %", hResult, v, okDs);

                const double maxNg = rep.gyroCross.cwiseAbs().maxCoeff();
                const bool okNg = hResult && maxNg < 1e-2;
                std::snprintf(v, sizeof(v), "%.3f %%", maxNg * 100.0);
                qcRow(I18n::trc("ins.calib.qc.gyro_ng"), "max|N_g| < 1 %", hResult, v, okNg);
                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        ImGui::SeparatorText(I18n::trc("ins.calib.sep.scale_consistency"));
        const double scA[3] = {accS(0), accS(1), accS(2)};
        const double scG[3] = {rep.gyroKeff(0), rep.gyroKeff(1), rep.gyroKeff(2)};
        float &scH = UI::splitHeight("ins_calib_scale_row", kChartHeight);
        ImGui::Columns(2, nullptr, false);
        drawScaleBars("ins.calib.chart.acc_scale", aReady, scA, kColZ, scH, "%.1f");
        ImGui::NextColumn();
        drawScaleBars("ins.calib.chart.gyro_keff", hResult, scG, kColZ, scH, "%.0f");
        ImGui::Columns(1);
        UI::splitHandle("##calibScaleGrip", scH, 200.0f, 900.0f);


        ImGui::Spacing();
        {

            ImGui::BeginDisabled(!done);
            const bool clicked = ImGui::Button(I18n::trc("ins.calib.export_result"), ImVec2(180, 0));
            const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
            ImGui::EndDisabled();
            if (!done && hovered) ImGui::SetTooltip("%s", I18n::trc("ins.calib.export_hint"));
            if (clicked) exportCalibCSV();
        }

        ImGui::SeparatorText(I18n::trc("ins.calib.sep.comp_series"));

        if (const auto segLabels = compSegLabels(); !segLabels.empty()) {
            compSel = std::clamp(compSel, 0, static_cast<int>(segLabels.size()) - 1);
            ImGui::SetNextItemWidth(190.0f);
            if (ImGui::BeginCombo("##comp_seg", segLabels[compSel].c_str())) {
                for (int i = 0; i < static_cast<int>(segLabels.size()); ++i) {
                    const bool sel = i == compSel;
                    ImGui::PushID(i);
                    if (ImGui::Selectable(segLabels[i].c_str(), sel)) compSel = i;
                    if (sel) ImGui::SetItemDefaultFocus();
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if (ImGui::Button(I18n::trc("ins.calib.export_comp"), ImVec2(210, 0))) exportCompCSV();


            if (CalibReport r; peekReport(r)) {
                ImuLog seg;
                {
                    std::lock_guard lk(resultMutex);
                    const int np = static_cast<int>(compSegs.pos.size());
                    const int nr = static_cast<int>(compSegs.rot.size());
                    if (const int total = np + nr; total > 0) { // 段可能被 worker 清空，空表不能 clamp
                        const int k = std::clamp(compSel, 0, total - 1);
                        decimateImu(k < np ? compSegs.pos[k].second : compSegs.rot[k - np].second, seg);
                    }
                }
                if (!seg.empty()) {
                    const Calibrator::CompMat cm = Calibrator::compMatrices(r);
                    std::vector<double> t, fx, fy, fz, gx, gy, gz;
                    t.reserve(seg.size());
                    fx.reserve(seg.size());
                    fy.reserve(seg.size());
                    fz.reserve(seg.size());
                    gx.reserve(seg.size());
                    gy.reserve(seg.size());
                    gz.reserve(seg.size());
                    for (const auto &smp: seg) {
                        t.push_back(smp.tow);
                        Eigen::Vector3d _f, _g;
                        Calibrator::compensate(cm, r, smp, _f, _g);
                        fx.push_back(_f(0));
                        fy.push_back(_f(1));
                        fz.push_back(_f(2));
                        gx.push_back(_g(0));
                        gy.push_back(_g(1));
                        gz.push_back(_g(2));
                    }
                    std::vector<double> tel;
                    toElapsed(t, tel);
                    float &cmH = UI::splitHeight("ins_calib_comp_row", kFileChartH);
                    ImGui::Columns(2, nullptr, false);
                    drawCompChart("ins.calib.chart.comp_f", tel, fx, fy, fz, chartNew, cmH);
                    ImGui::NextColumn();
                    drawCompChart("ins.calib.chart.comp_w", tel, gx, gy, gz, chartNew, cmH);
                    ImGui::Columns(1);
                    UI::splitHandle("##calibCompGrip", cmH, 160.0f, 900.0f);
                }
            }
        }
    }

    bool SixPosCalibTask::peekReport(CalibReport &r) {
        if (!hasResult.load()) return false;
        std::lock_guard lk(resultMutex);
        r = report;
        return true;
    }

    bool SixPosCalibTask::snapshotReport(CalibReport &r) {
        if (peekReport(r)) return true;
        setError(I18n::tr("ins.calib.err.no_result"));
        return false;
    }

    void SixPosCalibTask::exportCalibCSV() {
        CalibReport r;
        if (!snapshotReport(r)) return;
        exportCSVDialog(*this, "calib_result.csv", "calib_result.csv", [&](CSV::Writer &w) {
            w.row(I18n::trc("ins.calib.col.qty"), "X", "Y", "Z");

            auto qtyRow = [&](const int q) {
                const CalibQty &def = kCalibQty[q];
                const CalibUnit &u = def.units[s_qtyUnit[q]];
                const Eigen::Vector3d v = calibQtyValue(def, u, r, par.g);
                const std::string name = std::string(I18n::trc(def.csv)) + "(" + unitText(u.csv) + ")";
                const int d = def.csvDigits;
                if (def.scalar) {
                    if (def.csvSci) w.row(name, CSV::sci(v(0), d), "", "");
                    else w.row(name, CSV::fx(v(0), d), "", "");
                } else if (def.csvSci) {
                    w.row(name, CSV::sci(v(0), d), CSV::sci(v(1), d), CSV::sci(v(2), d));
                } else {
                    w.row(name, CSV::fx(v(0), d), CSV::fx(v(1), d), CSV::fx(v(2), d));
                }
            };

            qtyRow(Q_ACC_SCALE);
            qtyRow(Q_ACC_SCALE_SIGMA);
            qtyRow(Q_ACC_BIAS);
            for (int k = 0; k < 3; ++k) {
                const std::string rn = "M" + std::to_string(k) + "0, M" + std::to_string(k) + "1, M" +
                                       std::to_string(k) + "2";
                w.row(rn, CSV::fx(r.calib.accelM(k, 0), 6), CSV::fx(r.calib.accelM(k, 1), 6),
                      CSV::fx(r.calib.accelM(k, 2), 6));
            }
            qtyRow(Q_ACC_KNOM);
            qtyRow(Q_ACC_DS);
            for (int k = 0; k < 3; ++k) {
                const std::string rn = "N" + std::to_string(k) + "0, N" + std::to_string(k) + "1, N" +
                                       std::to_string(k) + "2";
                w.row(rn, CSV::sci(r.accelCross(k, 0), 9), CSV::sci(r.accelCross(k, 1), 9),
                      CSV::sci(r.accelCross(k, 2), 9));
            }
            qtyRow(Q_GYRO_BIAS);
            qtyRow(Q_GYRO_KEFF);
            qtyRow(Q_GYRO_KEFF_NOM);
            qtyRow(Q_GYRO_SCALE);
            qtyRow(Q_GYRO_DS);

            for (int k = 0; k < 3; ++k) {
                const std::string rn = "Ng" + std::to_string(k) + "0, Ng" + std::to_string(k) + "1, Ng" +
                                       std::to_string(k) + "2";
                w.row(rn, CSV::sci(r.gyroCross(k, 0), 9), CSV::sci(r.gyroCross(k, 1), 9),
                      CSV::sci(r.gyroCross(k, 2), 9));
            }
            w.row(I18n::trc("ins.calib.csv.dt"), CSV::fx(r.dt, 6));
            w.row(I18n::trc("ins.calib.csv.fit_rms"), CSV::fx(r.accelRms, 4));
            w.row(I18n::trc("ins.calib.csv.n_pos"), r.nPos[0], r.nPos[1], r.nPos[2], r.nPos[3], r.nPos[4], r.nPos[5]);
            w.row(I18n::trc("ins.calib.csv.n_rot"), r.nRot[0], r.nRot[1], r.nRot[2], r.nRot[3], r.nRot[4], r.nRot[5]);
        });
    }

    void SixPosCalibTask::exportDetectCSV() {
        std::vector<std::pair<std::string, std::string> > seq;
        {
            std::lock_guard lk(resultMutex);
            seq = detectSeq;
        }
        if (seq.empty()) {
            setError(I18n::tr("ins.calib.err.no_detect"));
            return;
        }
        exportCSVDialog(*this, "detect_summary.csv", "detect_summary.csv", [&](CSV::Writer &w) {
            w.row(I18n::trc("common.file"), I18n::trc("ins.calib.csv.segments"));
            for (const auto &[fst, snd]: seq) w.row(fst, snd);
        });
    }

    std::vector<std::string> SixPosCalibTask::compSegLabels() {
        std::lock_guard lk(resultMutex);
        std::vector<std::string> v;
        v.reserve(compSegs.pos.size() + compSegs.rot.size());
        for (const auto &[first, _]: compSegs.pos)
            v.emplace_back(I18n::tr("ins.calib.seg.pos", Calibrator::posLabel(first)));
        for (const auto &[first, _]: compSegs.rot)
            v.emplace_back(I18n::tr("ins.calib.seg.rot", Calibrator::rotLabel(first)));
        std::map<std::string, int> seen;
        for (auto &s: v) {
            if (const int n = ++seen[s]; n > 1) s += " #" + std::to_string(n);
        }
        return v;
    }

    static std::string compPosFileName(const Calibrator::Pos p) {
        static const char *tag[6] = {"xup", "xdn", "yup", "ydn", "zup", "zdn"};
        return std::string("calib_") + tag[static_cast<int>(p)] + "_comp.csv";
    }

    static std::string compRotFileName(const Calibrator::Rot r) {
        static const char *tag[6] = {"xfwd", "xrev", "yfwd", "yrev", "zfwd", "zrev"};
        return std::string("calib_") + tag[static_cast<int>(r)] + "_comp.csv";
    }

    void SixPosCalibTask::exportCompCSV() {
        CalibReport r;
        if (!snapshotReport(r)) return;

        ImuLog seg;
        std::string label;
        std::string defName;
        int selIdx = -1;
        {
            std::lock_guard lk(resultMutex);
            if (const int np = static_cast<int>(compSegs.pos.size()), nr = static_cast<int>(compSegs.rot.size()); np + nr > 0) {
                const int k = std::clamp(compSel, 0, np + nr - 1);
                selIdx = k;
                if (k < np) {
                    seg = compSegs.pos[k].second;
                    defName = compPosFileName(compSegs.pos[k].first);
                } else {
                    seg = compSegs.rot[k - np].second;
                    defName = compRotFileName(compSegs.rot[k - np].first);
                }
            }
        }
        if (seg.empty()) {
            setError(I18n::tr("ins.calib.err.no_samples"));
            return;
        }
        if (const auto segLabels = compSegLabels();
            selIdx < static_cast<int>(segLabels.size()))
            label = segLabels[selIdx];

        const Calibrator::CompMat cm = Calibrator::compMatrices(r);
        exportCSVDialog(*this, defName, defName + "（" + label + "）", [&](CSV::Writer &w) {
            w.row("tow", "comp_fx(m/s2)", "comp_fy(m/s2)", "comp_fz(m/s2)",
                  "raw_gx", "raw_gy", "raw_gz", "comp_gx", "comp_gy", "comp_gz");
            for (const auto &s: seg) {
                Eigen::Vector3d f, g;
                Calibrator::compensate(cm, r, s, f, g);
                w.row(CSV::fx(s.tow, 3),
                      CSV::fx(f(0), 5), CSV::fx(f(1), 5), CSV::fx(f(2), 5),
                      s.gx, s.gy, s.gz,
                      CSV::fx(g(0), 4), CSV::fx(g(1), 4), CSV::fx(g(2), 4));
            }
        });
    }
}
