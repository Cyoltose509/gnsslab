#include "GuiInsPanel.h"
#include "GuiInsCommon.h"

#include "GuiHelpers.h"
#include "SolverTask.h"
#include "core/PathMemory.h"
#include "ins/ImuCalibrator.h"
#include "core/InsMath.h"
#include "Const.h"
#include "util/StringUtils.h"

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

namespace Tools::InsPanel {

    SixPosCalibTask::SixPosCalibTask() {
        mode = GnssTask::SolverMode::Ins;
        fileName = "六位置法标定";
    }

    void SixPosCalibTask::startCalib() {
        {
            const auto &cfg = AppConfig::PathMemory::instance();
            saveInsParams(par);
            std::string joined;
            for (size_t i = 0; i < filePaths.size(); ++i) {
                if (i) joined += '|';
                joined += filePaths[i];
            }
            cfg.set("ins_cal_files", joined);
        }
        if (filePaths.empty()) {
            setError("请先添加至少一个 IMU 日志文件。");
            return;
        }
        {
            std::string sig = std::to_string(par.g);
            for (const auto &p: filePaths) {
                sig += '\n';
                sig += p;
            }
            keepManualScaleFor(scaleSf, sig);
        }
        beginRun(*this);
        accelReady.store(false);
        segStatic.store(0);
        segRot.store(0);
        dupSkipped.store(0);
        detectDone.store(false);
        {
            std::lock_guard lk(resultMutex);
            currentFile.clear();
            detectSeq.clear();
            accelMp.setIdentity();
            accelBp.setZero();
            accelScaleGp.setZero();
            accelRmsp = 0.0;
            report = CalibReport();
            scaleNotice = ScaleNotice{};

            compSegs.pos.clear();
            compSegs.rot.clear();
            compSel = 0;
        }
        launchWorker(*this, [this] { calibWorker(); });
    }

    void SixPosCalibTask::calibWorker() {

        ScaleIdentifier sen(scaleSf);
        ImuCalibrator::CalibSegments segs;
        std::vector<std::pair<std::string, std::string> > detSeqLines;
        int dups = 0;

        auto parseOne = [&](const std::string &path, ImuLog &out) -> bool {
            out.clear();
            auto onSample = [&](const ImuSample &s) { out.push_back(s); };
            auto onBatch = [&](const long long n) { sampleCount.store(n); };
            auto onReader = [&](auto &r) -> bool {
                return sen.observe(r, [&](const std::string &, const ScaleNotice &n) {
                    std::lock_guard lk(resultMutex);
                    scaleNotice = n;
                });
            };
            if (!pumpImuFile(path, stop, onReader, onSample, onBatch)) return false;
            return !out.empty();
        };

        std::set<std::string> seenHash;
        int nStatic = 0, nRot = 0;

        for (const auto &path: filePaths) {
            if (stop) break;
            const std::string name = Solvers::Helpers::baseName(path);
            {
                std::lock_guard lk(resultMutex);
                currentFile = name;
            }
            ImuLog log;
            if (!parseOne(path, log) || log.empty()) {
                if (sen.mixedModel()) {

                    abortRun(*this, sen.openFailure(path));
                    return;
                }
                if (!stop) detSeqLines.emplace_back(name, "解析失败（内容为空或格式不符）");
                continue;
            }
            const std::string hash = fileContentHash(path);
            if (seenHash.count(hash)) {
                ++dups;
                detSeqLines.emplace_back(name, "重复文件，已跳过");
                continue;
            }
            seenHash.insert(hash);


            auto segsFile = ImuCalibrator::detect(log);
            std::string seq;
            for (const auto &[isStatic, pos, rot, i0, i1]: segsFile) {
                if (i1 <= i0) continue;
                ImuLog sub(log.begin() + i0, log.begin() + i1);
                if (isStatic) {
                    ++nStatic;
                    sen.offerStatic(sub);
                    segs.pos.emplace_back(pos, std::move(sub));
                    seq += std::string(seq.empty() ? "" : " ") + ImuCalibrator::posLabel(pos);
                } else {
                    ++nRot;
                    segs.rot.emplace_back(rot, std::move(sub));
                    seq += std::string(seq.empty() ? "" : " ") + ImuCalibrator::rotLabel(rot);
                }
            }
            if (seq.empty()) seq = "未识别到静置段或旋转段";
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

        if (stoppedAndEnd(*this)) return;

        if (segs.pos.empty() && segs.rot.empty()) {
            abortRun(*this, "未识别到任何静置段或旋转段。请确认所选文件是 IMU 原始日志，且包含完整的六位置静置与旋转过程。");
            return;
        }
        if (!sen.resolve(par.g)) {

            abortRun(*this, sen.failText());
            return;
        }
        CalibReport rep = ImuCalibrator::calibrate(segs, par.turnAngleDeg * DEG_TO_RAD, par.g);
        if (!rep.ok) {
            abortRun(*this, rep.errorMsg);
            return;
        }
        ImuCalibrator::applyNominal(rep, sen.scale());
        if (!rep.ok) {
            abortRun(*this, rep.errorMsg);
            return;
        }

        {
            std::lock_guard lk(resultMutex);
            compSegs = segs;
            compSel = 0;
            for (int i = 0; i < static_cast<int>(compSegs.pos.size()); ++i)
                if (compSegs.pos[i].first == ImuCalibrator::XUP) {
                    compSel = i;
                    break;
                }
        }

        {
            std::lock_guard lk(resultMutex);
            accelMp = rep.calib.accelM;
            accelBp = rep.calib.accelB;
            accelScaleGp = rep.accelScaleG;
            accelRmsp = rep.accelRms;
        }
        accelReady.store(true);

        {

            char ka[96];
            std::snprintf(ka, sizeof ka, "[%.2f %.2f %.2f] LSB/g",
                          rep.accelScaleG(0), rep.accelScaleG(1), rep.accelScaleG(2));
            LOG_INFO << "[六位置法标定] 完成：静置段 " << segStatic.load() << " 个 / 旋转段 "
                     << segRot.load() << " 个，当量 " << sen.label() << "，K_a=" << ka
                     << "，拟合残差 RMS=" << rep.accelRms << " LSB";
        }
        {
            std::lock_guard lk(resultMutex);
            report = std::move(rep);
            scaleNotice = sen.notice(true);
        }
        finishRun(*this);
    }

    void SixPosCalibTask::renderConfigPanel() {
        renderInsConfig("六位置法标定配置###cfg_", *this, "开始标定",
                        [&] {
                            ImGui::Text("添加任意数量的 IMU 日志文件，程序自动识别每个文件里的静置段与旋转段。");
                            ImGui::Spacing();

                            if (ImGui::Button("添加文件…", ImVec2(140, 0))) {
                                if (std::vector<std::wstring> wpaths; ShowOpenFilesDialog(
                                    wpaths, Solvers::Helpers::fIns, std::size(Solvers::Helpers::fIns))) {
                                    for (auto &wp: wpaths) {
                                        std::string p = wideToAcp(wp);
                                        if (!p.empty() &&
                                            std::find(filePaths.begin(), filePaths.end(), p) == filePaths.end())
                                            filePaths.push_back(p);
                                        AppConfig::PathMemory::instance().addRecent("ins_cal_files", p);
                                    }
                                }
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("从记录添加", ImVec2(120, 0)))
                                ImGui::OpenPopup("##cal_hist_pop");
                            ImGui::SameLine();
                            if (ImGui::Button("清空", ImVec2(80, 0)) && !filePaths.empty())
                                filePaths.clear();

                            if (ImGui::BeginPopup("##cal_hist_pop")) {
                                const auto recent =
                                        AppConfig::PathMemory::instance().getRecent("ins_cal_files");
                                if (recent.empty()) ImGui::TextDisabled("（暂无记录）");
                                for (auto &r: recent) {
                                    if (ImGui::Selectable(Solvers::Helpers::baseName(r).c_str())) {
                                        if (std::find(filePaths.begin(), filePaths.end(), r) == filePaths.end())
                                            filePaths.push_back(r);
                                    }
                                }
                                ImGui::EndPopup();
                            }

                            ImGui::Spacing();
                            ImGui::Separator();
                            if (filePaths.empty()) {
                                ImGui::TextDisabled("尚未添加任何文件。点击「添加文件…」选择标定日志。");
                            } else {
                                ImGui::Text("已选 %d 个文件：", static_cast<int>(filePaths.size()));
                                for (size_t i = 0; i < filePaths.size(); ++i) {
                                    ImGui::Bullet();
                                    ImGui::Text("%s", Solvers::Helpers::baseName(filePaths[i]).c_str());
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton(("x##cal" + std::to_string(i)).c_str())) {
                                        filePaths.erase(filePaths.begin() + i);
                                        --i;
                                    }
                                }
                            }
                        },
                        [&] {
                            renderInsParams(par);
                        },
                        [this] { startCalib(); });
    }

    void SixPosCalibTask::renderResult() {
        const bool running = state == State::Running;
        const bool done = hasResult.load();
        const char *kPending = running ? "计算中" : "—";
        const bool blocked = renderResultPrologue(*this, false, [this] { startCalib(); });
        const bool detReady = detectDone.load();
        if (detReady) {
            ImGui::Spacing();
            ImGui::Text("已识别静置段 %d 个、旋转段 %d 个，跳过重复文件 %d 个，共处理 %lld 个历元",
                        segStatic.load(), segRot.load(), dupSkipped.load(),
                        static_cast<long long>(sampleCount.load()));
        }
        if (detReady && ImGui::Button("导出分段摘要 CSV", ImVec2(200, 0))) exportDetectCsv();
        if (blocked) return;
        if (running) {
            std::string label;
            {
                std::lock_guard lk(resultMutex);
                label = currentFile;
            }
            ImGui::TextColored(kColInfo, "正在解析 %s：静置段 %d 个、旋转段 %d 个、重复 %d 个、%lld 个历元",
                               label.empty() ? "—" : label.c_str(),
                               segStatic.load(), segRot.load(), dupSkipped.load(),
                               static_cast<long long>(sampleCount.load()));
            if (ImGui::Button("停止")) stop = true;
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
            ImGui::TableSetupColumn("量", ImGuiTableColumnFlags_WidthStretch, 3.4f);
            ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableSetupColumn("单位", ImGuiTableColumnFlags_WidthStretch, 1.7f);
            ImGui::TableHeadersRow();

            auto unitCombo = [](const int q) {
                const CalibQty &def = kCalibQty[q];
                ImGui::PushID(q);
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::BeginCombo("##unit", def.units[gCalibQtyUnit[q]].ui)) {
                    for (int i = 0; i < def.nUnits; ++i) {
                        ImGui::PushID(i);
                        if (ImGui::Selectable(def.units[i].ui, i == gCalibQtyUnit[q])) gCalibQtyUnit[q] = i;
                        ImGui::PopID();
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopID();
            };

            auto qtyRow = [&](const int q, const bool ready) {
                const CalibQty &def = kCalibQty[q];
                const CalibUnit &u = def.units[gCalibQtyUnit[q]];
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
                ImGui::TextUnformatted(def.ui);
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
                else ImGui::TextDisabled("%s", def.units[0].ui);
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
            ImGui::SeparatorText("加速度计交轴耦合矩阵 Na");
            if (ImGui::BeginTable("##cross_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("");
                ImGui::TableSetupColumn("输入 X");
                ImGui::TableSetupColumn("输入 Y");
                ImGui::TableSetupColumn("输入 Z");
                ImGui::TableHeadersRow();
                static const char *kOut[3] = {"输出 X", "输出 Y", "输出 Z"};
                for (int i = 0; i < 3; ++i) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(kOut[i]);
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
            ImGui::SeparatorText("陀螺交轴耦合矩阵 Ng");
            if (ImGui::BeginTable("##gcross_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableSetupColumn("");
                ImGui::TableSetupColumn("输入 X");
                ImGui::TableSetupColumn("输入 Y");
                ImGui::TableSetupColumn("输入 Z");
                ImGui::TableHeadersRow();
                static const char *kOutG[3] = {"输出 X", "输出 Y", "输出 Z"};
                for (int i = 0; i < 3; ++i) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(kOutG[i]);
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
            ImGui::SeparatorText("质量判读");
            if (ImGui::BeginTable("##qc_tbl", 4,
                                  ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn("检查项", ImGuiTableColumnFlags_WidthStretch, 1.9f);
                ImGui::TableSetupColumn("判据", ImGuiTableColumnFlags_WidthStretch, 1.6f);
                ImGui::TableSetupColumn("实测", ImGuiTableColumnFlags_WidthStretch, 2.6f);
                ImGui::TableSetupColumn("结论", ImGuiTableColumnFlags_WidthStretch, 0.8f);
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
                    else ImGui::TextColored(ok ? kColPass : kColFail, "%s", ok ? "合格" : "超限");
                };
                char v[128];
                const bool okG = aReady && meanG > 1.0 && spreadG / meanG < 0.05;
                std::snprintf(v, sizeof(v), "%.2f %%", meanG > 0.0 ? spreadG / meanG * 100.0 : 0.0);
                qcRow("加速度计三轴比例因子一致性", "(max−min)/mean < 5 %", aReady, v, okG);

                const bool okRms = aReady && arm < 0.02 * meanG;
                std::snprintf(v, sizeof(v), "%.2f LSB", arm);
                qcRow("六位置拟合残差 RMS", "RMS < 2 % · mean K_a", aReady, v, okRms);

                const bool okDs = hResult && maxds < 0.05;
                std::snprintf(v, sizeof(v), "%.3f %%", maxds * 100.0);
                qcRow("陀螺比例因子误差", "max|δs_g| < 5 %", hResult, v, okDs);

                const double maxNg = rep.gyroCross.cwiseAbs().maxCoeff();
                const bool okNg = hResult && maxNg < 1e-2;
                std::snprintf(v, sizeof(v), "%.3f %%", maxNg * 100.0);
                qcRow("陀螺交轴耦合", "max|N_g| < 1 %", hResult, v, okNg);
                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        ImGui::SeparatorText("比例因子一致性");
        const double scA[3] = {accS(0), accS(1), accS(2)};
        const double scG[3] = {rep.gyroKeff(0), rep.gyroKeff(1), rep.gyroKeff(2)};
        twoCol([&] { drawScaleBars("加速度计三轴比例因子 (LSB/g)", aReady, scA, kColZ, "%.1f"); },
               [&] { drawScaleBars("陀螺有效比例因子 K_eff,g (LSB/rad)", hResult, scG, kColZ, "%.0f"); });


        ImGui::Spacing();
        {

            ImGui::BeginDisabled(!done);
            const bool clicked = ImGui::Button("导出标定结果 CSV", ImVec2(180, 0));
            const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
            ImGui::EndDisabled();
            if (!done && hovered) ImGui::SetTooltip("完成一次标定后才能导出。");
            if (clicked) exportCalibCsv();
        }

        ImGui::SeparatorText("补偿序列");

        if (const auto segLabels = compSegLabels(); segLabels.empty()) {
            ImGui::TextDisabled("标定完成后可按段导出、按段查看补偿结果。");
        } else {
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
            if (ImGui::Button("导出该段补偿序列", ImVec2(210, 0))) exportCompCsv();


            if (CalibReport r; peekReport(r)) {
                ImuLog seg;
                {

                    std::lock_guard lk(resultMutex);
                    const int np = static_cast<int>(compSegs.pos.size());
                    const int nr = static_cast<int>(compSegs.rot.size());
                    const int k = std::clamp(compSel, 0, np + nr - 1);
                    decimateImu(k < np ? compSegs.pos[k].second : compSegs.rot[k - np].second, seg);
                }
                if (seg.empty()) {
                    ImGui::TextDisabled("该段没有样本，请重新标定后再看。");
                } else {
                    const ImuCalibrator::CompMat cm = ImuCalibrator::compMatrices(r);
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
                        ImuCalibrator::compensate(cm, r, smp, _f, _g);
                        fx.push_back(_f(0));
                        fy.push_back(_f(1));
                        fz.push_back(_f(2));
                        gx.push_back(_g(0));
                        gy.push_back(_g(1));
                        gz.push_back(_g(2));
                    }
                    std::vector<double> tel;
                    toElapsed(t, tel);
                    twoCol(
                        [&] { drawCompChart("补偿后比力 f (m/s²)", tel, fx, fy, fz, chartNew); },
                        [&] { drawCompChart("补偿后角速率 ω (LSB，零偏与交轴已扣)", tel, gx, gy, gz, chartNew); });
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
        setError("尚无标定结果，无法导出。请先完成一次标定。");
        return false;
    }

    void SixPosCalibTask::exportCalibCsv() {
        CalibReport r;
        if (!snapshotReport(r)) return;
        exportCsvDialog(*this, "calib_result.csv", "calib_result.csv", [&](Csv::Writer &w) {
            w.row("量", "X", "Y", "Z");

            auto qtyRow = [&](const int q) {
                const CalibQty &def = kCalibQty[q];
                const CalibUnit &u = def.units[gCalibQtyUnit[q]];
                const Eigen::Vector3d v = calibQtyValue(def, u, r, par.g);
                const std::string name = std::string(def.csv) + "(" + u.csv + ")";
                const int d = def.csvDigits;
                if (def.scalar) {
                    if (def.csvSci) w.row(name, Csv::sci(v(0), d), "", "");
                    else w.row(name, Csv::fx(v(0), d), "", "");
                } else if (def.csvSci) {
                    w.row(name, Csv::sci(v(0), d), Csv::sci(v(1), d), Csv::sci(v(2), d));
                } else {
                    w.row(name, Csv::fx(v(0), d), Csv::fx(v(1), d), Csv::fx(v(2), d));
                }
            };

            qtyRow(Q_ACC_SCALE);
            qtyRow(Q_ACC_SCALE_SIGMA);
            qtyRow(Q_ACC_BIAS);
            for (int k = 0; k < 3; ++k) {
                const std::string rn = "M" + std::to_string(k) + "0, M" + std::to_string(k) + "1, M" +
                                       std::to_string(k) + "2";
                w.row(rn, Csv::fx(r.calib.accelM(k, 0), 6), Csv::fx(r.calib.accelM(k, 1), 6),
                      Csv::fx(r.calib.accelM(k, 2), 6));
            }
            qtyRow(Q_ACC_KNOM);
            qtyRow(Q_ACC_DS);
            for (int k = 0; k < 3; ++k) {
                const std::string rn = "N" + std::to_string(k) + "0, N" + std::to_string(k) + "1, N" +
                                       std::to_string(k) + "2";
                w.row(rn, Csv::sci(r.accelCross(k, 0), 9), Csv::sci(r.accelCross(k, 1), 9),
                      Csv::sci(r.accelCross(k, 2), 9));
            }
            qtyRow(Q_GYRO_BIAS);
            qtyRow(Q_GYRO_KEFF);
            qtyRow(Q_GYRO_KEFF_NOM);
            qtyRow(Q_GYRO_SCALE);
            qtyRow(Q_GYRO_DS);

            for (int k = 0; k < 3; ++k) {
                const std::string rn = "Ng" + std::to_string(k) + "0, Ng" + std::to_string(k) + "1, Ng" +
                                       std::to_string(k) + "2";
                w.row(rn, Csv::sci(r.gyroCross(k, 0), 9), Csv::sci(r.gyroCross(k, 1), 9),
                      Csv::sci(r.gyroCross(k, 2), 9));
            }
            w.row("dt(s)", Csv::fx(r.dt, 6));
            w.row("六位置拟合RMS(LSB)", Csv::fx(r.accelRms, 4));
            w.row("nPos(每位置历元数)", r.nPos[0], r.nPos[1], r.nPos[2], r.nPos[3], r.nPos[4], r.nPos[5]);
            w.row("nRot(每旋转历元数)", r.nRot[0], r.nRot[1], r.nRot[2], r.nRot[3], r.nRot[4], r.nRot[5]);
        });
    }

    void SixPosCalibTask::exportDetectCsv() {
        std::vector<std::pair<std::string, std::string> > seq;
        {
            std::lock_guard lk(resultMutex);
            seq = detectSeq;
        }
        if (seq.empty()) {
            setError("尚无分段结果，无法导出。");
            return;
        }
        exportCsvDialog(*this, "detect_summary.csv", "detect_summary.csv", [&](Csv::Writer &w) {
            w.row("文件", "识别到的静置/旋转段");
            for (const auto &[fst, snd]: seq) w.row(fst, snd);
        });
    }

    std::vector<std::string> SixPosCalibTask::compSegLabels() {
        std::lock_guard lk(resultMutex);
        std::vector<std::string> v;
        v.reserve(compSegs.pos.size() + compSegs.rot.size());
        for (const auto &[first, _]: compSegs.pos)
            v.emplace_back(std::string("静置 · ") + ImuCalibrator::posLabel(first));
        for (const auto &[first, _]: compSegs.rot)
            v.emplace_back(std::string("旋转 · ") + ImuCalibrator::rotLabel(first));
        std::map<std::string, int> seen;
        for (auto &s: v) {
            if (const int n = ++seen[s]; n > 1) s += " #" + std::to_string(n);
        }
        return v;
    }

    static std::string compPosFileName(const ImuCalibrator::Pos p) {
        static const char *tag[6] = {"xup", "xdn", "yup", "ydn", "zup", "zdn"};
        return std::string("calib_") + tag[static_cast<int>(p)] + "_comp.csv";
    }

    static std::string compRotFileName(const ImuCalibrator::Rot r) {
        static const char *tag[6] = {"rot", "xrev", "yfwd", "yrev", "zfwd", "zrev"};
        return std::string("calib_") + tag[static_cast<int>(r)] + "_comp.csv";
    }

    void SixPosCalibTask::exportCompCsv() {
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
            setError("该段没有样本，无法导出补偿序列。请重新标定后再试。");
            return;
        }
        if (const auto segLabels = compSegLabels();
            selIdx >= 0 && selIdx < static_cast<int>(segLabels.size()))
            label = segLabels[selIdx];

        const ImuCalibrator::CompMat cm = ImuCalibrator::compMatrices(r);
        exportCsvDialog(*this, defName, defName + "（" + label + "）", [&](Csv::Writer &w) {
            w.row("tow", "comp_fx(m/s2)", "comp_fy(m/s2)", "comp_fz(m/s2)",
                  "raw_gx", "raw_gy", "raw_gz", "comp_gx", "comp_gy", "comp_gz");
            for (const auto &s: seg) {
                Eigen::Vector3d f, g;
                ImuCalibrator::compensate(cm, r, s, f, g);
                w.row(Csv::fx(s.tow, 3),
                      Csv::fx(f(0), 5), Csv::fx(f(1), 5), Csv::fx(f(2), 5),
                      s.gx, s.gy, s.gz,
                      Csv::fx(g(0), 4), Csv::fx(g(1), 4), Csv::fx(g(2), 4));
            }
        });
    }
}
