#include "QualityControl.h"
#include "GuiFileProcessor.h"
#include "imgui.h"
#include "implot.h"
#include "Const.h"
#include <mutex>
#include <vector>

namespace QualityControl {

// 把每次需要重算时的「解算结果 → lib/qc 输入」转换放在这里，render 本身只做绘图。
static QC::QualityReport buildReport(const GuiFileProcessor::SppTask &task) {
    std::vector<QC::QCObsEpoch> qcEpochs;
    qcEpochs.reserve(task.epochs.size());
    for (const auto &ep : task.epochs) {
        QC::QCObsEpoch qe;
        qe.sow = ep.sow;
        qe.satIds = ep.satIds;
        qe.allObs = ep.allObs;
        // SppEpochData.elevations/azimuths 为弧度，QC 显示与 Skyplot 用角度，这里转为度
        for (double e : ep.elevations) qe.elevations.push_back(e * RAD_TO_DEG);
        for (double a : ep.azimuths) qe.azimuths.push_back(a * RAD_TO_DEG);
        qe.pdop = ep.sppResult.pdop;
        qe.hdop = ep.sppResult.hdop;
        qe.vdop = ep.sppResult.vdop;
        qe.gdop = ep.sppResult.gdop;
        qe.nsat = ep.sppResult.numSats;
        qcEpochs.push_back(std::move(qe));
    }
    return QC::compute(qcEpochs);
}

void render(const std::shared_ptr<GuiFileProcessor::SppTask> &task) {
    const bool need = (!task->qcReady) || (task->qcEpochCount != (long)task->epochs.size());
    if (need && task->epochs.size() > 1) {
        std::lock_guard lk(task->mutex);
        task->qcReport = buildReport(*task);
        task->qcReady = true;
        task->qcEpochCount = (long)task->epochs.size();
    }
    if (!task->qcReady) { ImGui::Text("正在读取观测数据，质量分析稍后开始…"); return; }
    const QualityReport &rep = task->qcReport;
    if (rep.sats.empty()) { ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "无有效双频观测，无法进行质量分析。"); return; }

    static bool gDetrend = true;
    ImGui::Checkbox("去趋势(逐弧段移除漂移/失锁台阶，凸显小波动)", &gDetrend);
    ImGui::SameLine();
    ImGui::TextDisabled("关：显示原始趋势(保留漂移斜率)");

    static int sysSel = 0;
    ImGui::RadioButton("全部", &sysSel, 0); ImGui::SameLine();
    ImGui::RadioButton("GPS", &sysSel, 1); ImGui::SameLine();
    ImGui::RadioButton("BDS", &sysSel, 2);

    ImGui::SeparatorText("总体概览");
    ImGui::Text("总历元 %d | 间隔 %.2f s | 可用卫星 %d | 总体完整率 %.2f%% | SNR %.1f | 平均高度角 %.1f° | 平均 PDOP %.2f | 平均卫星数 %d",
                rep.totalEpochs, rep.interval, (int)rep.sats.size(), rep.overallCompleteness,
                rep.overallSnr, rep.overallElev, rep.overallPDOP, (int)rep.overallNsat);
    if (ImGui::BeginTable("##sys", 10, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("系统"); ImGui::TableSetupColumn("完整率%"); ImGui::TableSetupColumn("周跳数");
        ImGui::TableSetupColumn("周跳比"); ImGui::TableSetupColumn("MP1(m)"); ImGui::TableSetupColumn("MP2(m)");
        ImGui::TableSetupColumn("伪距噪声(m)"); ImGui::TableSetupColumn("载波噪声(cyc)");
        ImGui::TableSetupColumn("SNR(dB)"); ImGui::TableSetupColumn("高度角°");
        ImGui::TableHeadersRow();
        auto addSys = [&](char s, const char *name) {
            if (!rep.sysCompleteness.count(s)) return;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("%s", name);
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", rep.sysCompleteness.at(s));
            ImGui::TableSetColumnIndex(2); ImGui::Text("%d", rep.sysSlips.at(s));
            ImGui::TableSetColumnIndex(3); ImGui::Text("%.1f", rep.sysSlipRatio.at(s));
            ImGui::TableSetColumnIndex(4); ImGui::Text("%.3f", rep.sysMp1.at(s));
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.3f", rep.sysMp2.at(s));
            ImGui::TableSetColumnIndex(6); ImGui::Text("%.3f", rep.sysSigRho.at(s));
            ImGui::TableSetColumnIndex(7); ImGui::Text("%.4f", rep.sysSigPhase.at(s));
            ImGui::TableSetColumnIndex(8); ImGui::Text("%.1f", rep.sysSnr.count(s) ? rep.sysSnr.at(s) : 0.0);
            ImGui::TableSetColumnIndex(9); ImGui::Text("%.1f", rep.sysElev.count(s) ? rep.sysElev.at(s) : 0.0);
        };
        addSys('G', "GPS"); addSys('C', "BDS");
        ImGui::EndTable();
    }

    ImGui::SeparatorText("逐卫星统计");
    std::vector<SatID> list;
    for (auto &sat : rep.satOrder) {
        if (sysSel == 0 || (sysSel == 1 && sat.system == 'G') || (sysSel == 2 && sat.system == 'C')) list.push_back(sat);
    }
    if (list.empty()) { ImGui::Text("无符合条件的卫星。"); return; }
    static int comboSel = 0;
    if (comboSel < 0 || comboSel > (int)list.size()) comboSel = 0;
    std::string items = "全部"; items += '\0';
    for (auto &s : list) { items += s.toString(); items += '\0'; }
    items += '\0';
    ImGui::Combo("选择卫星", &comboSel, items.c_str());
    const bool allView = (comboSel == 0);

    std::vector<const SatQC *> toPlot;
    if (allView) for (auto &sat : list) toPlot.push_back(&rep.sats.at(sat));
    else if (comboSel >= 1 && comboSel <= (int)list.size()) toPlot.push_back(&rep.sats.at(list[comboSel - 1]));

    if (allView) {
        ImGui::TextDisabled("已选「全部卫星」：下列曲线按卫星叠加（每条线为一个卫星，可点击图例隐藏）。");
        auto plotAll = [&](const char *title, const char *yunit,
                           const std::vector<double> SatQC::*fDetr, const std::vector<double> SatQC::*fRaw) {
            const std::vector<double> SatQC::*field = gDetrend ? fDetr : fRaw;
            if (ImPlot::BeginPlot(title, ImVec2(-1, 360))) {
                ImPlot::SetupAxes("SOW (s)", yunit);
                for (auto q : toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), (q->*field).data(), (int)q->t.size());
                ImPlot::EndPlot();
            }
        };
        plotAll("MP1 (全部卫星)", "m", &SatQC::mp1, &SatQC::mp1Raw);
        plotAll("MP2 (全部卫星)", "m", &SatQC::mp2, &SatQC::mp2Raw);
        plotAll("几何无关组合 L4 (全部卫星)", "周", &SatQC::l4, &SatQC::l4Raw);
        plotAll("电离层延迟残差 I (全部卫星)", "m", &SatQC::iono, &SatQC::ionoRaw);

        // SNR / 高度角（与 RTKLIB 对齐）
        auto plotVec = [&](const char *title, const char *yunit, const std::vector<double> SatQC::*field) {
            if (ImPlot::BeginPlot(title, ImVec2(-1, 360))) {
                ImPlot::SetupAxes("SOW (s)", yunit);
                for (auto q : toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), (q->*field).data(), (int)q->t.size());
                ImPlot::EndPlot();
            }
        };
        plotVec("信噪比 SNR (全部卫星)", "dB-Hz", &SatQC::snr);
        plotVec("卫星高度角 Elevation (全部卫星)", "deg", &SatQC::elev);

        // DOP / Nsat 时序
        std::vector<double> nsatD(rep.nsat.begin(), rep.nsat.end());
        if (!rep.t.empty() && ImPlot::BeginPlot("卫星数 / DOP", ImVec2(-1, 360))) {
            ImPlot::SetupAxes("SOW (s)", "Nsat");
            ImPlot::SetupAxis(ImAxis_Y2, "DOP", ImPlotAxisFlags_AuxDefault);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
            if (!nsatD.empty()) ImPlot::PlotBars("Nsat", rep.t.data(), nsatD.data(), (int)rep.t.size(), rep.interval * 0.8);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            if (!rep.pdop.empty()) ImPlot::PlotLine("PDOP", rep.t.data(), rep.pdop.data(), (int)rep.t.size());
            if (!rep.hdop.empty()) ImPlot::PlotLine("HDOP", rep.t.data(), rep.hdop.data(), (int)rep.t.size());
            if (!rep.vdop.empty()) ImPlot::PlotLine("VDOP", rep.t.data(), rep.vdop.data(), (int)rep.t.size());
            ImPlot::EndPlot();
        }

        // Skyplot：方位角/高度角散点（与 RTKLIB 对齐）
        if (ImPlot::BeginPlot("天空视图 Skyplot", ImVec2(-1, 360), ImPlotFlags_Equal)) {
            ImPlot::SetupAxes("E-W", "N-S", 0, 0);
            ImPlot::SetupAxisLimits(ImAxis_X1, -90, 90);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -90, 90);
            // 画高度角圈 (30°, 60°)
            auto drawCircle = [&](double el, const char *label) {
                int n = 64; static double xs[64], ys[64]; double r = 90.0 - el;
                for (int i = 0; i < n; i++) { double a = i * 2.0 * 3.14159265358979 / n; xs[i] = r * sin(a); ys[i] = r * cos(a); }
                ImPlot::PlotLine(label, xs, ys, n);
            };
            drawCircle(0, "el=0"); drawCircle(30, "el=30"); drawCircle(60, "el=60");
            for (auto q : toPlot) {
                if (q->azim.empty()) continue;
                std::vector<double> xs, ys;
                for (size_t i = 0; i < q->azim.size(); i++) {
                    if (q->elev[i] <= 0 || q->elev[i] > 90) continue;
                    double a = q->azim[i] * 3.14159265358979 / 180.0;
                    double r = 90.0 - q->elev[i];
                    xs.push_back(r * sin(a)); ys.push_back(r * cos(a));
                }
                if (!xs.empty()) ImPlot::PlotScatter(q->sat.toString().c_str(), xs.data(), ys.data(), (int)xs.size());
            }
            ImPlot::EndPlot();
        }
    } else if (!toPlot.empty()) {
        const SatQC &q = *toPlot[0];
        if (ImGui::BeginTable("##satqc", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("指标"); ImGui::TableSetupColumn("值");
            ImGui::TableHeadersRow();
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("频点 f1 / f2");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%c / %c", q.band1, q.band2);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("完整率"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f %% (%d/%d)", q.completeness, q.validDual, q.totalEpochs);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("周跳数 / 周跳比"); ImGui::TableSetColumnIndex(1); ImGui::Text("%d / %.1f", q.slips, q.slipRatio);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("MP1 / MP2 (RMS)"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f / %.3f m", q.mp1Rms, q.mp2Rms);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("伪距噪声"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f / %.3f m", q.sigRho1, q.sigRho2);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("载波相位噪声"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f / %.4f cyc", q.sigPh1, q.sigPh2);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("电离层残差"); ImGui::TableSetColumnIndex(1); ImGui::Text("std %.3f m, 跳变 %d 次", q.ionoStd, q.ionoJumps);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("SNR (mean/min/max)"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.1f / %.1f / %.1f dB-Hz", q.snrMean, q.snrMin, q.snrMax);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("高度角 (mean/min/max)"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.1f / %.1f / %.1f deg", q.elevMean, q.elevMin, q.elevMax);
            ImGui::EndTable();
        }
        if (!q.t.empty()) {
            const std::vector<double> &mp1v = gDetrend ? q.mp1 : q.mp1Raw;
            const std::vector<double> &mp2v = gDetrend ? q.mp2 : q.mp2Raw;
            const std::vector<double> &l4v  = gDetrend ? q.l4  : q.l4Raw;
            const std::vector<double> &ionov = gDetrend ? q.iono : q.ionoRaw;
            if (ImPlot::BeginPlot("多路径 MP1 / MP2", ImVec2(-1, 360))) {
                ImPlot::SetupAxes("SOW (s)", "m");
                ImPlot::PlotLine("MP1", q.t.data(), mp1v.data(), (int)q.t.size());
                ImPlot::PlotLine("MP2", q.t.data(), mp2v.data(), (int)q.t.size());
                ImPlot::EndPlot();
            }
            std::vector<double> sl_t, sl_v;
            for (int i = 0; i < (int)q.t.size(); i++) if (q.slipFlag[i]) { sl_t.push_back(q.t[i]); sl_v.push_back(l4v[i]); }
            if (ImPlot::BeginPlot("几何无关组合 L4 (电离层残差 / 周跳)", ImVec2(-1, 360))) {
                ImPlot::SetupAxes("SOW (s)", "周");
                ImPlot::PlotLine("L4", q.t.data(), l4v.data(), (int)q.t.size());
                if (!sl_t.empty()) ImPlot::PlotScatter("周跳", sl_t.data(), sl_v.data(), (int)sl_t.size());
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("电离层延迟残差 I", ImVec2(-1, 360))) {
                ImPlot::SetupAxes("SOW (s)", "m");
                ImPlot::PlotLine("I", q.t.data(), ionov.data(), (int)q.t.size());
                ImPlot::EndPlot();
            }
            if (!q.snr.empty() && ImPlot::BeginPlot("信噪比 SNR / 高度角 Elevation", ImVec2(-1, 360))) {
                ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                ImPlot::SetupAxis(ImAxis_Y2, "deg", ImPlotAxisFlags_AuxDefault);
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
                ImPlot::PlotLine("SNR", q.t.data(), q.snr.data(), (int)q.t.size());
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
                ImPlot::PlotLine("Elevation", q.t.data(), q.elev.data(), (int)q.t.size());
                ImPlot::EndPlot();
            }
        }
    }
}

} // namespace QualityControl
