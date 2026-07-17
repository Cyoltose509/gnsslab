#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <filesystem>
#include "QualityControl.h"
#include "ui/Gui.h"
#include "GuiFileProcessor.h"
#include "imgui.h"
#include "implot.h"
#include "Const.h"
#include <mutex>
#include <vector>
#include <functional>
#include <fstream>
#include <iomanip>
#include  <StringUtils.h>


namespace QualityControl {
    // ---- 统一默认图高 ----
    constexpr float kPlotH = 420.0f;

    // ---- 批量导出状态 ----
    struct QcUiState {
        std::map<std::string, ImVec4> curRect;
        std::map<std::string, float> contentY;
        std::vector<std::string> lastIds;
        std::vector<std::string> exportQueue;
        std::string exportDir;
        bool exporting = false;
    };

    static QcUiState &uiState() {
        static QcUiState s;
        return s;
    }

    static const char *sysName(const char s) {
        switch (s) {
            case 'G': return "GPS";
            case 'C': return "BDS";
            case 'R': return "GLO";
            case 'E': return "GAL";
            default: return "?";
        }
    }


    // 将质量分析报告导出为 CSV（含系统汇总、逐卫星统计两段）
    static void exportCsv(const QualityReport &rep, const std::string &taskLabel) {
        static constexpr GuiFileFilter filters[] = {
            {L"CSV 文件 (*.csv)", L"*.csv"}, {L"所有文件 (*.*)", L"*.*"}
        };
        std::wstring path;
        const std::wstring def = makeWide(stripExt(taskLabel) + "_qc.csv");
        if (!ShowSaveFileDialog(path, filters, 2, def.c_str(), L"csv")) return; // 用户取消

        std::ofstream out(std::filesystem::path(path), std::ios::out);
        if (!out) return;
        out << "\xEF\xBB\xBF"; // UTF-8 BOM
        out << std::fixed << std::setprecision(3);

        out << "GNSS 观测数据质量分析报告," << taskLabel << "\n";
        out << "总历元," << rep.totalEpochs << ",间隔(s)," << rep.interval
                << ",可用卫星," << rep.sats.size() << ",总体完整率%," << rep.overallCompleteness
                << ",平均CNR(dB-Hz)," << rep.overallCnr << "\n\n";

        out << "[系统汇总]\n";
        out << "系统,完整率%,DI_f1%,DI_f2%,周跳数,周跳比,粗差数,钟跳数,MP1(m),MP2(m),伪距噪声(m),载波噪声(cyc),"
                "CNR_all(dB-Hz),CNR_f1(dB-Hz),CNR_f2(dB-Hz),IOD_std_all(m/s),IOD_std_f1(m/s),IOD_std_f2(m/s),IOD_跳变,IOD_跳变_f1,IOD_跳变_f2,综合E\n";
        for (auto &[s, comp]: rep.sysCompleteness) {
            out << sysName(s) << ","
                    << comp << ","
                    << (rep.sysDI_f1.count(s) ? rep.sysDI_f1.at(s) : 0.0) << ","
                    << (rep.sysDI_f2.count(s) ? rep.sysDI_f2.at(s) : 0.0) << ","
                    << rep.sysSlips.at(s) << "," << rep.sysSlipRatio.at(s)
                    << "," << (rep.sysOutliers.count(s) ? rep.sysOutliers.at(s) : 0) << ","
                    << (rep.sysClockJumps.count(s) ? rep.sysClockJumps.at(s) : 0) << ","
                    << rep.sysMp1.at(s) << "," << rep.sysMp2.at(s)
                    << "," << rep.sysSigRho.at(s) << "," << rep.sysSigPhase.at(s)
                    << "," << (rep.sysCnr.count(s) ? rep.sysCnr.at(s) : 0.0)
                    << "," << (rep.sysCnr1.count(s) ? rep.sysCnr1.at(s) : 0.0)
                    << "," << (rep.sysCnr2.count(s) ? rep.sysCnr2.at(s) : 0.0)
                    << "," << (rep.sysIonoRate.count(s) ? rep.sysIonoRate.at(s) : 0.0)
                    << "," << (rep.sysIonoRate1.count(s) ? rep.sysIonoRate1.at(s) : 0.0)
                    << "," << (rep.sysIonoRate2.count(s) ? rep.sysIonoRate2.at(s) : 0.0)
                    << "," << (rep.sysIonoJumps.count(s) ? rep.sysIonoJumps.at(s) : 0)
                    << "," << (rep.sysIonoJumps1.count(s) ? rep.sysIonoJumps1.at(s) : 0)
                    << "," << (rep.sysIonoJumps2.count(s) ? rep.sysIonoJumps2.at(s) : 0)
                    << "," << (rep.comprehensive.sysE.count(s) ? rep.comprehensive.sysE.at(s) : 0.0) << "\n";
        }
        out << "\n";

        out << "[逐卫星统计]\n";
        out << "卫星,系统,频点f1,频点f2,完整率%,DI_f1%,DI_f2%,周跳数,周跳比,粗差数,钟跳数,MP1(m),MP2(m),"
                "伪距噪声_f1(m),伪距噪声_f2(m),载波噪声_f1(cyc),载波噪声_f2(cyc),"
                "CNR_f1(dB-Hz),CNR_f2(dB-Hz),IOD_std_f1(m/s),IOD_std_f2(m/s),IOD_跳变_f1,IOD_跳变_f2\n";
        for (auto &sat: rep.satOrder) {
            const auto &q = rep.sats.at(sat);
            out << q.sat.toString() << "," << sysName(q.sat.system) << ","
                    << q.band1 << "," << q.band2 << ","
                    << q.completeness << "," << q.di_f1 << "," << q.di_f2 << ","
                    << q.slips << "," << q.slipRatio << "," << q.outliers << "," << q.clockJumps << ","
                    << q.mp1Rms << "," << q.mp2Rms
                    << "," << q.sigRho1 << "," << q.sigRho2
                    << "," << q.sigPh1 << "," << q.sigPh2
                    << "," << q.cnrMean << "," << q.cnrMean2
                    << "," << q.ionoRateStd1 << "," << q.ionoRateStd2
                    << "," << q.ionoJumps1 << "," << q.ionoJumps2 << "\n";
        }
        out << "\n";

        out << "[综合评估]\n";
        out << "整体综合E," << rep.comprehensive.overallE << "\n";
        out << "系统综合E,";
        for (auto &[s, e]: rep.comprehensive.sysE) out << sysName(s) << "=" << e << ",";
        out << "\n";
        out << "指标,";
        for (auto &ind: rep.comprehensive.indicators) out << ind << ",";
        out << "\n权重,";
        for (const double w: rep.comprehensive.weights) out << w << ",";
        out << "\n\n";
        out << "[逐卫星综合评估]\n";
        out << "卫星,系统";
        for (auto &ind: rep.comprehensive.indicators) out << "," << ind;
        out << ",E\n";
        for (auto &sat: rep.satOrder) {
            out << sat.toString() << "," << sysName(sat.system);
            if (rep.comprehensive.z.count(sat))
                for (const double zv: rep.comprehensive.z.at(sat)) out << "," << zv;
            else
                for (size_t l = 0; l < rep.comprehensive.indicators.size(); l++) out << ",0";
            out << "," << (rep.comprehensive.E.count(sat) ? rep.comprehensive.E.at(sat) : 0.0) << "\n";
        }
        out << "\n";
        out.flush();
    }

    // 「导出此图 PNG」小按钮
    static void plotExportBtn(const char *uid, const ImVec2 p0, const float w, const float h, const std::string &base) {
        const std::string lbl = std::string("导出此图 PNG##") + uid;
        if (ImGui::SmallButton(lbl.c_str())) {
            static constexpr GuiFileFilter filters[] = {
                {L"PNG 文件 (*.png)", L"*.png"}, {L"所有文件 (*.*)", L"*.*"}
            };
            const std::wstring def = makeWide(base + "_" + uid + ".png");
            if (std::wstring path; ShowSaveFileDialog(path, filters, 2, def.c_str(), L"png"))
                RequestCaptureRegionPNG(path, static_cast<int>(p0.x), static_cast<int>(p0.y), static_cast<int>(w), static_cast<int>(h));
        }
    }

    void render(const std::shared_ptr<GuiFileProcessor::SppTask> &task) {
        std::shared_ptr<QualityReport> repPtr;
        {
            std::lock_guard lk(task->qcMutex);
            repPtr = task->qcReport;
        }
        if (!repPtr || !task->qcReady) {
            if (task->qcComputing) {
                ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "正在计算质量分析…");
            } else {
                ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "正在准备质量分析…");
            }
            return;
        }
        const QualityReport &rep = *repPtr;
        if (rep.sats.empty()) {
            ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "无有效双频观测，无法进行质量分析。");
            return;
        }
        QcUiState &qs = uiState(); //NOLINT

        ImGui::BeginChild("##qcplots", ImVec2(0, 0));

        // 批量导出时把队列首图滚动到视口顶部
        if (qs.exporting && !qs.exportQueue.empty()) {
            if (auto it = qs.contentY.find(qs.exportQueue.front()); it != qs.contentY.end())
                ImGui::SetScrollY(it->second - 6.0f);
        }

        static int sysSel = 0;
        ImGui::RadioButton("全部", &sysSel, 0);
        ImGui::SameLine();
        ImGui::RadioButton("GPS", &sysSel, 1);
        ImGui::SameLine();
        ImGui::RadioButton("BDS", &sysSel, 2);

        ImGui::SeparatorText("总体概览");

        ImGui::Text("总历元 %d | 间隔 %.2f s | 可用卫星 %d 接收机钟跳 %d 历元",
                    rep.totalEpochs, rep.interval, static_cast<int>(rep.sats.size()), rep.clockJumpEpochs);
        // ScrollX 表必须给定 outer_size.y，否则会撑满父窗口全部高度（表现为超大空表）
        const float rowH = ImGui::GetTextLineHeightWithSpacing();
        const int sysRows = static_cast<int>(rep.sysCompleteness.size()) + 1; // +1 总体行
        if (const ImVec2 sysTblSize(0.0f, rowH * static_cast<float>(sysRows + 1) + 8.0f); ImGui::BeginTable(
            "##sys", 13, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX |
                         ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit, sysTblSize)) {
            ImGui::TableSetupColumn("系统");
            ImGui::TableSetupColumn("完整率%");
            ImGui::TableSetupColumn("周跳数");
            ImGui::TableSetupColumn("周跳比");
            ImGui::TableSetupColumn("粗差数");
            ImGui::TableSetupColumn("钟跳数");
            ImGui::TableSetupColumn("MP1(m)");
            ImGui::TableSetupColumn("MP2(m)");
            ImGui::TableSetupColumn("伪距噪声(m)");
            ImGui::TableSetupColumn("载波噪声(cyc)");
            ImGui::TableSetupColumn("CNR_f1(dB-Hz)");
            ImGui::TableSetupColumn("CNR_f2(dB-Hz)");
            ImGui::TableSetupColumn("综合评估 E");
            ImGui::TableHeadersRow();
            // 在"综合评估 E"列头显示熵权 tooltip
            if (ImGui::TableGetColumnFlags(12) & ImGuiTableColumnFlags_IsHovered) {
                if (ImGui::BeginTooltip()) {
                    const auto &ce = rep.comprehensive;
                    ImGui::TextDisabled("熵权 (越小越好 E)");
                    for (size_t l = 0; l < ce.indicators.size(); l++)
                        ImGui::Text("%s: %.4f", ce.indicators[l].c_str(), ce.weights[l]);
                    ImGui::EndTooltip();
                }
            }
            auto addSys = [&](const char s, const char *name) {
                if (!rep.sysCompleteness.count(s)) return;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%s", name);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", rep.sysCompleteness.at(s));
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%d", rep.sysSlips.at(s));
                ImGui::TableSetColumnIndex(3);
                if (rep.sysSlips.at(s) > 0) ImGui::Text("%.1f", rep.sysSlipRatio.at(s));
                else ImGui::Text("∞");
                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%d", rep.sysOutliers.count(s) ? rep.sysOutliers.at(s) : 0);
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%d", rep.sysClockJumps.count(s) ? rep.sysClockJumps.at(s) : 0);
                ImGui::TableSetColumnIndex(6);
                ImGui::Text("%.3f", rep.sysMp1.at(s));
                ImGui::TableSetColumnIndex(7);
                ImGui::Text("%.3f", rep.sysMp2.at(s));
                ImGui::TableSetColumnIndex(8);
                ImGui::Text("%.3f", rep.sysSigRho.at(s));
                ImGui::TableSetColumnIndex(9);
                ImGui::Text("%.4f", rep.sysSigPhase.at(s));
                ImGui::TableSetColumnIndex(10);
                ImGui::Text("%.1f", rep.sysCnr1.count(s) ? rep.sysCnr1.at(s) : 0.0);
                ImGui::TableSetColumnIndex(11);
                ImGui::Text("%.1f", rep.sysCnr2.count(s) ? rep.sysCnr2.at(s) : 0.0);
                ImGui::TableSetColumnIndex(12);
                if (rep.comprehensive.sysE.count(s))
                    ImGui::Text("%.4f", rep.comprehensive.sysE.at(s));
                else
                    ImGui::Text("-");
            };
            addSys('G', "GPS");
            addSys('C', "BDS");
            // 总体行：计数按系统累加，连续指标按所有卫星直接平均（与顶部 CNR 一致）
            if (!rep.sysCompleteness.empty()) {
                int totalSlips = 0, totalOutliers = 0, totalClockJumps = 0;
                for (const auto &[s, c]: rep.sysSlips) totalSlips += c;
                for (const auto &[s, c]: rep.sysOutliers) totalOutliers += c;
                for (const auto &[s, c]: rep.sysClockJumps) totalClockJumps += c;
                double sumMp1 = 0, sumMp2 = 0, sumSigRho = 0, sumSigPh = 0;
                int cnt = 0;
                for (const auto &[sat, q]: rep.sats) {
                    sumMp1 += q.mp1Rms;
                    sumMp2 += q.mp2Rms;
                    sumSigRho += 0.5 * (q.sigRho1 + q.sigRho2);
                    sumSigPh += 0.5 * (q.sigPh1 + q.sigPh2);
                    cnt++;
                }
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("总体");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", rep.overallCompleteness);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%d", totalSlips);
                ImGui::TableSetColumnIndex(3);
                if (totalSlips > 0) ImGui::Text("%.1f", static_cast<double>(rep.totalEpochs) / totalSlips);
                else ImGui::Text("∞");
                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%d", totalOutliers);
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%d", totalClockJumps);
                ImGui::TableSetColumnIndex(6);
                ImGui::Text("%.3f", cnt ? sumMp1 / cnt : 0.0);
                ImGui::TableSetColumnIndex(7);
                ImGui::Text("%.3f", cnt ? sumMp2 / cnt : 0.0);
                ImGui::TableSetColumnIndex(8);
                ImGui::Text("%.3f", cnt ? sumSigRho / cnt : 0.0);
                ImGui::TableSetColumnIndex(9);
                ImGui::Text("%.4f", cnt ? sumSigPh / cnt : 0.0);
                ImGui::TableSetColumnIndex(10);
                ImGui::Text("%.1f", rep.overallCnr1);
                ImGui::TableSetColumnIndex(11);
                ImGui::Text("%.1f", rep.overallCnr2);
                ImGui::TableSetColumnIndex(12);
                ImGui::Text("%.4f", rep.comprehensive.overallE);
            }
            ImGui::EndTable();
        }


        std::string baseName = stripExt(task->fileName);
        if (ImGui::Button("导出 CSV (系统汇总/逐卫星)")) {
            exportCsv(rep, task->fileName);
        }
        ImGui::SameLine();
        if (ImGui::Button("一键导出全部图 PNG ")) {
            if (std::wstring dir; ShowFolderDialog(dir)) {
                qs.exportDir = wideToUtf8(dir);
                qs.exportQueue = qs.lastIds;
                qs.exporting = !qs.exportQueue.empty();
            }
        }


        if (qs.exporting)
            ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1), "正在批量导出：剩余 %d 张（每帧 1 张，请勿操作）…",
                               static_cast<int>(qs.exportQueue.size()));
        else if (!qs.exportDir.empty())
            ImGui::TextDisabled("上次导出目录：%s", qs.exportDir.c_str());

        ImGui::SeparatorText("逐卫星统计");
        std::vector<SatID> list;
        for (auto &sat: rep.satOrder) {
            if (sysSel == 0 || (sysSel == 1 && sat.system == 'G') || (sysSel == 2 && sat.system == 'C')) list.push_back(sat);
        }
        if (list.empty()) {
            ImGui::Text("无符合条件的卫星。");
            ImGui::EndChild();
            return;
        }
        static int comboSel = 0;
        if (comboSel < 0 || comboSel > static_cast<int>(list.size())) comboSel = 0;
        std::string items = "全部";
        items += '\0';
        for (auto &s: list) {
            items += s.toString();
            items += '\0';
        }
        items += '\0';
        ImGui::Combo("选择卫星", &comboSel, items.c_str());
        const bool allView = comboSel == 0;

        std::vector<const SatQC *> toPlot;
        if (allView) for (auto &sat: list) toPlot.push_back(&rep.sats.at(sat));
        else if (comboSel >= 1 && comboSel <= static_cast<int>(list.size())) toPlot.push_back(&rep.sats.at(list[comboSel - 1]));


        qs.curRect.clear();
        qs.lastIds.clear();
        static std::map<std::string, std::pair<std::string, std::function<void()> > > plotDefs;


        auto drawPlot = [&](const char *id, const char *title, const float defH,
                            const std::function<void()> &body) {
            auto h = defH;
            if (h < 80.0f) h = 80.0f;

            const float w = ImGui::GetContentRegionAvail().x; // 宽度始终占满

            qs.contentY[id] = ImGui::GetCursorPosY();
            qs.lastIds.emplace_back(id);
            plotDefs[id] = {title, body};

            if (ImGui::BeginChild((std::string("##frame_") + id).c_str(),
                                  ImVec2(w, h), ImGuiChildFlags_Borders)) {
                if (ImPlot::BeginPlot(title, ImVec2(-1, -1))) {
                    body();
                    ImPlot::EndPlot();
                }
            }
            ImGui::EndChild();

            // 用 EndChild 后的精确屏幕矩形作为导出区域（含边框）
            const ImVec2 rmin = ImGui::GetItemRectMin(), rmax = ImGui::GetItemRectMax();
            qs.curRect[id] = ImVec4(rmin.x, rmin.y, rmax.x - rmin.x, rmax.y - rmin.y);

            plotExportBtn(id, rmin, rmax.x - rmin.x, rmax.y - rmin.y, baseName);
        };

        if (allView) {
            drawPlot("mp1_all", "多路径误差 MP1", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->mp1Resid.data(), static_cast<int>(q->t.size()));
            });
            drawPlot("mp2_all", "多路径误差 MP2", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->mp2Resid.data(), static_cast<int>(q->t.size()));
            });
            drawPlot("iod_all", "电离层残差变化率 IOD", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m/s");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->ionoRate.data(), static_cast<int>(q->t.size()));
            });
            drawPlot("cnr_all", "载噪比 CNR", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->cnr.data(), static_cast<int>(q->t.size()));
            });

            // ===== 逐卫星指标柱状图 =====
            ImGui::SeparatorText("逐卫星指标");
            drawPlot("completeness", "观测完整率 (%)", kPlotH, [&] {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, "%");
                const auto M = static_cast<int>(list.size());
                static std::vector<double> xs, ys;
                static std::vector<std::string> labs;
                static std::vector<const char *> labp;
                xs.resize(M);
                ys.resize(M);
                labs.resize(M);
                labp.resize(M);
                for (int i = 0; i < M; i++) {
                    const auto &q = rep.sats.at(list[i]);
                    xs[i] = i;
                    ys[i] = q.completeness;
                    labs[i] = q.sat.toString();
                    labp[i] = labs[i].c_str();
                }
                ImPlot::SetupAxisTicks(ImAxis_X1, xs.data(), M, labp.data());
                ImPlot::PlotBars("观测完整率", ys.data(), M, 0.7);
            });
            // 周跳比
            {
                const int M = static_cast<int>(list.size());
                static std::vector<double> jx, jy;
                static std::vector<std::string> jl;
                static std::vector<const char *> jlp;
                jx.clear();
                jy.clear();
                jl.clear();
                jlp.clear();
                for (int i = 0; i < M; i++) {
                    if (const auto &q = rep.sats.at(list[i]); q.slips > 0) {
                        jx.push_back(static_cast<double>(jlp.size()));
                        jy.push_back(q.slipRatio);
                        jl.push_back(q.sat.toString());
                        jlp.push_back(jl.back().c_str());
                    }
                }
                if (const int jc = static_cast<int>(jlp.size()); jc == 0) ImGui::TextDisabled("所有卫星均未检测到周跳");
                else
                    drawPlot("slipratio", "周跳比", kPlotH, [&] {
                        ImPlot::SetupAxis(ImAxis_X1, "");
                        ImPlot::SetupAxis(ImAxis_Y1, "历元/周跳");
                        ImPlot::SetupAxisTicks(ImAxis_X1, jx.data(), jc, jlp.data());
                        ImPlot::PlotBars("周跳比", jy.data(), jc, 0.7);
                    });
            }
            drawPlot("noise_combined", "伪距噪声 / 载波噪声", kPlotH, [&] {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, "伪距噪声 (m)");
                ImPlot::SetupAxis(ImAxis_Y2, "载波噪声 (cyc)", ImPlotAxisFlags_AuxDefault);
                const int M = static_cast<int>(list.size());
                static std::vector<double> xs, xsR, xsP, rho, ph;
                static std::vector<std::string> labs;
                static std::vector<const char *> labp;
                xs.resize(M);
                xsR.resize(M);
                xsP.resize(M);
                rho.resize(M);
                ph.resize(M);
                labs.resize(M);
                labp.resize(M);
                for (int i = 0; i < M; i++) {
                    const auto &q = rep.sats.at(list[i]);
                    xs[i] = i;
                    xsR[i] = i - 0.2;
                    xsP[i] = i + 0.2;
                    rho[i] = 0.5 * (q.sigRho1 + q.sigRho2);
                    ph[i] = 0.5 * (q.sigPh1 + q.sigPh2);
                    labs[i] = q.sat.toString();
                    labp[i] = labs[i].c_str();
                }
                ImPlot::SetupAxisTicks(ImAxis_X1, xs.data(), M, labp.data());
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
                ImPlot::PlotBars("伪距噪声", xsR.data(), rho.data(), M, 0.38);
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
                ImPlot::PlotBars("载波噪声", xsP.data(), ph.data(), M, 0.38);
            });

            // ===== 综合评估 E 柱状图 (6.8, 越小越好) =====
            {
                const auto &ce = rep.comprehensive;
                const int M = static_cast<int>(list.size());
                static std::vector<double> xs, ys;
                static std::vector<std::string> labs;
                static std::vector<const char *> labp;
                xs.resize(M);
                ys.resize(M);
                labs.resize(M);
                labp.resize(M);
                for (int i = 0; i < M; i++) {
                    const auto &q = rep.sats.at(list[i]);
                    xs[i] = i;
                    ys[i] = ce.E.count(q.sat) ? ce.E.at(q.sat) : 0.0;
                    labs[i] = q.sat.toString();
                    labp[i] = labs[i].c_str();
                }
                drawPlot("comprehensiveE", "综合质量评估 E", kPlotH, [&] {
                    ImPlot::SetupAxis(ImAxis_X1, "");
                    ImPlot::SetupAxis(ImAxis_Y1, "E");
                    ImPlot::SetupAxisTicks(ImAxis_X1, xs.data(), M, labp.data());
                    ImPlot::PlotBars("综合 E", ys.data(), M, 0.7);
                });
            }
        } else if (!toPlot.empty()) {
            const SatQC &q = *toPlot[0];
            if (ImGui::BeginTable("##satqc", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("指标");
                ImGui::TableSetupColumn("值");
                ImGui::TableHeadersRow();
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("频点 f1 / f2");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%c / %c", q.band1, q.band2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("完整率");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f %% (%d/%d)", q.completeness, q.validDual, q.totalEpochs);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("频点完整率 DI_f1 / DI_f2");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f %% / %.2f %%", q.di_f1, q.di_f2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("周跳数 / 周跳比");
                ImGui::TableSetColumnIndex(1);
                if (q.slips > 0) ImGui::Text("%d / %.1f", q.slips, q.slipRatio);
                else ImGui::Text("%d / ∞", q.slips);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("粗差数 / 接收机钟跳数");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d / %d", q.outliers, q.clockJumps);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("MP1 / MP2 (RMS)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3f / %.3f m", q.mp1Rms, q.mp2Rms);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("伪距噪声 f1 / f2");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3f / %.3f m", q.sigRho1, q.sigRho2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("载波相位噪声 f1 / f2");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.4f / %.4f cyc", q.sigPh1, q.sigPh2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("电离层残差变化率 IOD f1 / f2");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("std %.4f / %.4f m/s, 跳变 %d / %d 次",
                            q.ionoRateStd1, q.ionoRateStd2, q.ionoJumps1, q.ionoJumps2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("CNR f1 / f2 (mean/min/max)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.1f / %.1f / %.1f | %.1f / %.1f / %.1f dB-Hz",
                            q.cnrMean, q.cnrMin, q.cnrMax, q.cnrMean2, q.cnrMin2, q.cnrMax2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("综合评估 E");
                ImGui::TableSetColumnIndex(1);
                if (rep.comprehensive.E.count(q.sat))
                    ImGui::Text("%.4f", rep.comprehensive.E.at(q.sat));
                else
                    ImGui::Text("-");
                ImGui::EndTable();
            }
            if (!q.t.empty()) {
                const std::vector<double> &mp1v = q.mp1Resid;
                const std::vector<double> &mp2v = q.mp2Resid;
                drawPlot("s_mp", "多路径误差 MP1 / MP2", kPlotH, [&] {
                    ImPlot::SetupAxes("SOW (s)", "m");
                    ImPlot::PlotLine("MP1", q.t.data(), mp1v.data(), static_cast<int>(q.t.size()));
                    ImPlot::PlotLine("MP2", q.t.data(), mp2v.data(), static_cast<int>(q.t.size()));
                });
                std::vector<double> sl_t, sl_v;
                for (int i = 0; i < static_cast<int>(q.t.size()); i++)
                    if (q.slipFlag[i]) {
                        sl_t.push_back(q.t[i]);
                        sl_v.push_back(mp1v[i]);
                    }
                if (!sl_t.empty())
                    drawPlot("s_slip", "周跳位置", kPlotH, [&] {
                        ImPlot::SetupAxes("SOW (s)", "m");
                        ImPlot::PlotScatter("周跳", sl_t.data(), sl_v.data(), static_cast<int>(sl_t.size()));
                    });
                drawPlot("s_iod", "电离层残差变化率 IOD", kPlotH, [&] {
                    ImPlot::SetupAxes("SOW (s)", "m/s");
                    ImPlot::PlotLine("IOD f1", q.t.data(), q.ionoRate.data(), static_cast<int>(q.t.size()));
                    if (!q.ionoRate2.empty())
                        ImPlot::PlotLine("IOD f2", q.t.data(), q.ionoRate2.data(), static_cast<int>(q.t.size()));
                });
                if (!q.cnr.empty() || !q.cnr2.empty())
                    drawPlot("s_cnr", "载噪比 CNR", kPlotH, [&] {
                        ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                        if (!q.cnr.empty())
                            ImPlot::PlotLine("CNR f1", q.t.data(), q.cnr.data(), static_cast<int>(q.t.size()));
                        if (!q.cnr2.empty() && q.cnr2.size() == q.t.size())
                            ImPlot::PlotLine("CNR f2", q.t.data(), q.cnr2.data(), static_cast<int>(q.t.size()));
                    });
            }
        }

        ImGui::EndChild(); // ##qcplots

        // 批量导出：overlay 中重放 plot body
        if (qs.exporting && !qs.exportQueue.empty()) {
            std::string id = qs.exportQueue.front();
            if (auto it = plotDefs.find(id); it != plotDefs.end()) {
                const float pw = std::min(ImGui::GetIO().DisplaySize.x - 40.0f, 1920.0f);
                constexpr float ph = kPlotH + 60.0f;
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
                if (ImGui::Begin("##qc_overlay", nullptr,
                                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings)) {
                    ImGui::SetCursorPos(ImVec2((ImGui::GetIO().DisplaySize.x - pw) * 0.5f, 30.0f));
                    if (ImGui::BeginChild("##eframe", ImVec2(pw, ph), ImGuiChildFlags_Borders)) {
                        if (ImPlot::BeginPlot(it->second.first.c_str(), ImVec2(-1, -1))) {
                            it->second.second();
                            ImPlot::EndPlot();
                        }
                    }
                    ImGui::EndChild();
                    const ImVec2 rmin = ImGui::GetItemRectMin(), rmax = ImGui::GetItemRectMax();
                    RequestCaptureRegionPNG(
                        utf8ToWide(qs.exportDir + "/" + baseName + "_" + sanitizeId(id) + ".png"),
                        static_cast<int>(rmin.x), static_cast<int>(rmin.y), static_cast<int>(rmax.x - rmin.x),
                        static_cast<int>(rmax.y - rmin.y));
                    qs.exportQueue.erase(qs.exportQueue.begin());
                    if (qs.exportQueue.empty()) qs.exporting = false;
                    ImGui::End();
                }
            } else {
                qs.exportQueue.erase(qs.exportQueue.begin());
            }
        }
    } // render
} // namespace QualityControl
