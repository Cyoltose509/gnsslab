#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <filesystem>
#include "QualityControl.h"
#include "ui/Gui.h"
#include "SolverTask.h"
#include "implot.h"
#include "Const.h"
#include "Csv.h"
#include <mutex>
#include <vector>
#include <functional>
#include <fstream>
#include <iomanip>
#include <cstdio>      // snprintf：概览表列取值器
#include <StringUtils.h>
#include "GuiSolverPanel.h"
#include "GuiHelpers.h"   // Helpers::saveCSVDialog / saveFileDialog：统一保存对话框


namespace QualityControl {
    constexpr float kPlotH = 420.0f;



    namespace {
    // 按任务实例隔离 UI 状态：同一类型的两个任务（如 RTK 基准站/流动站）各自持有一份。
    // weak_ptr 键 + 进入时清理过期条目：任务销毁后条目残留虽无害，但地址复用会让新任务
    // 继承旧任务的 sysSel/comboSel/导出状态（RTK 双 tab 曾靠 owner 指针区分，是潜在串状态 bug）。
    static QcUiState &uiState(const std::shared_ptr<void> &owner) {
        static std::map<std::weak_ptr<void>, QcUiState, std::owner_less<>> pool;
        for (auto it = pool.begin(); it != pool.end();) {
            if (it->first.expired()) it = pool.erase(it);
            else ++it;
        }
        return pool[owner];
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


    // 将质量分析报告导出为 CSV（含系统汇总、逐卫星统计两段）。
    // 统一走 Csv::Writer（与 SolverTask 定位导出一致），不再手写 ofstream 拼接。
    static void exportCsv(const QualityReport &rep, const std::string &taskLabel) {
        std::wstring path;
        // 统一走 Helpers 保存对话框（与结果页 CSV 导出、INS 标定导出同一入口）
        if (!Solvers::Helpers::saveCSVDialog(stripExt(taskLabel) + "_qc.csv", path)) return; // 用户取消

        // d3 = 3 位定点小数，逐字段对齐旧全局 fixed<<setprecision(3) 的输出宽度。
        Csv::Writer w;
        auto d3 = [](const double v) { return Csv::fx(v, 3); };
        auto num3 = [](const double v) {
            std::ostringstream os;
            os << std::fixed << std::setprecision(3) << v;
            return os.str();
        };
        // 频点按单字符输出（band1/band2 是 char；直接进 cell 会整型提升成 ASCII 码）。
        auto bandCh = [](const char c) { return std::string(1, c); };

        w.row("GNSS 观测数据质量分析报告", taskLabel);
        w.row("总历元", rep.totalEpochs, "间隔(s)", d3(rep.interval), "可用卫星",
              static_cast<int>(rep.sats.size()), "总体完整率%", d3(rep.overallCompleteness),
              "平均CNR(dB-Hz)", d3(rep.overallCnr));
        w.endRow();

        w.cell("[系统汇总]").endRow();
        w.row("系统", "完整率%", "DI_f1%", "DI_f2%", "周跳数", "周跳比", "粗差数", "钟跳数",
              "MP1(m)", "MP2(m)", "伪距噪声(m)", "载波噪声(cyc)",
              "CNR_all(dB-Hz)", "CNR_f1(dB-Hz)", "CNR_f2(dB-Hz)",
              "IOD_std_all(m/s)", "IOD_std_f1(m/s)", "IOD_std_f2(m/s)",
              "IOD_跳变", "IOD_跳变_f1", "IOD_跳变_f2", "综合E");
        for (auto &[s, comp]: rep.sysCompleteness) {
            w.cell(sysName(s))
                    .cell(d3(comp))
                    .cell(d3(rep.sysDI_f1.count(s) ? rep.sysDI_f1.at(s) : 0.0))
                    .cell(d3(rep.sysDI_f2.count(s) ? rep.sysDI_f2.at(s) : 0.0))
                    .cell(rep.sysSlips.at(s)).cell(d3(rep.sysSlipRatio.at(s)))
                    .cell(rep.sysOutliers.count(s) ? rep.sysOutliers.at(s) : 0)
                    .cell(rep.sysClockJumps.count(s) ? rep.sysClockJumps.at(s) : 0)
                    .cell(d3(rep.sysMp1.at(s))).cell(d3(rep.sysMp2.at(s)))
                    .cell(d3(rep.sysSigRho.at(s))).cell(d3(rep.sysSigPhase.at(s)))
                    .cell(d3(rep.sysCnr.count(s) ? rep.sysCnr.at(s) : 0.0))
                    .cell(d3(rep.sysCnr1.count(s) ? rep.sysCnr1.at(s) : 0.0))
                    .cell(d3(rep.sysCnr2.count(s) ? rep.sysCnr2.at(s) : 0.0))
                    .cell(d3(rep.sysIonoRate.count(s) ? rep.sysIonoRate.at(s) : 0.0))
                    .cell(d3(rep.sysIonoRate1.count(s) ? rep.sysIonoRate1.at(s) : 0.0))
                    .cell(d3(rep.sysIonoRate2.count(s) ? rep.sysIonoRate2.at(s) : 0.0))
                    .cell(rep.sysIonoJumps.count(s) ? rep.sysIonoJumps.at(s) : 0)
                    .cell(rep.sysIonoJumps1.count(s) ? rep.sysIonoJumps1.at(s) : 0)
                    .cell(rep.sysIonoJumps2.count(s) ? rep.sysIonoJumps2.at(s) : 0)
                    .cell(d3(rep.comprehensive.sysE.count(s) ? rep.comprehensive.sysE.at(s) : 0.0))
                    .endRow();
        }
        w.endRow();

        w.cell("[逐卫星统计]").endRow();
        w.row("卫星", "系统", "频点f1", "频点f2", "完整率%", "DI_f1%", "DI_f2%",
              "周跳数", "周跳比", "粗差数", "钟跳数", "MP1(m)", "MP2(m)",
              "伪距噪声_f1(m)", "伪距噪声_f2(m)", "载波噪声_f1(cyc)", "载波噪声_f2(cyc)",
              "CNR_f1(dB-Hz)", "CNR_f2(dB-Hz)", "IOD_std_f1(m/s)", "IOD_std_f2(m/s)",
              "IOD_跳变_f1", "IOD_跳变_f2");
        for (auto &sat: rep.satOrder) {
            const auto &q = rep.sats.at(sat);
            w.cell(q.sat.toString()).cell(sysName(q.sat.system))
                    .cell(bandCh(q.band1)).cell(bandCh(q.band2))
                    .cell(d3(q.completeness)).cell(d3(q.di_f1)).cell(d3(q.di_f2))
                    .cell(q.slips).cell(d3(q.slipRatio)).cell(q.outliers).cell(q.clockJumps)
                    .cell(d3(q.mp1Rms)).cell(d3(q.mp2Rms))
                    .cell(d3(q.sigRho1)).cell(d3(q.sigRho2))
                    .cell(d3(q.sigPh1)).cell(d3(q.sigPh2))
                    .cell(d3(q.cnrMean)).cell(d3(q.cnrMean2))
                    .cell(d3(q.ionoRateStd1)).cell(d3(q.ionoRateStd2))
                    .cell(q.ionoJumps1).cell(q.ionoJumps2)
                    .endRow();
        }
        w.endRow();

        w.cell("[综合评估]").endRow();
        w.row("整体综合E", d3(rep.comprehensive.overallE));
        w.cell("系统综合E");
        for (auto &[s, e]: rep.comprehensive.sysE) w.cell(std::string(sysName(s)) + "=" + num3(e));
        w.endRow();
        w.cell("指标");
        for (auto &ind: rep.comprehensive.indicators) w.cell(ind);
        w.endRow();
        w.cell("权重");
        for (const double wt: rep.comprehensive.weights) w.cell(d3(wt));
        w.endRow();
        w.endRow();

        w.cell("[逐卫星综合评估]").endRow();
        w.cell("卫星").cell("系统");
        for (auto &ind: rep.comprehensive.indicators) w.cell(ind);
        w.cell("E").endRow();
        for (auto &sat: rep.satOrder) {
            w.cell(sat.toString()).cell(sysName(sat.system));
            if (rep.comprehensive.z.count(sat))
                for (const double zv: rep.comprehensive.z.at(sat)) w.cell(d3(zv));
            else
                for (size_t l = 0; l < rep.comprehensive.indicators.size(); l++) w.cell("0");
            w.cell(d3(rep.comprehensive.E.count(sat) ? rep.comprehensive.E.at(sat) : 0.0)).endRow();
        }
        w.endRow();

        // withBom=true：中文表头无 BOM 会被 Excel 按 ANSI 解成乱码。
        (void) w.save(path, true);
    }

    // 「导出此图 PNG」小按钮
    static void plotExportBtn(const char *uid, const ImVec2 p0, const float w, const float h, const std::string &base) {
        const std::string lbl = std::string("导出此图 PNG##") + uid;
        if (ImGui::SmallButton(lbl.c_str())) {
            if (std::wstring path; Solvers::Helpers::saveFileDialog(base + "_" + uid + ".png", path,
                                                                   Solvers::Helpers::fPng, 2, L"png"))
                RequestCaptureRegionPNG(path, static_cast<int>(p0.x), static_cast<int>(p0.y), static_cast<int>(w), static_cast<int>(h));
        }
    }

        // 统一的“图框 + 导出此图 PNG 按钮”容器：登记导出回调并记录布局矩形
        static void drawPlot(QcUiState &qs, const std::string &baseName,
                             const char *id, const char *title, const float defH,
                             const std::function<void()> &body) {
            auto h = defH;
            if (h < 80.0f) h = 80.0f;

            const float w = ImGui::GetContentRegionAvail().x; // 宽度始终占满

            qs.contentY[id] = ImGui::GetCursorPosY();
            qs.lastIds.emplace_back(id);
            qs.plotDefs[id] = {title, body};

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
        }

        // 系统汇总表列定义（13 列，与 CSV 导出同序）
        static const char *kSysColHdrs[] = {
            "系统", "完整率%", "周跳数", "周跳比", "粗差数", "钟跳数", "MP1(m)", "MP2(m)",
            "伪距噪声(m)", "载波噪声(cyc)", "CNR_f1(dB-Hz)", "CNR_f2(dB-Hz)", "综合评估 E"
        };

        // 概览：顶部统计行 + 各系统完整率表（含“总体”汇总行）。
        // 原逐格 4 行手写 TableSetColumnIndex/Text（13 列 × 2 行 ≈ 120 行），收敛为
        // “列定义 + 行循环 + 列取值器”；显示格式逐列与原实现一致。
        void renderOverviewStats(const QualityReport &rep) {
            ImGui::SeparatorText("总体概览");

            ImGui::Text("总历元 %d | 间隔 %.2f s | 可用卫星 %d 接收机钟跳 %d 历元",
                        rep.totalEpochs, rep.interval, static_cast<int>(rep.sats.size()), rep.clockJumpEpochs);
            // 总体行派生值：计数按系统累加，连续指标按所有卫星直接平均（与顶部 CNR 一致）
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
            // 列取值器：col 同 kSysColHdrs 下标；s 为 G/R/E/C 或 'T'（总体行）
            auto cellText = [&](const int col, const char s) -> std::string {
                const bool tot = s == 'T';
                char b[64];
                switch (col) {
                    case 0: return tot ? "总体" : sysName(s);
                    case 1: snprintf(b, sizeof(b), "%.2f", tot ? rep.overallCompleteness : rep.sysCompleteness.at(s)); return b;
                    case 2: return std::to_string(tot ? totalSlips : rep.sysSlips.at(s));
                    case 3: {
                        const int sl = tot ? totalSlips : rep.sysSlips.at(s);
                        if (sl <= 0) return "∞";
                        snprintf(b, sizeof(b), "%.1f", tot ? static_cast<double>(rep.totalEpochs) / sl : rep.sysSlipRatio.at(s));
                        return b;
                    }
                    case 4: return std::to_string(tot ? totalOutliers : (rep.sysOutliers.count(s) ? rep.sysOutliers.at(s) : 0));
                    case 5: return std::to_string(tot ? totalClockJumps : (rep.sysClockJumps.count(s) ? rep.sysClockJumps.at(s) : 0));
                    case 6: snprintf(b, sizeof(b), "%.3f", tot ? (cnt ? sumMp1 / cnt : 0.0) : rep.sysMp1.at(s)); return b;
                    case 7: snprintf(b, sizeof(b), "%.3f", tot ? (cnt ? sumMp2 / cnt : 0.0) : rep.sysMp2.at(s)); return b;
                    case 8: snprintf(b, sizeof(b), "%.3f", tot ? (cnt ? sumSigRho / cnt : 0.0) : rep.sysSigRho.at(s)); return b;
                    case 9: snprintf(b, sizeof(b), "%.4f", tot ? (cnt ? sumSigPh / cnt : 0.0) : rep.sysSigPhase.at(s)); return b;
                    case 10: snprintf(b, sizeof(b), "%.1f", tot ? rep.overallCnr1 : (rep.sysCnr1.count(s) ? rep.sysCnr1.at(s) : 0.0)); return b;
                    case 11: snprintf(b, sizeof(b), "%.1f", tot ? rep.overallCnr2 : (rep.sysCnr2.count(s) ? rep.sysCnr2.at(s) : 0.0)); return b;
                    default:
                        if (tot) { snprintf(b, sizeof(b), "%.4f", rep.comprehensive.overallE); return b; }
                        if (!rep.comprehensive.sysE.count(s)) return "-";
                        snprintf(b, sizeof(b), "%.4f", rep.comprehensive.sysE.at(s));
                        return b;
                }
            };
            auto writeRow = [&](const char s) {
                ImGui::TableNextRow();
                for (int c = 0; c < 13; c++) {
                    ImGui::TableSetColumnIndex(c);
                    ImGui::TextUnformatted(cellText(c, s).c_str());
                }
            };
            // ScrollX 表必须给定 outer_size.y，否则会撑满父窗口全部高度（表现为超大空表）
            const float rowH = ImGui::GetTextLineHeightWithSpacing();
            const int sysRows = static_cast<int>(rep.sysCompleteness.size()) + 1; // +1 总体行
            if (const ImVec2 sysTblSize(0.0f, rowH * static_cast<float>(sysRows + 1) + 8.0f); ImGui::BeginTable(
                "##sys", 13, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX |
                             ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit, sysTblSize)) {
                for (int c = 0; c < 13; c++) ImGui::TableSetupColumn(kSysColHdrs[c]);
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
                // 逐系统行：固定顺序 GPS/GLO/GAL/BDS；仅统计实际出现的系统
                for (const char s: {'G', 'R', 'E', 'C'})
                    if (rep.sysCompleteness.count(s)) writeRow(s);
                if (!rep.sysCompleteness.empty()) writeRow('T'); // 总体行
                ImGui::EndTable();
            }
        }

        // 导出条：CSV 报告 / 一键导出全部 PNG / 导出状态提示
        void renderExportBar(QcUiState &qs, const QualityReport &rep,
                             const std::string &taskFileName) {
            if (ImGui::Button("导出 CSV (系统汇总/逐卫星)")) {
                exportCsv(rep, taskFileName);
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
        }

        // 逐卫星统计：按系统筛选后的卫星列表 + 下拉选择；无可用卫星返回 false
        bool renderSatelliteSelector(const QualityReport &rep, QcUiState &qs, const int sysSel,
                                     std::vector<SatID> &list,
                                     std::vector<const SatQC *> &toPlot, bool &allView) {
            ImGui::SeparatorText("逐卫星统计");
            for (auto &sat: rep.satOrder) {
                if (sysSel == 0 || (sysSel == 1 && sat.system == 'G') ||
                    (sysSel == 2 && sat.system == 'R') || (sysSel == 3 && sat.system == 'E') ||
                    (sysSel == 4 && sat.system == 'C')) list.push_back(sat);
            }
            if (list.empty()) {
                ImGui::Text("无符合条件的卫星。");
                return false;
            }
            int &comboSel = qs.comboSel;
            if (comboSel < 0 || comboSel > static_cast<int>(list.size())) comboSel = 0;
            std::string items = "全部";
            items += '\0';
            for (auto &s: list) {
                items += s.toString();
                items += '\0';
            }
            items += '\0';
            ImGui::Combo("选择卫星", &comboSel, items.c_str());
            allView = comboSel == 0;

            if (allView) for (auto &sat: list) toPlot.push_back(&rep.sats.at(sat));
            else if (comboSel >= 1 && comboSel <= static_cast<int>(list.size())) toPlot.push_back(&rep.sats.at(list[comboSel - 1]));

            return true;
        }

        // 全部卫星视图：MP1 / MP2 / IOD / CNR 四条时间序列
        void renderAllSatTimeSeries(QcUiState &qs, const std::string &baseName,
                                    const std::vector<const SatQC *> &toPlot) {
            drawPlot(qs, baseName, "mp1_all", "多路径误差 MP1", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->mp1Resid.data(), static_cast<int>(q->t.size()));
            });
            drawPlot(qs, baseName, "mp2_all", "多路径误差 MP2", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->mp2Resid.data(), static_cast<int>(q->t.size()));
            });
            drawPlot(qs, baseName, "iod_all", "电离层残差变化率 IOD", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m/s");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->ionoRate.data(), static_cast<int>(q->t.size()));
            });
            drawPlot(qs, baseName, "cnr_all", "载噪比 CNR", kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                for (const auto q: toPlot)
                    if (!q->t.empty())
                        ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->cnr.data(), static_cast<int>(q->t.size()));
            });
        }

        // 柱状图标签缓冲：按卫星列表填 xs=i / ys=getter(q) / labs / labp。
        // completeness 与 comprehensiveE 两款逐字同形，抽此共用（slipratio 有过滤、
        // noise_combined 是双轴 5 列，结构不同，保留各自写法）。
        static void fillBarBuf(QcUiState::PlotBuf &buf, const std::vector<SatID> &list,
                               const QualityReport &rep, const std::function<double(const SatQC &)> &get) {
            const auto M = static_cast<int>(list.size());
            auto &[col, labs, labp] = buf;
            auto &xs = col[0], &ys = col[1];
            xs.resize(M);
            ys.resize(M);
            labs.resize(M);
            labp.resize(M);
            for (int i = 0; i < M; i++) {
                const auto &q = rep.sats.at(list[i]);
                xs[i] = i;
                ys[i] = get(q);
                labs[i] = q.sat.toString();
                labp[i] = labs[i].c_str();
            }
        }

        // 逐卫星指标柱状图：观测完整率 / 周跳比 / 伪距-载波噪声
        void renderPerSatMetricBars(const QualityReport &rep, QcUiState &qs,
                                    const std::string &baseName,
                                    const std::vector<SatID> &list) {
            // ===== 逐卫星指标柱状图 =====
            ImGui::SeparatorText("逐卫星指标");
            {
                auto &buf = qs.buf("completeness");
                fillBarBuf(buf, list, rep, [](const SatQC &q) { return q.completeness; });
                const auto M = static_cast<int>(list.size());
                drawPlot(qs, baseName, "completeness", "观测完整率 (%)", kPlotH, [&] {
                    ImPlot::SetupAxis(ImAxis_X1, "");
                    ImPlot::SetupAxis(ImAxis_Y1, "%");
                    auto &[col, labs, labp] = buf;
                    ImPlot::SetupAxisTicks(ImAxis_X1, col[0].data(), M, labp.data());
                    ImPlot::PlotBars("观测完整率", col[1].data(), M, 0.7);
                });
            }
            // 周跳比
            {
                const int M = static_cast<int>(list.size());
                auto &[col, labs, labp] = qs.buf("slipratio");
                auto &jx = col[0], &jy = col[1];
                jx.clear();
                jy.clear();
                labs.clear();
                labp.clear();
                for (int i = 0; i < M; i++) {
                    if (const auto &q = rep.sats.at(list[i]); q.slips > 0) {
                        jx.push_back(static_cast<double>(labp.size()));
                        jy.push_back(q.slipRatio);
                        labs.push_back(q.sat.toString());
                        labp.push_back(labs.back().c_str());
                    }
                }
                if (const int jc = static_cast<int>(labp.size()); jc == 0) ImGui::TextDisabled("所有卫星均未检测到周跳");
                else
                    drawPlot(qs, baseName, "slipratio", "周跳比", kPlotH, [&] {
                        ImPlot::SetupAxis(ImAxis_X1, "");
                        ImPlot::SetupAxis(ImAxis_Y1, "历元/周跳");
                        ImPlot::SetupAxisTicks(ImAxis_X1, jx.data(), jc, labp.data());
                        ImPlot::PlotBars("周跳比", jy.data(), jc, 0.7);
                    });
            }
            drawPlot(qs, baseName, "noise_combined", "伪距噪声 / 载波噪声", kPlotH, [&] {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, "伪距噪声 (m)");
                ImPlot::SetupAxis(ImAxis_Y2, "载波噪声 (cyc)", ImPlotAxisFlags_AuxDefault);
                const int M = static_cast<int>(list.size());
                auto &[col, labs, labp] = qs.buf("noise_combined");
                auto &xs = col[0], &xsR = col[1], &xsP = col[2], &rho = col[3], &ph = col[4];
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
        }

        // 综合评估 E 柱状图（熵权法，越小越好）
        void renderComprehensiveBars(const QualityReport &rep, QcUiState &qs,
                                     const std::string &baseName,
                                     const std::vector<SatID> &list) {
            // ===== 综合评估 E 柱状图 (6.8, 越小越好) =====
            const auto &ce = rep.comprehensive;
            auto &buf = qs.buf("comprehensive");
            fillBarBuf(buf, list, rep, [&ce](const SatQC &q) { return ce.E.count(q.sat) ? ce.E.at(q.sat) : 0.0; });
            const auto M = static_cast<int>(list.size());
            drawPlot(qs, baseName, "comprehensiveE", "综合质量评估 E", kPlotH, [&] {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, "E");
                auto &[col, labs, labp] = buf;
                ImPlot::SetupAxisTicks(ImAxis_X1, col[0].data(), M, labp.data());
                ImPlot::PlotBars("综合 E", col[1].data(), M, 0.7);
            });
        }

        // 单颗卫星：指标明细表（频点 / 完整率 / 周跳 / 噪声 / IOD / CNR / 综合 E）
        void renderSingleSatMetricTable(const QualityReport &rep, const SatQC &q) {
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
        }

        // 单颗卫星视图：指标明细表 + MP / 周跳位置 / IOD / CNR 时间序列
        void renderSingleSatView(const QualityReport &rep, QcUiState &qs,
                                 const std::string &baseName,
                                 const std::vector<const SatQC *> &toPlot) {
            const SatQC &q = *toPlot[0];
            renderSingleSatMetricTable(rep, q);
            if (!q.t.empty()) {
                const std::vector<double> &mp1v = q.mp1Resid;
                const std::vector<double> &mp2v = q.mp2Resid;
                drawPlot(qs, baseName, "s_mp", "多路径误差 MP1 / MP2", kPlotH, [&] {
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
                    drawPlot(qs, baseName, "s_slip", "周跳位置", kPlotH, [&] {
                        ImPlot::SetupAxes("SOW (s)", "m");
                        ImPlot::PlotScatter("周跳", sl_t.data(), sl_v.data(), static_cast<int>(sl_t.size()));
                    });
                drawPlot(qs, baseName, "s_iod", "电离层残差变化率 IOD", kPlotH, [&] {
                    ImPlot::SetupAxes("SOW (s)", "m/s");
                    ImPlot::PlotLine("IOD f1", q.t.data(), q.ionoRate.data(), static_cast<int>(q.t.size()));
                    if (!q.ionoRate2.empty())
                        ImPlot::PlotLine("IOD f2", q.t.data(), q.ionoRate2.data(), static_cast<int>(q.t.size()));
                });
                if (!q.cnr.empty() || !q.cnr2.empty())
                    drawPlot(qs, baseName, "s_cnr", "载噪比 CNR", kPlotH, [&] {
                        ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                        if (!q.cnr.empty())
                            ImPlot::PlotLine("CNR f1", q.t.data(), q.cnr.data(), static_cast<int>(q.t.size()));
                        if (!q.cnr2.empty() && q.cnr2.size() == q.t.size())
                            ImPlot::PlotLine("CNR f2", q.t.data(), q.cnr2.data(), static_cast<int>(q.t.size()));
                    });
            }
        }

        // 批量导出：每帧在 overlay 中重放一张已登记的 plot body 并截图
        void renderExportOverlay(QcUiState &qs, const std::string &baseName) {
            auto &plotDefs = qs.plotDefs;
            if (qs.exporting && !qs.exportQueue.empty()) {
                const std::string id = qs.exportQueue.front();
                if (const auto it = plotDefs.find(id); it != plotDefs.end()) {
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
        }
    } // namespace (anonymous)

    template<typename TaskT>
    void render(const std::shared_ptr<TaskT> &task) {
        // 文件模式：必须等所有历元读取完毕，质量分析才算（QC 不依赖定位解算，读取完即可）
        if (!task->isRealtime && !task->readDone) {
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "正在等待历元读取完全…");
            return;
        }

        // 懒加载：仅当任务标记 qcLazy 且尚未计算时，在 tab 首次可见时触发一次 LaunchQC。
        // RTK 双标签置 qcLazy=true（加快首屏）；SPP/PPP/LEO 保持 eager 触发（qcLazy=false），
        // 此处不会抢先触发，避免双算 QC。
        if (task->qcLazy && !task->qcReady && !task->qcComputing) Solvers::SolverPanel::LaunchQC(task);

        std::shared_ptr<QualityReport> repPtr;
        {
            std::lock_guard lk(task->qcMutex);
            repPtr = task->qcReport;
        }
        if (!repPtr || !task->qcReady) {
            if (task->qcComputing) {
                ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "正在解算…");
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
        QcUiState &qs = uiState(task);

        ImGui::BeginChild("##qcplots", ImVec2(0, 0));

        // 批量导出时把队列首图滚动到视口顶部
        if (qs.exporting && !qs.exportQueue.empty()) {
            if (const auto it = qs.contentY.find(qs.exportQueue.front()); it != qs.contentY.end())
                ImGui::SetScrollY(it->second - 6.0f);
        }

        int &sysSel = qs.sysSel;
        ImGui::RadioButton("全部", &sysSel, 0);
        ImGui::SameLine();
        ImGui::RadioButton("GPS", &sysSel, 1);
        ImGui::SameLine();
        ImGui::RadioButton("GLO", &sysSel, 2);
        ImGui::SameLine();
        ImGui::RadioButton("GAL", &sysSel, 3);
        ImGui::SameLine();
        ImGui::RadioButton("BDS", &sysSel, 4);

        renderOverviewStats(rep);

        const std::string baseName = stripExt(task->fileName);
        renderExportBar(qs, rep, task->fileName);

        std::vector<SatID> list;
        std::vector<const SatQC *> toPlot;
        bool allView = false;
        if (!renderSatelliteSelector(rep, qs, sysSel, list, toPlot, allView)) {
            ImGui::EndChild(); // ##qcplots
            return;
        }

        qs.curRect.clear();
        qs.lastIds.clear();

        if (allView) {
            renderAllSatTimeSeries(qs, baseName, toPlot);
            renderPerSatMetricBars(rep, qs, baseName, list);
            renderComprehensiveBars(rep, qs, baseName, list);
        } else if (!toPlot.empty()) {
            renderSingleSatView(rep, qs, baseName, toPlot);
        }

        ImGui::EndChild(); // ##qcplots

        renderExportOverlay(qs, baseName);
    } // render

    // 显式实例化：SPP 与 PPP 共用同一套质量分析渲染
    template void render<GnssTask::SolveTask>(const std::shared_ptr<GnssTask::SolveTask> &);

}
