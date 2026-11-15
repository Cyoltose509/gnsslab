#include "ui/GuiColors.h"
#include <windows.h>
#include "QualityControl.h"
#include "ui/Gui.h"
#include "ui/FileDialog.h"
#include "solvers/common/Core.h"
#include "implot.h"
#include "CSV.h"
#include "Log.h"
#include "FileUtils.h"
#include <mutex>
#include <vector>
#include <functional>
#include <iomanip>
#include <cstdio>
#include "StringUtils.h"
#include "GuiSolverPanel.h"
#include "GuiHelpers.h"
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"


namespace QualityControl {
    constexpr float kPlotH = 420.0f;



    namespace {
        // map 兜底取值：键不存在时返回默认值（报告字段按系统/卫星稀疏存放）
        template<class M>
        typename M::mapped_type atOr(const M &m, const typename M::key_type &k,
                                     const typename M::mapped_type &dft = {}) {
            const auto it = m.find(k);
            return it == m.end() ? dft : it->second;
        }

        QcUiState &uiState(const std::shared_ptr<void> &owner) {
            static std::map<std::weak_ptr<void>, QcUiState, std::owner_less<>> pool;
            for (auto it = pool.begin(); it != pool.end();) {
                if (it->first.expired()) it = pool.erase(it);
                else ++it;
            }
            return pool[owner];
        }

        /// 断点轴：QC 序列只在双频齐全的历元上有点，卫星不可见或丢频的整段是真空档，
        /// 直连会画出一条跨越数小时的假斜线。首帧按需建，报告换掉时整批作废。
        const Solvers::BrokenAxis &axisOf(QcUiState &qs, const SatQC &q) {
            auto &m = qs.axes.bySat;
            if (const auto it = m.find(q.sat); it != m.end()) return it->second;
            Solvers::BrokenAxis b;
            b.build(q.t, qs.axes.interval > 0.0 ? 2.5 * qs.axes.interval : 0.0);
            return m.emplace(q.sat, std::move(b)).first->second;
        }


    void exportCSV(const QualityReport &rep, const std::string &taskLabel) {
        std::wstring path;

        if (!FileDialog::saveCSV(File::stripExt(taskLabel) + "_qc.csv", path)) return;


        CSV::Writer w;
        CSV::Meta m{w};
        auto d3 = [](const double v) { return CSV::fx(v, 3); };
        auto num3 = [](const double v) {
            std::ostringstream os;
            os << std::fixed << std::setprecision(3) << v;
            return os.str();
        };

        auto bandCh = [](const char c) { return std::string(1, c); };

        w.row(I18n::trc("qc.report_title"), taskLabel);
        w.row(I18n::trc("qc.total_epochs"), rep.totalEpochs, I18n::trc("qc.interval_s"), d3(rep.interval), I18n::trc("qc.usable_sats"),
              static_cast<int>(rep.sats.size()), I18n::trc("qc.overall_completeness"), d3(rep.overallCompleteness),
              I18n::trc("qc.mean_cnr"), d3(rep.overallCnr));
        w.blank();

        m.section(I18n::trc("qc.section.by_system"));
        w.row(I18n::trc("common.system"), I18n::trc("qc.col.completeness_pct"), "DI_f1%", "DI_f2%", I18n::trc("qc.col.slips"), I18n::trc("CycleSlipRatio"), I18n::trc("qc.col.outliers"), I18n::trc("qc.col.clock_jumps"),
              "MP1(m)", "MP2(m)", I18n::trc("qc.col.code_noise"), I18n::trc("qc.col.phase_noise"),
              "CNR_all(dB-Hz)", "CNR_f1(dB-Hz)", "CNR_f2(dB-Hz)",
              "IOD_std_all(m/s)", "IOD_std_f1(m/s)", "IOD_std_f2(m/s)",
              I18n::trc("qc.col.iod_jump"), I18n::trc("qc.col.iod_jump_f1"), I18n::trc("qc.col.iod_jump_f2"), I18n::trc("qc.col.composite_e"));
        for (auto &[s, comp]: rep.sysCompleteness) {
            w.cell(Solvers::Helpers::systemName(s))
                    .cell(d3(comp))
                    .cell(d3(atOr(rep.sysDI_f1, s)))
                    .cell(d3(atOr(rep.sysDI_f2, s)))
                    .cell(rep.sysSlips.at(s)).cell(d3(rep.sysSlipRatio.at(s)))
                    .cell(atOr(rep.sysOutliers, s))
                    .cell(atOr(rep.sysClockJumps, s))
                    .cell(d3(rep.sysMp1.at(s))).cell(d3(rep.sysMp2.at(s)))
                    .cell(d3(rep.sysSigRho.at(s))).cell(d3(rep.sysSigPhase.at(s)))
                    .cell(d3(atOr(rep.sysCnr, s)))
                    .cell(d3(atOr(rep.sysCnr1, s)))
                    .cell(d3(atOr(rep.sysCnr2, s)))
                    .cell(d3(atOr(rep.sysIonoRate, s)))
                    .cell(d3(atOr(rep.sysIonoRate1, s)))
                    .cell(d3(atOr(rep.sysIonoRate2, s)))
                    .cell(atOr(rep.sysIonoJumps, s))
                    .cell(atOr(rep.sysIonoJumps1, s))
                    .cell(atOr(rep.sysIonoJumps2, s))
                    .cell(d3(atOr(rep.comprehensive.sysE, s)))
                    .endRow();
        }
        w.blank();

        m.section(I18n::trc("qc.section.by_sat"));
        w.row(I18n::trc("Satellite"), I18n::trc("common.system"), I18n::trc("qc.col.freq1"), I18n::trc("qc.col.freq2"), I18n::trc("qc.col.completeness_pct"), "DI_f1%", "DI_f2%",
              I18n::trc("qc.col.slips"), I18n::trc("CycleSlipRatio"), I18n::trc("qc.col.outliers"), I18n::trc("qc.col.clock_jumps"), "MP1(m)", "MP2(m)",
              I18n::trc("qc.col.code_noise_f1"), I18n::trc("qc.col.code_noise_f2"), I18n::trc("qc.col.phase_noise_f1"), I18n::trc("qc.col.phase_noise_f2"),
              "CNR_f1(dB-Hz)", "CNR_f2(dB-Hz)", "IOD_std_f1(m/s)", "IOD_std_f2(m/s)",
              I18n::trc("qc.col.iod_jump_f1"), I18n::trc("qc.col.iod_jump_f2"));
        for (auto &sat: rep.satOrder) {
            const auto &q = rep.sats.at(sat);
            w.cell(q.sat.toString()).cell(Solvers::Helpers::systemName(q.sat.system))
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
        w.blank();

        m.section(I18n::trc("qc.section.assessment"));
        w.row(I18n::trc("qc.overall_e"), d3(rep.comprehensive.overallE));
        w.cell(I18n::trc("qc.system_e"));
        for (auto &[s, e]: rep.comprehensive.sysE) w.cell(std::string(Solvers::Helpers::systemName(s)) + "=" + num3(e));
        w.endRow();
        w.cell(I18n::trc("qc.col.indicator"));
        for (auto &ind: rep.comprehensive.indicators) w.cell(I18n::tr(ind));
        w.endRow();
        w.cell(I18n::trc("Weight"));
        for (const double wt: rep.comprehensive.weights) w.cell(d3(wt));
        w.endRow();
        w.blank();

        m.section(I18n::trc("qc.section.sat_assessment"));
        w.cell(I18n::trc("Satellite")).cell(I18n::trc("common.system"));
        for (auto &ind: rep.comprehensive.indicators) w.cell(I18n::tr(ind));
        w.cell("E").endRow();
        for (auto &sat: rep.satOrder) {
            w.cell(sat.toString()).cell(Solvers::Helpers::systemName(sat.system));
            if (const auto it = rep.comprehensive.z.find(sat); it != rep.comprehensive.z.end())
                for (const double zv: it->second) w.cell(d3(zv));
            else
                for (std::size_t l = 0; l < rep.comprehensive.indicators.size(); l++) w.cell("0");
            w.cell(d3(atOr(rep.comprehensive.E, sat))).endRow();
        }
        w.blank();


        std::string error;
        if (!File::safeWrite(std::filesystem::path(path), [&](std::ostream &f) {
                CSV::Writer::writeBom(f);
                f << w.str();
            }, error))
            LOG_ERROR << "[QC] " << error;
    }


    void plotExportBtn(const char *uid, const ImVec2 p0, const float w, const float h, const std::string &base) {
        const char *lbl = UI::idLabel("qc.export_plot_png", "##", uid);
        if (ImGui::SmallButton(lbl)) {
            if (std::wstring path; FileDialog::save(base + "_" + uid + ".png", path,
                                                    FileDialog::fPng(), L"png"))
                RequestCaptureRegionPNG(path, static_cast<int>(p0.x), static_cast<int>(p0.y), static_cast<int>(w), static_cast<int>(h));
        }
    }


    void drawPlot(QcUiState &qs, const std::string &baseName,
                  const char *id, const char *title, const float defH,
                  const std::function<void()> &body) {
            auto h = defH;
            if (h < 80.0f) h = 80.0f;

            const float w = ImGui::GetContentRegionAvail().x;

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


            const ImVec2 rmin = ImGui::GetItemRectMin(), rmax = ImGui::GetItemRectMax();
            qs.curRect[id] = ImVec4(rmin.x, rmin.y, rmax.x - rmin.x, rmax.y - rmin.y);

            plotExportBtn(id, rmin, rmax.x - rmin.x, rmax.y - rmin.y, baseName);
        }


        const char *kSysColHdrs[] = {
            I18n::trc("common.system"), I18n::trc("qc.col.completeness_pct"), I18n::trc("qc.col.slips"), I18n::trc("CycleSlipRatio"), I18n::trc("qc.col.outliers"), I18n::trc("qc.col.clock_jumps"), "MP1(m)", "MP2(m)",
            I18n::trc("qc.col.code_noise"), I18n::trc("qc.col.phase_noise"), "CNR_f1(dB-Hz)", "CNR_f2(dB-Hz)", I18n::trc("qc.col.assessment_e")
        };




        void renderOverviewStats(const QualityReport &rep) {
            ImGui::SeparatorText(I18n::trc("qc.section.overview"));

            ImGui::Text(I18n::trc("qc.overview_line"),
                        rep.totalEpochs, rep.interval, static_cast<int>(rep.sats.size()), rep.clockJumpEpochs);

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

            auto cellText = [&](const int col, const char s) -> std::string {
                const bool tot = s == 'T';
                char b[64];
                switch (col) {
                    case 0: return tot ? I18n::trc("qc.system_total") : Solvers::Helpers::systemName(s);
                    case 1: std::snprintf(b, sizeof(b), "%.2f", tot ? rep.overallCompleteness : rep.sysCompleteness.at(s)); return b;
                    case 2: return std::to_string(tot ? totalSlips : rep.sysSlips.at(s));
                    case 3: {
                        const int sl = tot ? totalSlips : rep.sysSlips.at(s);
                        if (sl <= 0) return "∞";
                        std::snprintf(b, sizeof(b), "%.1f", tot ? static_cast<double>(rep.totalEpochs) / sl : rep.sysSlipRatio.at(s));
                        return b;
                    }
                    case 4: return std::to_string(tot ? totalOutliers : atOr(rep.sysOutliers, s));
                    case 5: return std::to_string(tot ? totalClockJumps : atOr(rep.sysClockJumps, s));
                    case 6: std::snprintf(b, sizeof(b), "%.3f", tot ? (cnt ? sumMp1 / cnt : 0.0) : rep.sysMp1.at(s)); return b;
                    case 7: std::snprintf(b, sizeof(b), "%.3f", tot ? (cnt ? sumMp2 / cnt : 0.0) : rep.sysMp2.at(s)); return b;
                    case 8: std::snprintf(b, sizeof(b), "%.3f", tot ? (cnt ? sumSigRho / cnt : 0.0) : rep.sysSigRho.at(s)); return b;
                    case 9: std::snprintf(b, sizeof(b), "%.4f", tot ? (cnt ? sumSigPh / cnt : 0.0) : rep.sysSigPhase.at(s)); return b;
                    case 10: std::snprintf(b, sizeof(b), "%.1f", tot ? rep.overallCnr1 : atOr(rep.sysCnr1, s)); return b;
                    case 11: std::snprintf(b, sizeof(b), "%.1f", tot ? rep.overallCnr2 : atOr(rep.sysCnr2, s)); return b;
                    default:
                        if (tot) { std::snprintf(b, sizeof(b), "%.4f", rep.comprehensive.overallE); return b; }
                        const auto itE = rep.comprehensive.sysE.find(s);
                        if (itE == rep.comprehensive.sysE.end()) return "-";
                        std::snprintf(b, sizeof(b), "%.4f", itE->second);
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

            const float rowH = ImGui::GetTextLineHeightWithSpacing();
            const int sysRows = static_cast<int>(rep.sysCompleteness.size()) + 1;
            // 要横向滚动就得先把滚动条那一行留出来，否则最后一行数据会被裁掉。
            const float scrollH = ImGui::GetStyle().ScrollbarSize;
            if (const ImVec2 sysTblSize(0.0f, rowH * static_cast<float>(sysRows + 1) + scrollH + 8.0f); ImGui::BeginTable(
                "##sys", 13, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX |
                             ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit, sysTblSize)) {
                // 列宽按表头文本给足：总宽超出窗口时 auto 会被压到比文字还窄，表头就压到邻列上；
                // 定死宽度让表格横向滚动，而不是把 13 列挤成一团。
                const float padX = ImGui::GetStyle().CellPadding.x * 2.0f + 6.0f;
                for (auto &kSysColHdr: kSysColHdrs)
                    ImGui::TableSetupColumn(kSysColHdr, ImGuiTableColumnFlags_WidthFixed,
                                            ImGui::CalcTextSize(kSysColHdr).x + padX);
                ImGui::TableHeadersRow();

                if (ImGui::TableGetColumnFlags(12) & ImGuiTableColumnFlags_IsHovered) {
                    if (ImGui::BeginTooltip()) {
                        const auto &ce = rep.comprehensive;
                        ImGui::TextDisabled(I18n::trc("qc.entropy_hint"));
                        for (std::size_t l = 0; l < ce.indicators.size(); l++)
                            ImGui::Text("%s: %.4f", I18n::tr(ce.indicators[l]).c_str(), ce.weights[l]);
                        ImGui::EndTooltip();
                    }
                }

                for (const char s: {'G', 'R', 'E', 'C'})
                    if (rep.sysCompleteness.count(s)) writeRow(s);
                if (!rep.sysCompleteness.empty()) writeRow('T');
                ImGui::EndTable();
            }
        }


        void renderExportBar(QcUiState &qs, const QualityReport &rep,
                             const std::string &taskFileName) {
            if (ImGui::Button(I18n::trc("qc.export_csv"))) {
                exportCSV(rep, taskFileName);
            }
            ImGui::SameLine();
            if (ImGui::Button(I18n::trc("qc.export_all_png"))) {
                if (std::wstring dir; ShowFolderDialog(dir)) {
                    qs.exportDir = Str::wideToUTF8(dir);
                    qs.exportQueue = qs.lastIds;
                    qs.exporting = !qs.exportQueue.empty();
                }
            }


            if (qs.exporting)
                ImGui::TextColored(UI::kColInfo, I18n::trc("qc.export_all_progress"),
                                   static_cast<int>(qs.exportQueue.size()));
            else if (!qs.exportDir.empty())
                ImGui::TextDisabled(I18n::trc("qc.last_export_dir"), qs.exportDir.c_str());
        }


        bool renderSatelliteSelector(const QualityReport &rep, QcUiState &qs, const int sysSel,
                                     std::vector<SatID> &list,
                                     std::vector<const SatQC *> &toPlot, bool &allView) {
            ImGui::SeparatorText(I18n::trc("qc.section.by_sat"));
            for (auto &sat: rep.satOrder) {
                if (sysSel == 0 || (sysSel == 1 && sat.system == 'G') ||
                    (sysSel == 2 && sat.system == 'R') || (sysSel == 3 && sat.system == 'E') ||
                    (sysSel == 4 && sat.system == 'C')) list.push_back(sat);
            }
            if (list.empty()) {
                ImGui::Text(I18n::trc("qc.no_sats"));
                return false;
            }
            int &comboSel = qs.comboSel;
            if (comboSel < 0 || comboSel > static_cast<int>(list.size())) comboSel = 0;
            std::string items = I18n::trc("common.all");
            items += '\0';
            for (auto &s: list) {
                items += s.toString();
                items += '\0';
            }
            items += '\0';
            ImGui::Combo(I18n::trc("qc.select_sat"), &comboSel, items.c_str());
            allView = comboSel == 0;

            if (allView) for (auto &sat: list) toPlot.push_back(&rep.sats.at(sat));
            else if (comboSel >= 1 && comboSel <= static_cast<int>(list.size())) toPlot.push_back(&rep.sats.at(list[comboSel - 1]));

            return true;
        }


        void renderAllSatTimeSeries(QcUiState &qs, const std::string &baseName,
                                    const std::vector<const SatQC *> &toPlot) {
            drawPlot(qs, baseName, "mp1_all", I18n::trc("qc.plot.mp1_all"), kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m");
                for (const auto q: toPlot) {
                    if (q->t.empty()) continue;
                    const auto &ax = axisOf(qs, *q);
                    const std::vector<double> v = ax.align(q->mp1Resid);
                    ImPlot::PlotLine(q->sat.toString().c_str(), ax.t.data(), v.data(), static_cast<int>(ax.t.size()));
                }
            });
            drawPlot(qs, baseName, "mp2_all", I18n::trc("qc.plot.mp2_all"), kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m");
                for (const auto q: toPlot) {
                    if (q->t.empty()) continue;
                    const auto &ax = axisOf(qs, *q);
                    const std::vector<double> v = ax.align(q->mp2Resid);
                    ImPlot::PlotLine(q->sat.toString().c_str(), ax.t.data(), v.data(), static_cast<int>(ax.t.size()));
                }
            });
            drawPlot(qs, baseName, "iod_all", I18n::trc("qc.plot.iod_all"), kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "m/s");
                for (const auto q: toPlot) {
                    if (q->t.empty()) continue;
                    const auto &ax = axisOf(qs, *q);
                    const std::vector<double> v = ax.align(q->ionoRate);
                    ImPlot::PlotLine(q->sat.toString().c_str(), ax.t.data(), v.data(), static_cast<int>(ax.t.size()));
                }
            });
            drawPlot(qs, baseName, "cnr_all", I18n::trc("qc.plot.cnr_all"), kPlotH, [&] {
                ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                for (const auto q: toPlot) {
                    if (q->t.empty()) continue;
                    const auto &ax = axisOf(qs, *q);
                    const std::vector<double> v = ax.align(q->cnr);
                    ImPlot::PlotLine(q->sat.toString().c_str(), ax.t.data(), v.data(), static_cast<int>(ax.t.size()));
                }
            });
        }




        void fillBarBuf(QcUiState::PlotBuf &buf, const std::vector<SatID> &list,
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


        void renderPerSatMetricBars(const QualityReport &rep, QcUiState &qs,
                                    const std::string &baseName,
                                    const std::vector<SatID> &list) {

            ImGui::SeparatorText(I18n::trc("qc.section.sat_metrics"));
            {
                auto &buf = qs.buf("completeness");
                fillBarBuf(buf, list, rep, [](const SatQC &q) { return q.completeness; });
                const auto M = static_cast<int>(list.size());
                drawPlot(qs, baseName, "completeness", I18n::trc("qc.plot.completeness"), kPlotH, [&] {
                    ImPlot::SetupAxis(ImAxis_X1, "");
                    ImPlot::SetupAxis(ImAxis_Y1, "%");
                    auto &[col, labs, labp] = buf;
                    ImPlot::SetupAxisTicks(ImAxis_X1, col[0].data(), static_cast<int>(labs.size()), labp.data());
                    ImPlot::PlotBars(I18n::trc("qc.legend.completeness"), col[1].data(), static_cast<int>(col[1].size()), 0.7);
                });
            }

            {
                const int M = static_cast<int>(list.size());
                auto &[col, labs, labp] = qs.buf("slipratio");
                auto &jx = col[0], &jy = col[1];
                jx.clear();
                jy.clear();
                std::vector<const SatQC *> slid;
                for (int i = 0; i < M; i++)
                    if (const auto &q = rep.sats.at(list[i]); q.slips > 0) slid.push_back(&q);
                const auto jc = static_cast<int>(slid.size());
                labs.resize(jc);
                labp.resize(jc);
                for (int i = 0; i < jc; i++) {
                    jx.push_back(i);
                    jy.push_back(slid[i]->slipRatio);
                    labs[i] = slid[i]->sat.toString();
                    labp[i] = labs[i].c_str();
                }
                if (jc == 0) ImGui::TextDisabled(I18n::trc("qc.no_slips"));
                else
                    drawPlot(qs, baseName, "slipratio", I18n::trc("CycleSlipRatio"), kPlotH, [&] {
                        const QcUiState::PlotBuf &_b = qs.buf("slipratio");
                        ImPlot::SetupAxis(ImAxis_X1, "");
                        ImPlot::SetupAxis(ImAxis_Y1, I18n::trc("qc.axis.epoch_slip"));
                        ImPlot::SetupAxisTicks(ImAxis_X1, _b.col[0].data(), static_cast<int>(_b.labp.size()),
                                               _b.labp.data());
                        ImPlot::PlotBars(I18n::trc("CycleSlipRatio"), _b.col[1].data(), static_cast<int>(_b.col[1].size()), 0.7);
                    });
            }
            drawPlot(qs, baseName, "noise_combined", I18n::trc("qc.plot.noise_combined"), kPlotH, [&] {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, I18n::trc("qc.col.code_noise"));
                ImPlot::SetupAxis(ImAxis_Y2, I18n::trc("qc.col.phase_noise"), ImPlotAxisFlags_AuxDefault);
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
                ImPlot::PlotBars(I18n::trc("PseudorangeNoise"), xsR.data(), rho.data(), M, 0.38);
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
                ImPlot::PlotBars(I18n::trc("CarrierNoise"), xsP.data(), ph.data(), M, 0.38);
            });
        }


        void renderComprehensiveBars(const QualityReport &rep, QcUiState &qs,
                                     const std::string &baseName,
                                     const std::vector<SatID> &list) {

            const auto &ce = rep.comprehensive;
            auto &buf = qs.buf("comprehensive");
            fillBarBuf(buf, list, rep, [&ce](const SatQC &q) { return atOr(ce.E, q.sat); });
            const auto M = static_cast<int>(list.size());
            drawPlot(qs, baseName, "comprehensiveE", I18n::trc("qc.plot.assessment_e"), kPlotH, [&] {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, "E");
                auto &[col, labs, labp] = buf;
                ImPlot::SetupAxisTicks(ImAxis_X1, col[0].data(), M, labp.data());
                ImPlot::PlotBars(I18n::trc("qc.col.composite_e"), col[1].data(), M, 0.7);
            });
        }


        void renderSingleSatMetricTable(const QualityReport &rep, const SatQC &q) {
            if (ImGui::BeginTable("##satqc", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn(I18n::trc("qc.col.indicator"));
                ImGui::TableSetupColumn(I18n::trc("common.value"));
                ImGui::TableHeadersRow();
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.row.freq"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%c / %c", q.band1, q.band2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("Completeness"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f %% (%d/%d)", q.completeness, q.validDual, q.totalEpochs);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.row.di"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f %% / %.2f %%", q.di_f1, q.di_f2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.row.slips"));
                ImGui::TableSetColumnIndex(1);
                if (q.slips > 0) ImGui::Text("%d / %.1f", q.slips, q.slipRatio);
                else ImGui::Text("%d / ∞", q.slips);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.row.outliers"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d / %d", q.outliers, q.clockJumps);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("MP1 / MP2 (RMS)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3f / %.3f m", q.mp1Rms, q.mp2Rms);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.row.code_noise"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3f / %.3f m", q.sigRho1, q.sigRho2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.row.phase_noise"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.4f / %.4f cyc", q.sigPh1, q.sigPh2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.row.iod"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text(I18n::trc("qc.row.iod_std"),
                            q.ionoRateStd1, q.ionoRateStd2, q.ionoJumps1, q.ionoJumps2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("CNR f1 / f2 (mean/min/max)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.1f / %.1f / %.1f | %.1f / %.1f / %.1f dB-Hz",
                            q.cnrMean, q.cnrMin, q.cnrMax, q.cnrMean2, q.cnrMin2, q.cnrMax2);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(I18n::trc("qc.col.assessment_e"));
                ImGui::TableSetColumnIndex(1);
                if (const auto itE = rep.comprehensive.E.find(q.sat); itE != rep.comprehensive.E.end())
                    ImGui::Text("%.4f", itE->second);
                else
                    ImGui::Text("-");
                ImGui::EndTable();
            }
        }


        void renderSingleSatView(const QualityReport &rep, QcUiState &qs,
                                 const std::string &baseName,
                                 const std::vector<const SatQC *> &toPlot) {
            const SatQC &q = *toPlot[0];
            renderSingleSatMetricTable(rep, q);
            if (!q.t.empty()) {
                const std::vector<double> &mp1v = q.mp1Resid;
                const std::vector<double> &mp2v = q.mp2Resid;
                const auto &ax = axisOf(qs, q);
                drawPlot(qs, baseName, "s_mp", I18n::trc("qc.plot.sat_mp"), kPlotH, [&] {
                    ImPlot::SetupAxes("SOW (s)", "m");
                    const std::vector<double> a = ax.align(mp1v), b = ax.align(mp2v);
                    ImPlot::PlotLine("MP1", ax.t.data(), a.data(), static_cast<int>(ax.t.size()));
                    ImPlot::PlotLine("MP2", ax.t.data(), b.data(), static_cast<int>(ax.t.size()));
                });
                std::vector<double> &sl_t = qs.buf("s_slip").col[0];
                std::vector<double> &sl_v = qs.buf("s_slip").col[1];
                sl_t.clear();
                sl_v.clear();
                const std::size_t ns = std::min({q.t.size(), q.slipFlag.size(), mp1v.size()});
                for (std::size_t i = 0; i < ns; i++)
                    if (q.slipFlag[i]) {
                        sl_t.push_back(q.t[i]);
                        sl_v.push_back(mp1v[i]);
                    }
                if (!sl_t.empty())
                    drawPlot(qs, baseName, "s_slip", I18n::trc("qc.plot.sat_slip"), kPlotH, [&] {
                        ImPlot::SetupAxes("SOW (s)", "m");
                        ImPlot::PlotScatter(I18n::trc("CycleSlip"), sl_t.data(), sl_v.data(), static_cast<int>(sl_t.size()));
                    });
                drawPlot(qs, baseName, "s_iod", I18n::trc("qc.plot.iod_all"), kPlotH, [&] {
                    ImPlot::SetupAxes("SOW (s)", "m/s");
                    const std::vector<double> a = ax.align(q.ionoRate);
                    ImPlot::PlotLine("IOD f1", ax.t.data(), a.data(), static_cast<int>(ax.t.size()));
                    if (!q.ionoRate2.empty()) {
                        const std::vector<double> b = ax.align(q.ionoRate2);
                        ImPlot::PlotLine("IOD f2", ax.t.data(), b.data(), static_cast<int>(ax.t.size()));
                    }
                });
                if (!q.cnr.empty() || !q.cnr2.empty())
                    drawPlot(qs, baseName, "s_cnr", I18n::trc("qc.plot.cnr_all"), kPlotH, [&] {
                        ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                        if (!q.cnr.empty()) {
                            const std::vector<double> a = ax.align(q.cnr);
                            ImPlot::PlotLine("CNR f1", ax.t.data(), a.data(), static_cast<int>(ax.t.size()));
                        }
                        if (!q.cnr2.empty() && q.cnr2.size() == q.t.size()) {
                            const std::vector<double> b = ax.align(q.cnr2);
                            ImPlot::PlotLine("CNR f2", ax.t.data(), b.data(), static_cast<int>(ax.t.size()));
                        }
                    });
            }
        }


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
                            Str::UTF8ToWide(qs.exportDir + "/" + baseName + "_" + Str::sanitizeId(id) + ".png"),
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
    }

    void render(Solvers::View &view, const std::string &fileName) {
        if (view.qcDirty && !view.qcComputing) {
            view.qcDirty = false;
            Solvers::SolverPanel::launchQC(view);
        }

        std::shared_ptr<QualityReport> repPtr;
        {
            std::lock_guard lk(view.qcMutex);
            repPtr = view.qcReport;
        }
        if (!repPtr || !view.qcReady) {
            if (view.qcComputing) {
                ImGui::TextColored(UI::kColInfo, I18n::trc("qc.solving"));
            } else {
                ImGui::TextColored(UI::kColInfo, I18n::trc("qc.preparing"));
            }
            return;
        }
        const QualityReport &rep = *repPtr;
        if (rep.sats.empty()) {
            ImGui::TextColored(UI::kColWarn, I18n::trc("qc.no_dual_freq"));
            return;
        }
        QcUiState &qs = uiState(view.uiTicket);
        if (qs.axes.report != repPtr.get()) {
            qs.axes.report = repPtr.get();
            qs.axes.interval = rep.interval;
            qs.axes.bySat.clear();
        }

        ImGui::BeginChild("##qcplots", ImVec2(0, 0));

        // 批量导出时把队列首图滚动到视口顶部
        if (qs.exporting && !qs.exportQueue.empty()) {
            if (const auto it = qs.contentY.find(qs.exportQueue.front()); it != qs.contentY.end())
                ImGui::SetScrollY(it->second - 6.0f);
        }

        int &sysSel = qs.sysSel;
        ImGui::RadioButton(I18n::trc("common.all"), &sysSel, 0);
        ImGui::SameLine();
        ImGui::RadioButton("GPS", &sysSel, 1);
        ImGui::SameLine();
        ImGui::RadioButton("GLO", &sysSel, 2);
        ImGui::SameLine();
        ImGui::RadioButton("GAL", &sysSel, 3);
        ImGui::SameLine();
        ImGui::RadioButton("BDS", &sysSel, 4);

        renderOverviewStats(rep);

        const std::string baseName = File::stripExt(fileName);
        renderExportBar(qs, rep, fileName);

        std::vector<SatID> list;
        std::vector<const SatQC *> toPlot;
        bool allView = false;
        if (!renderSatelliteSelector(rep, qs, sysSel, list, toPlot, allView)) {
            ImGui::EndChild();
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

        ImGui::EndChild();

        renderExportOverlay(qs, baseName);
    }
}
