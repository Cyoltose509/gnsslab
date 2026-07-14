#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
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
#include <cctype>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>

namespace QualityControl {

// ---- 逐图尺寸 / 批量导出状态（跨帧持久）----
struct QcPlotSize { int w = 0; int h = 340; };          // w=0 表示占满宽度
struct QcUiState {
    std::map<std::string, QcPlotSize> sizes;             // 每个图的用户自定义宽高
    std::map<std::string, ImVec4>     curRect;           // 本帧记录：id -> (screenX, screenY, w, h)
    std::map<std::string, float>      contentY;          // 本帧记录：id -> 在滚动子窗口内的内容纵坐标（用于批量导出时定位）
    std::vector<std::string>          lastIds;            // 上一帧实际绘制的图 id 列表
    std::vector<std::string>          exportQueue;        // 批量导出剩余队列
    std::string                       exportDir;          // UTF-8 文件夹路径
    bool                              exporting = false;
};
static QcUiState &uiState() { static QcUiState s; return s; }

// UTF-8 <-> 宽字符（用于路径拼接后传给截屏 API）
static std::wstring Utf8ToWide(const std::string &s) {
    if (s.empty()) return {};
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring w(n > 0 ? n - 1 : 0, L'\0');
    if (n > 0) MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
    return w;
}
static std::string WideToUtf8(const std::wstring &w) {
    if (w.empty()) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string s(n > 0 ? n - 1 : 0, '\0');
    if (n > 0) WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
    return s;
}
static std::string sanitizeId(const std::string &s) {
    std::string r; r.reserve(s.size());
    for (char c : s) r += (std::isalnum((unsigned char)c) || c == '_' || c == '-') ? c : '_';
    return r;
}

// 将质量分析报告导出为 CSV（含系统汇总、逐卫星统计两段）。
// 仅导出「纯观测质量指标」：MP/IOD/噪声/周跳/完整率/SNR。
// DOP/卫星数/高度角等依赖定位解算的几何量不在质量分析范畴，故不导出。
// 使用 Win32 保存对话框，输出 UTF-8 BOM 以便 Excel 正确显示中文。
static void exportCsv(const QC::QualityReport &rep, const std::string &taskLabel) {
    static const GuiFileFilter filters[] = {
        {L"CSV 文件 (*.csv)", L"*.csv"}, {L"所有文件 (*.*)", L"*.*"}};
    std::wstring path;
    if (!ShowSaveFileDialog(path, filters, 2, L"qc_report.csv", L"csv")) return;   // 用户取消

    std::ofstream out(std::filesystem::path(path), std::ios::out);
    if (!out) return;
    out << "\xEF\xBB\xBF";                 // UTF-8 BOM
    auto sysName = [](char s) -> const char * {
        if (s == 'G') return "GPS"; if (s == 'C') return "BDS";
        if (s == 'R') return "GLO"; if (s == 'E') return "GAL"; return "?";
    };
    out << std::fixed << std::setprecision(3);

    out << "GNSS 观测数据质量分析报告," << taskLabel << "\n";
    out << "总历元," << rep.totalEpochs << ",间隔(s)," << rep.interval
        << ",可用卫星," << rep.sats.size() << ",总体完整率%," << rep.overallCompleteness
        << ",平均SNR(dB)," << rep.overallSnr << "\n\n";

    // ---- 系统汇总 ----
    out << "[系统汇总]\n";
    out << "系统,完整率%,周跳数,周跳比,MP1(m),MP2(m),伪距噪声(m),载波噪声(cyc),SNR(dB)\n";
    for (auto &[s, comp] : rep.sysCompleteness) {
        out << sysName(s) << ","
            << comp << "," << rep.sysSlips.at(s) << "," << rep.sysSlipRatio.at(s)
            << "," << rep.sysMp1.at(s) << "," << rep.sysMp2.at(s)
            << "," << rep.sysSigRho.at(s) << "," << rep.sysSigPhase.at(s)
            << "," << (rep.sysSnr.count(s) ? rep.sysSnr.at(s) : 0.0) << "\n";
    }
    out << "\n";

    // ---- 逐卫星统计 ----
    out << "[逐卫星统计]\n";
    out << "卫星,系统,频点f1,频点f2,完整率%,周跳数,周跳比,MP1(m),MP2(m),"
           "伪距噪声(m),载波噪声(cyc),SNR(dB),IOD_std(m/s),IOD_跳变\n";
    for (auto &sat : rep.satOrder) {
        const auto &q = rep.sats.at(sat);
        out << q.sat.toString() << "," << sysName(q.sat.system) << ","
            << q.band1 << "," << q.band2 << ","
            << q.completeness << "," << q.slips << "," << q.slipRatio << ","
            << q.mp1Rms << "," << q.mp2Rms
            << "," << (0.5 * (q.sigRho1 + q.sigRho2)) << "," << (0.5 * (q.sigPh1 + q.sigPh2))
            << "," << q.snrMean
            << "," << q.ionoRateStd << "," << q.ionoJumps << "\n";
    }
    out << "\n";
    out.flush();
}

// 「导出此图 PNG」小按钮：只裁剪该图矩形 [p0, p0+(w,h)]（屏幕/后台缓冲像素坐标）。
// 需紧跟对应 ImPlot::EndPlot() 之后调用；p0/w/h 取自该图 BeginPlot 前的光标位置与尺寸。
static void plotExportBtn(const char *uid, ImVec2 p0, float w, float h) {
    std::string lbl = std::string("导出此图 PNG##") + uid;
    if (ImGui::SmallButton(lbl.c_str())) {
        static const GuiFileFilter filters[] = {
            {L"PNG 文件 (*.png)", L"*.png"}, {L"所有文件 (*.*)", L"*.*"}};
        std::wstring path;
        if (ShowSaveFileDialog(path, filters, 2, L"qc_plot.png", L"png"))
            RequestCaptureRegionPNG(path, (int)p0.x, (int)p0.y, (int)w, (int)h);
    }
}

void render(const std::shared_ptr<GuiFileProcessor::SppTask> &task) {
    // QC 报告由后台线程(SolveThread 内 launchQC)在「读完后」算一次，渲染线程只读取缓存。
    // 质量分析仅用原始观测(C/L/S)，不依赖定位解算 → 算一次即可，无论文件多大都不会卡。
    std::shared_ptr<QualityReport> repPtr;
    { std::lock_guard lk(task->qcMutex); repPtr = task->qcReport; }
    if (!repPtr || !task->qcReady) {
        if (task->qcComputing) {
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "正在计算质量分析…（后台线程，不阻塞界面）");
            ImGui::ProgressBar(task->qcProgress.load(), ImVec2(320, 0), "%.0f%%");
        } else {
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "正在准备质量分析…（等待数据读入）");
        }
        return;
    }
    const QualityReport &rep = *repPtr;
    if (rep.sats.empty()) { ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "无有效双频观测，无法进行质量分析。"); return; }
    if (!task->done)
        ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f),
                "质量分析仅依赖原始观测（C/L/S），与定位解算无关；解算进行中本页指标不受影响，无需等待。");
    QcUiState &qs = uiState();

    // 整页可滚动：顶部控制区与下方所有图同处一个滚动容器，顶部不再独占半屏
    ImGui::BeginChild("##qcplots", ImVec2(0, 0));

    static int sysSel = 0;
    ImGui::RadioButton("全部", &sysSel, 0); ImGui::SameLine();
    ImGui::RadioButton("GPS", &sysSel, 1); ImGui::SameLine();
    ImGui::RadioButton("BDS", &sysSel, 2);

    ImGui::SeparatorText("总体概览");
    if (rep.totalInputEpochs > rep.totalEpochs)
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.4f, 1.0f),
                "总历元 %d（已解算 %d，未解算 %d）| 间隔 %.2f s | 可用卫星 %d | 总体完整率 %.2f%% | SNR %.1f",
                rep.totalInputEpochs, rep.totalEpochs, rep.totalInputEpochs - rep.totalEpochs, rep.interval,
                (int)rep.sats.size(), rep.overallCompleteness, rep.overallSnr);
    else
        ImGui::Text("总历元 %d | 间隔 %.2f s | 可用卫星 %d | 总体完整率 %.2f%% | SNR %.1f",
                rep.totalEpochs, rep.interval, (int)rep.sats.size(), rep.overallCompleteness, rep.overallSnr);
    if (ImGui::BeginTable("##sys", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("系统"); ImGui::TableSetupColumn("完整率%"); ImGui::TableSetupColumn("周跳数");
        ImGui::TableSetupColumn("周跳比"); ImGui::TableSetupColumn("MP1(m)"); ImGui::TableSetupColumn("MP2(m)");
        ImGui::TableSetupColumn("伪距噪声(m)"); ImGui::TableSetupColumn("载波噪声(cyc)");
        ImGui::TableSetupColumn("SNR(dB)");
        ImGui::TableHeadersRow();
        auto addSys = [&](char s, const char *name) {
            if (!rep.sysCompleteness.count(s)) return;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("%s", name);
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", rep.sysCompleteness.at(s));
            ImGui::TableSetColumnIndex(2); ImGui::Text("%d", rep.sysSlips.at(s));
            ImGui::TableSetColumnIndex(3);
            if (rep.sysSlips.at(s) > 0) ImGui::Text("%.1f", rep.sysSlipRatio.at(s));
            else ImGui::Text("∞");
            ImGui::TableSetColumnIndex(4); ImGui::Text("%.3f", rep.sysMp1.at(s));
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.3f", rep.sysMp2.at(s));
            ImGui::TableSetColumnIndex(6); ImGui::Text("%.3f", rep.sysSigRho.at(s));
            ImGui::TableSetColumnIndex(7); ImGui::Text("%.4f", rep.sysSigPhase.at(s));
            ImGui::TableSetColumnIndex(8); ImGui::Text("%.1f", rep.sysSnr.count(s) ? rep.sysSnr.at(s) : 0.0);
        };
        addSys('G', "GPS"); addSys('C', "BDS");
        ImGui::EndTable();
    }

    if (ImGui::Button("导出 CSV (系统汇总/逐卫星)")) {
        exportCsv(rep, task->fileName);
    }
    ImGui::SameLine();
    if (ImGui::Button("导出整窗 PNG")) {
        static const GuiFileFilter filters[] = {
            {L"PNG 文件 (*.png)", L"*.png"}, {L"所有文件 (*.*)", L"*.*"}};
        std::wstring path;
        if (ShowSaveFileDialog(path, filters, 2, L"qc_window.png", L"png"))
            RequestCapturePNG(path);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(每张图下方有「导出此图 PNG」，只导出该图)");

    // ===== 一键导出全部图到指定文件夹 =====
    if (ImGui::Button("📦 一键导出全部图 PNG (选文件夹)", ImVec2(-1, 0))) {
        std::wstring dir;
        if (ShowFolderDialog(dir)) {
            qs.exportDir = WideToUtf8(dir);
            qs.exportQueue = qs.lastIds;          // 用上一帧已绘制的图 id 列表
            qs.exporting = !qs.exportQueue.empty();
        }
    }
    if (qs.exporting)
        ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1), "正在批量导出：剩余 %d 张（每帧 1 张，请勿操作）…",
                           (int)qs.exportQueue.size());
    else if (!qs.exportDir.empty())
        ImGui::TextDisabled("上次导出目录：%s", qs.exportDir.c_str());

    ImGui::SeparatorText("逐卫星统计");
    std::vector<SatID> list;
    for (auto &sat : rep.satOrder) {
        if (sysSel == 0 || (sysSel == 1 && sat.system == 'G') || (sysSel == 2 && sat.system == 'C')) list.push_back(sat);
    }
    if (list.empty()) { ImGui::Text("无符合条件的卫星。"); ImGui::EndChild(); return; }
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

    // ===== 绘图区（处于整页滚动容器内；每张图底边可上下拖拽缩放；支持批量导出）=====

    // 批量导出：把队列中下一张图滚动到顶部，便于本帧截到它
    if (qs.exporting && !qs.exportQueue.empty()) {
        auto it = qs.contentY.find(qs.exportQueue.front());
        if (it != qs.contentY.end()) ImGui::SetScrollY(it->second - 6.0f);
    }
    qs.curRect.clear();
    qs.lastIds.clear();

    // 统一绘图 helper：每张图包进带边框的子窗口；右下角有可拖拽缩放手柄
    // （鼠标移到角落变 ↘ 光标，按住拖动即可改宽高，无需填写数字）。
    // 记录屏幕矩形（用于「导出此图」与「批量导出」），全程不出现宽/高输入框。
    auto drawPlot = [&](const char *id, const char *title, float defH,
                        const std::function<void()> &body) {
        if (!qs.sizes.count(id)) qs.sizes[id] = QcPlotSize{0, (int)defH};
        QcPlotSize &sz = qs.sizes[id];

        const float availW = ImGui::GetContentRegionAvail().x;
        const float w = availW;                  // 宽度始终占满，仅支持上下拉伸
        float h = (float)sz.h; if (h < 80.0f) h = 80.0f;

        const ImVec2 p0 = ImGui::GetCursorScreenPos();   // 框架左上（屏幕坐标）
        qs.contentY[id] = ImGui::GetCursorPosY();         // 滚动子窗口内纵坐标（批量导出滚动定位用）
        qs.curRect[id]  = ImVec4(p0.x, p0.y, w, h);
        qs.lastIds.push_back(id);

        if (ImGui::BeginChild((std::string("##frame_") + id).c_str(),
                              ImVec2(w, h), ImGuiChildFlags_Borders)) {
            if (ImPlot::BeginPlot(title, ImVec2(-1, -1))) { body(); ImPlot::EndPlot(); }

            // ----- 底边拖拽手柄（仅上下拉伸高度）-----
            const ImVec2 cmax = ImGui::GetWindowContentRegionMax();
            ImGui::SetCursorPos(ImVec2(0.0f, cmax.y - 8.0f));
            ImGui::InvisibleButton((std::string("##grip_") + id).c_str(), ImVec2(cmax.x, 8.0f));
            if (ImGui::IsItemHovered() || ImGui::IsItemActive())
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
            if (ImGui::IsItemActive()) {
                const float d = ImGui::GetIO().MouseDelta.y;
                sz.h = (int)((float)sz.h + d < 80.0f ? 80.0f : (float)sz.h + d);
            }
            // 手柄视觉提示（中间一条横线）
            ImDrawList *dl = ImGui::GetWindowDrawList();
            const ImVec2 gmin = ImGui::GetItemRectMin(), gmax = ImGui::GetItemRectMax();
            dl->AddLine(ImVec2(gmin.x + 6, (gmin.y + gmax.y) * 0.5f),
                        ImVec2(gmax.x - 6, (gmin.y + gmax.y) * 0.5f),
                        IM_COL32(190, 190, 190, 255), 1.5f);
        }
        ImGui::EndChild();

        plotExportBtn(id, p0, w, h);
    };

    if (allView) {
        ImGui::TextDisabled("已选「全部卫星」：下列曲线按卫星叠加（每条线为一个卫星，可点击图例隐藏）。");
        drawPlot("mp1_all", "MP1 (全部卫星)", 360, [&] {
            ImPlot::SetupAxes("SOW (s)", "m");
            for (auto q : toPlot) if (!q->t.empty())
                ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->mp1.data(), (int)q->t.size());
        });
        drawPlot("mp2_all", "MP2 (全部卫星)", 360, [&] {
            ImPlot::SetupAxes("SOW (s)", "m");
            for (auto q : toPlot) if (!q->t.empty())
                ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->mp2.data(), (int)q->t.size());
        });
        drawPlot("iod_all", "电离层残差变化率 IOD (全部卫星)", 360, [&] {
            ImPlot::SetupAxes("SOW (s)", "m/s");
            for (auto q : toPlot) if (!q->t.empty())
                ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->ionoRate.data(), (int)q->t.size());
        });
        drawPlot("snr_all", "信噪比 SNR (全部卫星)", 360, [&] {
            ImPlot::SetupAxes("SOW (s)", "dB-Hz");
            for (auto q : toPlot) if (!q->t.empty())
                ImPlot::PlotLine(q->sat.toString().c_str(), q->t.data(), q->snr.data(), (int)q->t.size());
        });

        // ===== 逐卫星指标柱状图 =====
        ImGui::SeparatorText("逐卫星指标 (柱状图)");
        drawPlot("completeness", "观测完整率 (%)", 300, [&] {
            ImPlot::SetupAxis(ImAxis_X1, "");
            ImPlot::SetupAxis(ImAxis_Y1, "%");
            int M = (int)list.size();
            std::vector<double> xs(M), ys(M);
            std::vector<std::string> labs(M); const char *labp[128];
            for (int i = 0; i < M; i++) {
                const auto &q = rep.sats.at(list[i]);
                xs[i] = i; ys[i] = q.completeness; labs[i] = q.sat.toString();
                if (i < 128) labp[i] = labs[i].c_str();
            }
            ImPlot::SetupAxisTicks(ImAxis_X1, xs.data(), M, labp);
            ImPlot::PlotBars("观测完整率", ys.data(), M, 0.7);
        });
        // 周跳比：仅展示发生过周跳的卫星；无周跳卫星周跳比→∞（不画柱，避免被 250 类的占位值误导）
        {
            int M = (int)list.size();
            std::vector<double> jx, jy; std::vector<std::string> jl(M); const char *jlp[128]; int jc = 0;
            for (int i = 0; i < M; i++) {
                const auto &q = rep.sats.at(list[i]);
                if (q.slips > 0) {
                    jx.push_back(jc); jy.push_back(q.slipRatio); jl[jc] = q.sat.toString();
                    if (jc < 128) jlp[jc] = jl[jc].c_str();
                    jc++;
                }
            }
            if (jc == 0) ImGui::TextDisabled("所有卫星均未检测到周跳 → 周跳比 ∞（已跳过柱状图）。");
            else drawPlot("slipratio", "周跳比 (历元/周跳, 仅含发生周跳的卫星)", 300, [&] {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, "历元/周跳");
                ImPlot::SetupAxisTicks(ImAxis_X1, jx.data(), jc, jlp);
                ImPlot::PlotBars("周跳比", jy.data(), jc, 0.7);
            });
        }
        drawPlot("noise_combined", "伪距噪声 / 载波噪声 (全部卫星)", 300, [&] {
            ImPlot::SetupAxis(ImAxis_X1, "");
            ImPlot::SetupAxis(ImAxis_Y1, "伪距噪声 (m)");
            ImPlot::SetupAxis(ImAxis_Y2, "载波噪声 (cyc)", ImPlotAxisFlags_AuxDefault);
            int M = (int)list.size();
            std::vector<double> xs(M), xsR(M), xsP(M), rho(M), ph(M);
            std::vector<std::string> labs(M); const char *labp[128];
            for (int i = 0; i < M; i++) {
                const auto &q = rep.sats.at(list[i]);
                xs[i] = i; xsR[i] = i - 0.2; xsP[i] = i + 0.2;
                rho[i] = 0.5 * (q.sigRho1 + q.sigRho2); ph[i] = 0.5 * (q.sigPh1 + q.sigPh2);
                labs[i] = q.sat.toString(); if (i < 128) labp[i] = labs[i].c_str();
            }
            ImPlot::SetupAxisTicks(ImAxis_X1, xs.data(), M, labp);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
            ImPlot::PlotBars("伪距噪声", xsR.data(), rho.data(), M, 0.38);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            ImPlot::PlotBars("载波噪声", xsP.data(), ph.data(), M, 0.38);
        });
    } else if (!toPlot.empty()) {
        const SatQC &q = *toPlot[0];
        if (ImGui::BeginTable("##satqc", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("指标"); ImGui::TableSetupColumn("值");
            ImGui::TableHeadersRow();
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("频点 f1 / f2"); ImGui::TableSetColumnIndex(1); ImGui::Text("%c / %c", q.band1, q.band2);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("完整率"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f %% (%d/%d)", q.completeness, q.validDual, q.totalEpochs);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("周跳数 / 周跳比"); ImGui::TableSetColumnIndex(1);
            if (q.slips > 0) ImGui::Text("%d / %.1f", q.slips, q.slipRatio);
            else ImGui::Text("%d / ∞", q.slips);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("MP1 / MP2 (RMS)"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f / %.3f m", q.mp1Rms, q.mp2Rms);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("伪距噪声"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f / %.3f m", q.sigRho1, q.sigRho2);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("载波相位噪声"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f / %.4f cyc", q.sigPh1, q.sigPh2);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("电离层残差变化率 IOD"); ImGui::TableSetColumnIndex(1); ImGui::Text("std %.4f m/s, 跳变 %d 次", q.ionoRateStd, q.ionoJumps);
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("SNR (mean/min/max)"); ImGui::TableSetColumnIndex(1); ImGui::Text("%.1f / %.1f / %.1f dB-Hz", q.snrMean, q.snrMin, q.snrMax);
            ImGui::EndTable();
        }
        if (!q.t.empty()) {
            const std::vector<double> &mp1v = q.mp1;
            const std::vector<double> &mp2v = q.mp2;
            drawPlot("s_mp", "多路径 MP1 / MP2", 360, [&] {
                ImPlot::SetupAxes("SOW (s)", "m");
                ImPlot::PlotLine("MP1", q.t.data(), mp1v.data(), (int)q.t.size());
                ImPlot::PlotLine("MP2", q.t.data(), mp2v.data(), (int)q.t.size());
            });
            std::vector<double> sl_t, sl_v;
            for (int i = 0; i < (int)q.t.size(); i++) if (q.slipFlag[i]) { sl_t.push_back(q.t[i]); sl_v.push_back(mp1v[i]); }
            if (!sl_t.empty())
                drawPlot("s_slip", "周跳位置 (MP1)", 360, [&] {
                    ImPlot::SetupAxes("SOW (s)", "m");
                    ImPlot::PlotScatter("周跳", sl_t.data(), sl_v.data(), (int)sl_t.size());
                });
            drawPlot("s_iod", "电离层残差变化率 IOD", 360, [&] {
                ImPlot::SetupAxes("SOW (s)", "m/s");
                ImPlot::PlotLine("IOD", q.t.data(), q.ionoRate.data(), (int)q.t.size());
            });
            if (!q.snr.empty())
                drawPlot("s_snr", "信噪比 SNR", 360, [&] {
                    ImPlot::SetupAxes("SOW (s)", "dB-Hz");
                    ImPlot::PlotLine("SNR", q.t.data(), q.snr.data(), (int)q.t.size());
                });
        }
    }

    // ===== 批量导出：每帧把队列首图（本帧已滚动到顶部）截为独立 PNG =====
    if (qs.exporting && !qs.exportQueue.empty()) {
        std::string id = qs.exportQueue.front();
        auto it = qs.curRect.find(id);
        if (it != qs.curRect.end()) {
            ImVec4 r = it->second;
            std::string fn = qs.exportDir + "/" + sanitizeId(id) + ".png";
            RequestCaptureRegionPNG(Utf8ToWide(fn), (int)r.x, (int)r.y, (int)r.z, (int)r.w);
        }
        qs.exportQueue.erase(qs.exportQueue.begin());
        if (qs.exportQueue.empty()) qs.exporting = false;
    }

    ImGui::EndChild();

}

} // namespace QualityControl
