#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <string>
#include <functional>
#include <set>
#include <map>
#include <vector>
#include <memory>      // std::shared_ptr
#include <mutex>       // std::lock_guard
#include <algorithm>   // std::transform
#include <cctype>      // ::tolower
#include <cstdio>      // snprintf
#include <utility>     // std::pair
#include <windows.h>
#include "imgui.h"
#include "ui/Gui.h"   // GuiFileFilter, ShowOpenFileDialog
#include "GnssStruct.h"  // SatID
#include "StringUtils.h" // ansiToWide / 全局 fmt3/fmt4

namespace GuiHelpers {
    bool inputTextStd(const char *label, std::string *str, ImGuiInputTextFlags flags = 0);

    void fileRow(const char *label, std::string *buf, const char *iniKey,
                 const GuiFileFilter *filters, int nFilters,
                 const std::function<void(const std::string &)> &onSelect = {});

    void renderCutoffAndSystems(double *cutoffDeg, std::set<char> *sysMask,
                                const std::function<void()> &extraSameLine = {});

    static constexpr GuiFileFilter fClk[] = {
        {L"精密钟差 CLK (*.CLK;*.clk)", L"*.CLK;*.clk"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fOsb[] = {
        {L"码偏差 OSB/BIA (*.BIA;*.bia;*.OSB)", L"*.BIA;*.bia;*.OSB"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fAtx[] = {
        {L"天线文件 ATX (*.atx;*.ATX)", L"*.atx;*.ATX"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fSp3[] = {
        {L"精密轨道 SP3 (*.SP3;*.sp3)", L"*.SP3;*.sp3"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fOrb[] = {
        {L"精密/广播星历 (*.SP3;*.sp3;*.??N;*.??G;*.??C;*.nav;*.rnx)", L"*.SP3;*.sp3;*.??N;*.??G;*.??C;*.nav;*.rnx"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fRnx[] = {
        {L"广播星历 (*.??N;*.??G;*.??C;*.nav;*.rnx)", L"*.??N;*.??G;*.??C;*.nav;*.rnx"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fObs[] = {
        {L"LEO 星载 RINEX (*.??O;*.rnx)", L"*.??O;*.rnx"},
        {L"OEM7 日志 (*.log)", L"*.log"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fHdr[] = {
        {L"辅助文件 (*.HDR;*.hdr)", L"*.HDR;*.hdr"},
        {L"所有文件 (*.*)", L"*.*"}
    };
    static constexpr GuiFileFilter fCsv[] = {
        {L"CSV (*.csv)", L"*.csv"},
        {L"所有文件 (*.*)", L"*.*"}
    };

    struct RobustResRange {
        double lo = -8.0, hi = 8.0;
    };

    RobustResRange computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes);


    // 路径取文件名（去掉目录部分）
    std::string baseName(const std::string &p);

    // 判断路径是否为 SP3 精密轨道文件
    bool isSp3Path(const std::string &path);

    // 解算系统 = 用户勾选掩码 ∩ 观测文件实际出现的系统；无交集时退回文件全部系统，
    std::set<char> resolveNavSystems(const std::set<char> &mask,
                                     const std::map<char, std::vector<std::string> > &mapObsTypes,
                                     const char fallbackSys = '\0');

    // 数值格式化 fmt3/fmt4 是全局函数（src/util/StringUtils.h），不在此命名空间声明；
    // 调用方直接用 fmt4/fmt3 即可（GuiHelpers.h 已包含 StringUtils.h）。

    // 配置窗口通用开头
    void beginConfigWindow(const std::string &idBase, void *task);

    // 「取消」按钮
    template<class Task>
    void renderCancelButton(const std::shared_ptr<Task> &task) {
        if (ImGui::Button("取消", ImVec2(80, 40))) {
            task->state = Task::State::Done;
            task->hasError = true;
            task->errorMsg = "用户取消";
        }
    }

    // CSV 导出：默认文件名 → 保存对话框。复用 StringUtils::ansiToWide（统一 SPP 原 ansiToWide 与 PPP/LEO 原 MultiByteToWideChar 两种写法）。
    bool saveCSVDialog(const std::string &defaultName, std::wstring &outPath);

    // 读取历元数量并钳制选中历元
    template<class Task>
    std::pair<int, int> readEpochView(const std::shared_ptr<Task> &task) {
        std::lock_guard lock(task->mutex);
        const int epochCount = static_cast<int>(task->epochs.size());
        if (task->selectedEpoch >= epochCount) task->selectedEpoch = epochCount - 1;
        if (task->selectedEpoch < 0 && epochCount > 0) task->selectedEpoch = 0;
        return {epochCount, task->selectedEpoch};
    }
} // namespace GuiHelpers
