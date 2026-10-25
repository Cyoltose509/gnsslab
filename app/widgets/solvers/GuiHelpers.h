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
#include "Log.h"         // renderCancelButton 记一行取消日志

namespace GnssTask {
    struct SolverTaskBase;
    struct RealtimeEndpoint;
}

namespace Solvers::Helpers {
    bool inputTextStd(const char *label, std::string *str, ImGuiInputTextFlags flags = 0);

    // obsIniKey：产品行的日期源＝该 ini 键指向的观测文件；nullptr ⇒ 本行不出"获取"按钮
    void fileRow(const char *label, std::string *buf, const char *iniKey,
                 const GuiFileFilter *filters, int nFilters,
                 const std::function<void(const std::string &)> &onSelect = {},
                 const char *obsIniKey = nullptr);

    void renderCutoffAndSystems(double *cutoffDeg, std::set<char> *sysMask,
                                const std::function<void()> &extraSameLine = {});

    inline constexpr GuiFileFilter fAll = {L"所有文件 (*.*)", L"*.*"};
    inline constexpr GuiFileFilter fOem7 = {L"NovAtel Oem7日志 (*.ASC;*.DAT;*.dat;*.log)", L"*.ASC;*.DAT;*.dat;*.log"};

    inline constexpr GuiFileFilter fClk[] = {
        {L"精密钟差 CLK (*.CLK;*.clk)", L"*.CLK;*.clk"},
        fAll
    };
    inline constexpr GuiFileFilter fOsb[] = {
        {L"码偏差 OSB/BIA (*.BIA;*.bia;*.OSB)", L"*.BIA;*.bia;*.OSB"},
        fAll
    };
    inline constexpr GuiFileFilter fAtx[] = {
        {L"天线文件 ATX (*.atx;*.ATX)", L"*.atx;*.ATX"},
        fAll
    };
    inline constexpr GuiFileFilter fSp3[] = {
        {L"精密轨道 SP3 (*.SP3;*.sp3)", L"*.SP3;*.sp3"},
        fAll
    };
    inline constexpr GuiFileFilter fRnx[] = {
        {L"广播星历 (*.??N;*.??G;*.??C;*.nav;*.rnx)", L"*.??N;*.??G;*.??C;*.nav;*.rnx"},
        fAll
    };
    inline constexpr GuiFileFilter fObs[] = {
        {L"RINEX 观测 (*.??O;*.rnx;*.obs)", L"*.??O;*.rnx;*.obs"},
        fOem7,
        fAll
    };
    inline constexpr GuiFileFilter fHdr[] = {
        {L"杠杆臂 (*.HDR;*.hdr)", L"*.HDR;*.hdr"},
        fAll
    };
    inline constexpr GuiFileFilter fCsv[] = {
        {L"CSV (*.csv)", L"*.csv"},
        fAll
    };
    inline constexpr GuiFileFilter fErp[] = {
        {L"地球自转参数 ERP (*.ERP;*.erp)", L"*.ERP;*.erp"},
        fAll
    };
    inline constexpr GuiFileFilter fTro[] = {
        {L"对流层 TRO (*.TRO;*.tro)", L"*.TRO;*.tro"},
        fAll
    };
    inline constexpr GuiFileFilter fIonex[] = {
        {L"电离层 IONEX (*.ION;*.ion;*.IONEX;*.ionex)", L"*.ION;*.ion;*.IONEX;*.ionex"},
        fAll
    };
    inline constexpr GuiFileFilter fIns[] = {
        {L"ASCII 文本 (*.ASC;*.asc)", L"*.ASC;*.asc"},
        fOem7,
        fAll
    };

    struct RobustResRange {
        double lo = -8.0, hi = 8.0;
    };

    RobustResRange computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes);


    // 路径取文件名（去掉目录部分）
    std::string baseName(const std::string &p);

    // 数值格式化 fmt3/fmt4 是全局函数（src/util/StringUtils.h），不在此命名空间声明；
    // 调用方直接用 fmt4/fmt3 即可（GuiHelpers.h 已包含 StringUtils.h）。

    // 配置面板的通用开头/收尾。配置现在是"标签页里的一段内容"，不再是独立的浮动窗口——
    // 那样同一个任务就有两套形态（Config=浮窗、其余=标签页），浮窗只能靠抢 z 序/焦点去顶，
    // 反而盖住工具窗。改为整块可滚动子区承载（PPP 有 9 行文件选择，可能超过一屏）。
    // EndChild 必须始终调用（ImGui 1.90+ 的约定），故用 endConfigBody() 收尾。
    void beginConfigBody(const std::string &idBase, const GnssTask::SolverTaskBase &task);

    void endConfigBody();

    // 「取消」按钮
    template<class Task>
    void renderCancelButton(Task *task) {
        if (ImGui::Button("取消", ImVec2(80, 40))) {
            LOG_WARN << "[任务] 用户取消";
            // 必须置 stop：各 Processor 的读取/解算线程都靠它退出循环。
            // 原先只置 state/hasError，线程仍会跑完整个文件，取消形同虚设。
            // 两套 stop 都要置：SolverTaskBase::stop 是状态机/取消信号入口，
            // SolveTask::stop（经 solveResult() 取到的 core）才是 SPP 的读取/解算线程与
            // 实时线程实际在 watch 的标志——Application 清理、GuiSolverPanel「停止连接」改的都是它。
            // 只置其一，另一处的循环就收不到停止信号。
            task->stop = true;
            if (auto c = task->solveResult()) c->stop = true;
            task->state = Task::State::Done;
            task->setError("用户取消");
        }
    }

    // CSV 导出：默认文件名 → 保存对话框。复用 StringUtils::ansiToWide（统一 SPP 原 ansiToWide 与 PPP/LEO 原 MultiByteToWideChar 两种写法）。
    bool saveCSVDialog(const std::string &defaultName, std::wstring &outPath);

    // 实时流配置面板：SPP(1 路) 与 RTK(2 路) 共用同一份渲染代码，消除「俩人写的」观感。
    // endpoints：指向 task->core->rtEndpoints（统一存储）；labels：每路的中文名（{"基准站","流动站"} 或 {"流动站"}），
    //            长度须等于 endpoints->size()，空串=不显示分组标题。
    // cutoffDeg/systems：与文件模式共用的截止角/星座筛选。
    // onStart：点击「开始解算」时的回调，由各 Processor 自行构造 ConnectionConfig + 起 worker 线程
    //          （SPP=单路 Solvers::RealtimeProcessor::SolveRealtimeThread；RTK=双路 Solvers::RtkProcessor::SolveRealtimeThread），
    //          差异收敛到这一处，面板本身不再分叉。
    // 取消按钮统一走 renderCancelButton（双 stop 标志 + 日志 + 状态机收尾），SPP 原先手写的「只置 stop 不记日志」被取代。
    template<class Task>
    void renderRealtimeConfig(Task *task,
                              std::vector<GnssTask::RealtimeEndpoint> *endpoints,
                              const std::vector<std::string> &labels,
                              double *cutoffDeg, std::set<char> *systems,
                              const std::function<void()> &onStart) {
        const size_t n = endpoints ? endpoints->size() : 0;
        ImGui::Text("连接到 Oem7 实时流 (socket)：");
        ImGui::PushItemWidth(200);
        for (size_t i = 0; i < n && i < labels.size(); ++i) {
            if (!labels[i].empty()) ImGui::Text("%s：", labels[i].c_str());
            Solvers::Helpers::inputTextStd(("IP 地址##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].ip);
            Solvers::Helpers::inputTextStd(("端口##rt_ep_" + std::to_string(i)).c_str(), &(*endpoints)[i].port);
        }
        ImGui::PopItemWidth();
        ImGui::Separator();

        Solvers::Helpers::renderCutoffAndSystems(cutoffDeg, systems);

        ImGui::Separator();
        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            if (onStart) onStart();
        }
        ImGui::SameLine();
        Solvers::Helpers::renderCancelButton(task);
    }

    // 读取历元数量并钳制选中历元
    template<class Task>
    std::pair<int, int> readEpochView(const std::shared_ptr<Task> &task) {
        std::lock_guard lock(task->mutex);
        const int epochCount = static_cast<int>(task->epochs.size());
        if (task->selectedEpoch >= epochCount) task->selectedEpoch = epochCount - 1;
        if (task->selectedEpoch < 0 && epochCount > 0) task->selectedEpoch = 0;
        return {epochCount, task->selectedEpoch};
    }
} // namespace Solvers::Helpers
