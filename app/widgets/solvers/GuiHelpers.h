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
#include <memory>
#include <mutex>

#include <utility>
#include <future>
#include "imgui.h"
#include "ui/Gui.h"
#include "ObsFormat.h"
#include "GnssStruct.h"
#include "StringUtils.h"
#include "Log.h"

namespace GnssTask {
    struct SolverTaskBase;
    struct RealtimeEndpoint;
}

namespace Solvers::Helpers {
    bool inputTextStd(const char *label, std::string *str, ImGuiInputTextFlags flags = 0);

    void fileRow(const char *label, std::string *buf, const char *iniKey,
                 const GuiFileFilter *filters, int nFilters,
                 const std::function<void(const std::string &)> &onSelect = {},
                 const char *obsIniKey = nullptr,
                 std::string *autoNavBuf = nullptr);

    void renderCutoffAndSystems(double *cutoffDeg, std::set<char> *sysMask,
                                double *sampleInterval = nullptr,
                                const std::function<void()> &extraSameLine = {});

    inline constexpr GuiFileFilter fAll = {L"所有文件 (*.*)", L"*.*"};
    inline constexpr GuiFileFilter fOem7 = {L"NovAtel Oem7日志 (*.DAT;*.dat;*.log)", L"*.DAT;*.dat;*.log"};

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
    inline constexpr GuiFileFilter fPng[] = {
        {L"PNG 文件 (*.png)", L"*.png"},
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

    RobustResRange computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes,
                                         double minHalf = 1.0);



    std::string baseName(const std::string &p);

    void beginConfigBody(const std::string &idBase, const GnssTask::SolverTaskBase &task);

    void endConfigBody();


    template<class Task>
    void renderCancelButton(Task *task) {
        if (ImGui::Button("取消", ImVec2(80, 40))) {
            LOG_WARN << "[任务] 用户取消";
            task->stop = true;
            if (auto c = task->solveResult()) c->stop = true;
            task->state = Task::State::Done;
            task->setError("用户取消");
        }
    }

    template <class Task>
    bool beginFileSolve(Task *task, const std::string obsPath) {
        const auto fmt = obsPath.empty()
                             ? GnssTask::ObsFormat::Unknown
                             : GnssTask::classifyObsPath(obsPath);
        if (obsPath.empty()) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
            return false;
        }
        if (fmt == GnssTask::ObsFormat::Unknown) {
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "无法识别观测文件格式");
            return false;
        }
        if (task->worker.joinable()) task->worker.join();
        task->stop = false;
        auto core = task->solveResult();
        core->stop = false;
        core->initializedRefECEF = false;
        core->filePath = obsPath;
        core->fileName = baseName(obsPath);
        core->obsPathBuf = obsPath;
        core->isRinex = fmt == GnssTask::ObsFormat::Rinex;
        task->state = Task::State::Running;
        task->loading = true;
        core->loading = true;
        return true;
    }

    bool saveFileDialog(const std::string &defaultName, std::wstring &outPath,
                        const GuiFileFilter *filters, int nFilters, const wchar_t *defExt);

    bool saveCSVDialog(const std::string &defaultName, std::wstring &outPath);

    void renderRealtimeConfig(GnssTask::SolverTaskBase *task,
                              std::vector<GnssTask::RealtimeEndpoint> *endpoints,
                              const std::vector<std::string> &labels,
                              double *cutoffDeg, std::set<char> *systems,
                              double *sampleInterval,
                              const std::function<void()> &onStart,
                              const std::function<void()> &extraContent = {});

    template<class Task>
    std::pair<int, int> readEpochView(const std::shared_ptr<Task> &task) {
        std::lock_guard lock(task->mutex);
        const int epochCount = static_cast<int>(task->epochs.size());
        if (task->selectedEpoch >= epochCount) task->selectedEpoch = epochCount - 1;
        if (task->selectedEpoch < 0 && epochCount > 0) task->selectedEpoch = 0;
        return {epochCount, task->selectedEpoch};
    }

    template <class Fn>
    void launchProductLoad(std::vector<std::future<bool>> &jobs, const char *tag,
                           const char *name, const std::string &path, Fn fn) {
        if (path.empty()) return;
        jobs.emplace_back(std::async(std::launch::async, [tag, name, &path, fn] {
            const bool ok = fn();
            LOG_INFO << "[" << tag << "] 读取" << name << ": " << path << (ok ? "" : "[失败]");
            return ok;
        }));
    }

    template <class Obj, class MemObj, class Ret>
    void launchProductLoad(std::vector<std::future<bool>> &jobs, const char *tag,
                           const char *name, const std::string &path,
                           Obj &obj, Ret (MemObj::*load)(const std::string &)) {
        launchProductLoad(jobs, tag, name, path, [&obj, load, &path] { return (obj.*load)(path); });
    }

    template <class Obj, class MemObj, class Ret, class Arg>
    void launchProductLoad(std::vector<std::future<bool>> &jobs, const char *tag,
                           const char *name, const std::string &path,
                           Obj &obj, Ret (MemObj::*load)(const std::string &, const Arg &), const Arg &arg) {
        launchProductLoad(jobs, tag, name, path, [&obj, load, &arg, &path] { return (obj.*load)(path, arg); });
    }
}
