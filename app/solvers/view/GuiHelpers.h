#pragma once

#include <string>
#include <functional>
#include <set>
#include <map>
#include <vector>
#include <memory>
#include <mutex>

#include <utility>
#include "imgui.h"
#include "ui/Gui.h"
#include "ui/GuiColors.h"
#include "Observer.h"
#include "GnssStruct.h"
#include "StringUtils.h"
#include "Log.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"

namespace Solvers::Helpers {
    bool inputTextStd(const char *label, std::string *str, ImGuiInputTextFlags flags = 0);

    void renderSimpleConfig(double *cutoffDeg, std::set<char> *sysMask,
                            double *sampleInterval = nullptr,
                            const std::function<void()> &extraSameLine = {});

    struct ConstellationName {
        char ch;
        const char *label;
    };

    inline constexpr ConstellationName kConstellations[] = {
        {'G', "GPS"}, {'R', "GLO"}, {'E', "GAL"}, {'C', "BDS"}
    };

    inline const char *systemName(const char s) {
        for (const auto &[ch, label]: kConstellations)
            if (ch == s) return label;
        return "?";
    }

    struct RobustResRange {
        double lo = -8.0, hi = 8.0;
    };

    RobustResRange computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes,
                                         double minHalf = 1.0);


    void beginConfigBody(const std::string &idBase, const Solvers::Task &task);

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

    template<class Task>
    bool beginFileSolve(Task *task, const std::string obsPath) {
        const auto fmt = obsPath.empty()
                             ? Observer::Format::Unknown
                             : Observer::classifyObsPath(obsPath);
        if (obsPath.empty()) {
            ImGui::TextColored(UI::kColFail, "请先选择观测文件");
            return false;
        }
        if (fmt == Observer::Format::Unknown) {
            ImGui::TextColored(UI::kColFail, "无法识别观测文件格式");
            return false;
        }
        if (task->worker.joinable()) task->worker.join();
        task->stop = false;
        auto core = task->solveResult();
        core->stop = false;
        core->initializedRefECEF = false;
        core->filePath = obsPath;
        core->fileName = Str::baseName(obsPath);
        core->obsPathBuf = obsPath;
        core->isRINEX = Observer::isRINEX(fmt);
        task->state = Task::State::Running;
        task->loading = true;
        core->loading = true;
        return true;
    }

    void renderRealtimeConfig(Solvers::Task *task,
                              std::vector<Solvers::RealtimeEndpoint> *endpoints,
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
}
