#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "GuiPppProcessor.h"
#include "GuiHelpers.h"
#include "core/PathMemory.h"
#include "imgui.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "Log.h"

#include <filesystem>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <future>
#include "GuiSolverPanel.h"
#include "ObsFormat.h"      // classifyObsPath：RINEX/OEM7 三值判别（Unknown 显式报错）

namespace Solvers::PppProcessor {
    using GnssTask::EpochData;
    using GnssTask::SolveTask;
    using GnssTask::SolverMode;

    template <typename PppSolver>
    static void LoadProductsInto(PppTask &task, PppSolver &ppp, std::vector<std::future<bool>> &jobs) {
        Helpers::launchProductLoad(jobs, "PPP", "精密轨道 SP3", task.sp3PathBuf, ppp, &PppSolver::loadSp3);
        Helpers::launchProductLoad(jobs, "PPP", "精密钟差 CLK", task.clkPathBuf, ppp, &PppSolver::loadClk, task.core->enabledSystems);
        Helpers::launchProductLoad(jobs, "PPP", "广播星历 BRDC", task.brdcPathBuf, ppp, &PppSolver::loadBrdc);
        Helpers::launchProductLoad(jobs, "PPP", "天线文件 ATX", task.atxPathBuf, ppp, &PppSolver::loadAtx);
        Helpers::launchProductLoad(jobs, "PPP", "码偏差 OSB", task.osbPathBuf, ppp, &PppSolver::loadOsb);
        Helpers::launchProductLoad(jobs, "PPP", "地球自转 ERP", task.erpPathBuf, ppp, &PppSolver::loadErp);
        Helpers::launchProductLoad(jobs, "PPP", "对流层 TRO", task.troPathBuf, ppp, &PppSolver::loadTro);
        Helpers::launchProductLoad(jobs, "PPP", "电离层 IONEX", task.ionexPathBuf, ppp, &PppSolver::loadIonex);
    }


    bool PppTask::initSolver(void *solver) {
        std::vector<std::future<bool>> jobs;
        if (kinematic) LoadProductsInto(*this, *static_cast<PPPKinematic *>(solver), jobs);
        else            LoadProductsInto(*this, *static_cast<PPPStatic *>(solver), jobs);
        for (auto &j : jobs) j.get();
        // 极潮/对流层ZTD/高阶电离层：产品加载即应用（文件为准，无独立开关）
        return true;
    }

    static void SolverThread(const std::shared_ptr<PppTask> &task) {
        if (task->kinematic) {
            PPPKinematic solver;
            GnssTask::RunSingleStationSolve<PPPKinematic>(task->core, solver, task);
        } else {
            PPPStatic solver;
            GnssTask::RunSingleStationSolve<PPPStatic>(task->core, solver, task);
        }
    }


    void SolveThread(const std::shared_ptr<PppTask> &task) {
        GnssTask::RunDualThreadJob(task, "PPP", SolverThread);
    }

    void PppTask::renderConfigPanel() {
        Helpers::beginConfigBody("PPP 处理配置###ppp_cfg_", *this);


        Helpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &this->obsPathBuf, "ppp_obs", Helpers::fObs, 3,
                            {}, "ppp_obs", &this->brdcPathBuf);
        Helpers::fileRow("广播星历 BRDC", &this->brdcPathBuf, "ppp_brdc", Helpers::fRnx, 2, {}, "ppp_obs");
        Helpers::fileRow("精密轨道 SP3", &this->sp3PathBuf, "ppp_sp3", Helpers::fSp3, 2, {}, "ppp_obs");
        Helpers::fileRow("精密钟差 CLK", &this->clkPathBuf, "ppp_clk", Helpers::fClk, 2, {}, "ppp_obs");
        Helpers::fileRow("天线文件 ATX", &this->atxPathBuf, "ppp_atx", Helpers::fAtx, 2, {}, "ppp_obs");
        Helpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "ppp_osb", Helpers::fOsb, 2, {}, "ppp_obs");
        Helpers::fileRow("地球自转 ERP", &this->erpPathBuf, "ppp_erp", Helpers::fErp, 2, {}, "ppp_obs");
        Helpers::fileRow("对流层 TRO", &this->troPathBuf, "ppp_tro", Helpers::fTro, 2, {}, "ppp_obs");
        Helpers::fileRow("电离层 IONEX", &this->ionexPathBuf, "ppp_ionex", Helpers::fIonex, 2, {}, "ppp_obs");
        ImGui::Separator();
        Helpers::renderCutoffAndSystems(&this->core->cutoffDeg, &this->core->enabledSystems, &this->core->sampleInterval);
        ImGui::Separator();
        ImGui::Checkbox("动态解算 (Kinematic)", &this->kinematic);
        ImGui::Separator();

        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            // 校验/停旧任务/填 core/置运行态走公共 helper（与 SPP/Leo 同一份）；
            // ini 持久化键各家不同，留在调用方。校验不过时红字已由 helper 绘出。
            if (Helpers::beginFileSolve(this, this->obsPathBuf)) {
                const std::string &path = this->obsPathBuf;
                // 记忆上次选择（项目外 ini）
                const auto &cfg = AppConfig::PathMemory::instance();
                cfg.set("ppp_obs", path);
                cfg.set("ppp_sp3", this->sp3PathBuf);
                cfg.set("ppp_clk", this->clkPathBuf);
                cfg.set("ppp_brdc", this->brdcPathBuf);
                cfg.set("ppp_atx", this->atxPathBuf);
                cfg.set("ppp_osb", this->osbPathBuf);
                cfg.set("ppp_erp", this->erpPathBuf);
                cfg.set("ppp_tro", this->troPathBuf);
                cfg.set("ppp_ionex", this->ionexPathBuf);
                cfg.set("ppp_kinematic", this->kinematic ? "1" : "0");
                cfg.setNum("ppp_cutoff", this->core->cutoffDeg);
                cfg.setNum("ppp_interval", this->core->sampleInterval);
                cfg.set("ppp_systems", GnssTask::sysSetToString(this->core->enabledSystems));
                LOG_INFO << "[PPP] 开始解算: 观测=" << path
                         << ", 格式=" << (this->core->isRinex ? "RINEX" : "OEM7")
                         << ", 动态=" << (this->kinematic ? "是" : "否");
                this->worker = std::thread(SolveThread, std::static_pointer_cast<PppTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        Helpers::renderCancelButton(this);

        Helpers::endConfigBody();
    }
}
