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

namespace GuiPppProcessor {
    using GnssTask::EpochData;
    using GnssTask::SolveTask;
    using GnssTask::SolverMode;

    static void ReaderThread(const std::shared_ptr<PppTask> &task) {
        try {
            const auto &path = task->core->filePath;
            LOG_INFO << "PPP 读取文件: " << path;
            if (const int ok = GnssTask::ReadRinexObsToQueue(task->core, task->core->stop); ok < 0) {
                task->core->loading = false;
                return;
            }
            task->core->hasNav = true; // PPP 自载星历，无外部 nav 依赖
            bool hasQcInput = false;
            {
                std::lock_guard lk(task->core->qcInputMutex);
                hasQcInput = !task->core->qcInput.empty();
            }
            if (hasQcInput)
                GuiSolverPanel::LaunchQC(task->core); // 读完后触发质量分析
            else
                task->core->qcReady = true; // 无 QC 输入时直接标记完成
        } catch (const std::exception &e) {
            task->core->setError(e.what());
        }
        task->core->loading = false;
    }

    // 7 个产品文件相互独立，每个 load 只写 PPPStatic 自己持有的独立 reader 成员
    // (mSp3/mClk/mAntx/...)，load 阶段零交叉读写 => 可并行加载，把"Σ各文件"压成"max单文件"，
    // 端到端加载从 ~1.6s 降到 ~0.7s。flag 在 join 后由主线程统一设，避免多线程写同一 bool。
    // solver 以 void* 传入（钩子契约见 SolverTaskBase）：static 与 kinematic 的 load 集合
    // 完全一致，只是所属类不同，故按 task.kinematic 分派。launch 用模板化 lambda
    // 而非 std::function，避免类型擦除的间接调用。
    // 解算参数不在此下发：最优默认值已内置于 PPPStatic（见其构造），GUI/CLI 共享同一套。
    template <typename PppSolver>
    static void LoadProductsInto(PppTask &task, PppSolver &ppp, std::vector<std::future<bool>> &jobs) {
        auto launch = [&](const char *name, const std::string &path, auto fn) {
            if (path.empty()) return;
            jobs.emplace_back(std::async(std::launch::async, [name, &path, fn]() {
                const bool ok = fn();
                LOG_INFO << "[PPP] 读取" << name << ": " << path << (ok ? "" : "[失败]");
                return ok;
            }));
        };
        launch("精密轨道 SP3", task.sp3PathBuf, [&] { return ppp.loadSp3(task.sp3PathBuf); });
        launch("精密钟差 CLK", task.clkPathBuf, [&] { return ppp.loadClk(task.clkPathBuf, task.core->enabledSystems); });
        launch("广播星历 BRDC", task.brdcPathBuf, [&] { return ppp.loadBrdc(task.brdcPathBuf); });
        launch("天线文件 ATX", task.atxPathBuf, [&] { return ppp.loadAtx(task.atxPathBuf); });
        launch("码偏差 OSB", task.osbPathBuf, [&] { return ppp.loadOsb(task.osbPathBuf); });
        launch("地球自转 ERP", task.erpPathBuf, [&] { return ppp.loadErp(task.erpPathBuf); });
        launch("对流层 TRO", task.troPathBuf, [&] { return ppp.loadTro(task.troPathBuf); });
        launch("电离层 IONEX", task.ionexPathBuf, [&] { return ppp.loadIonex(task.ionexPathBuf); });
    }

    // ---- PppTask 的解算钩子（骨架见 RunSingleStationSolve）----

    bool PppTask::initSolver(void *solver) {
        std::vector<std::future<bool>> jobs;
        if (kinematic) LoadProductsInto(*this, *static_cast<PPPKinematic *>(solver), jobs);
        else            LoadProductsInto(*this, *static_cast<PPPStatic *>(solver), jobs);
        for (auto &j : jobs) j.get();
        // 极潮/对流层ZTD/高阶电离层：产品加载即应用（文件为准，无独立开关）
        return true;
    }

    bool PppTask::afterSolveEpoch(void *, GnssTask::EpochData *data, GnssTask::ObsItem *, int) {
        if (!data->solved) return false;
        std::lock_guard lk(core->mutex);
        if (!core->initializedRefECEF) {
            core->refECEF = data->sppResult.xyz;
            core->initializedRefECEF = true;
        }
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
        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();
        task->state = PppTask::State::Done;
        task->done = true;
        task->loading = false;
        GnssTask::SyncTaskStatus(task, task->core);
    }

    void PppTask::renderConfigPanel() {
        GuiHelpers::beginConfigBody("PPP 处理配置###ppp_cfg_", *this);


        GuiHelpers::fileRow("观测文件", &this->obsPathBuf, "ppp_obs", GuiHelpers::fObs, 3, {}, "ppp_obs");
        GuiHelpers::fileRow("广播星历 BRDC", &this->brdcPathBuf, "ppp_brdc", GuiHelpers::fRnx, 2, {}, "ppp_obs");
        GuiHelpers::fileRow("精密轨道 SP3", &this->sp3PathBuf, "ppp_sp3", GuiHelpers::fSp3, 2, {}, "ppp_obs");
        GuiHelpers::fileRow("精密钟差 CLK", &this->clkPathBuf, "ppp_clk", GuiHelpers::fClk, 2, {}, "ppp_obs");
        GuiHelpers::fileRow("天线文件 ATX", &this->atxPathBuf, "ppp_atx", GuiHelpers::fAtx, 2, {}, "ppp_obs");
        GuiHelpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "ppp_osb", GuiHelpers::fOsb, 2, {}, "ppp_obs");
        GuiHelpers::fileRow("地球自转 ERP", &this->erpPathBuf, "ppp_erp", GuiHelpers::fErp, 2, {}, "ppp_obs");
        GuiHelpers::fileRow("对流层 TRO", &this->troPathBuf, "ppp_tro", GuiHelpers::fTro, 2, {}, "ppp_obs");
        GuiHelpers::fileRow("电离层 IONEX", &this->ionexPathBuf, "ppp_ionex", GuiHelpers::fIonex, 2, {}, "ppp_obs");
        ImGui::Separator();
        GuiHelpers::renderCutoffAndSystems(&this->core->cutoffDeg, &this->core->enabledSystems);
        ImGui::Separator();
        ImGui::Checkbox("动态解算 (Kinematic)", &this->kinematic);
        ImGui::Separator();

        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            if (this->obsPathBuf.empty()) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
            } else {
                // 记忆上次选择（项目外 ini）
                const auto &cfg = GnssCore::PathMemory::instance();
                cfg.set("ppp_obs", this->obsPathBuf);
                cfg.set("ppp_sp3", this->sp3PathBuf);
                cfg.set("ppp_clk", this->clkPathBuf);
                cfg.set("ppp_brdc", this->brdcPathBuf);
                cfg.set("ppp_atx", this->atxPathBuf);
                cfg.set("ppp_osb", this->osbPathBuf);
                cfg.set("ppp_erp", this->erpPathBuf);
                cfg.set("ppp_tro", this->troPathBuf);
                cfg.set("ppp_ionex", this->ionexPathBuf);
                cfg.set("ppp_kinematic", this->kinematic ? "1" : "0");
                this->stop = false;
                this->core->stop = false;

                this->core->filePath = this->obsPathBuf;
                this->core->fileName = GuiHelpers::baseName(this->obsPathBuf);
                this->core->obsPathBuf = this->obsPathBuf;
                this->core->isRinex = true;
                LOG_INFO << "[PPP] 开始解算: 观测=" << this->obsPathBuf
                         << ", 动态=" << (this->kinematic ? "是" : "否");

                this->state = State::Running;
                this->loading = true;
                this->core->loading = true;
                this->worker = std::thread(SolveThread, std::static_pointer_cast<PppTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        GuiHelpers::renderCancelButton(this);

        GuiHelpers::endConfigBody();
    }
}
