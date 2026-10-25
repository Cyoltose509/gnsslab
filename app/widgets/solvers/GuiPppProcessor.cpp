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

    static void ReaderThread(const std::shared_ptr<PppTask> &task) {
        try {
            const auto &path = task->core->filePath;
            LOG_INFO << "PPP 读取文件: " << path;
            // RINEX / OEM7 分派已下沉 GnssTask::ReadObsToQueue（SPP/LEO 共用同一份）：
            // 返回值 -1 = 打开或解析失败；0 = 无历元；>0 = 入队历元数。
            // OEM7 内含星历，逐历元携带 eph 快照入队，由 beforeSolveEpoch 注入；
            // RINEX 模式星历在 initSolver 内由 SP3/BRDC 自载，无外部 nav 依赖。
            const int ok = GnssTask::ReadObsToQueue(task->core, task->core->stop);
            if (ok < 0) {
                task->core->loading = false;
                return;
            }
            // RINEX 模式星历在 initSolver 内由 SP3/BRDC 自载，无外部 nav 依赖（保持原语义）；
            // OEM7 内含星历，hasNav 已由 ReadOem7ObsToQueue 按入队历元数置位。
            if (task->core->isRinex) task->core->hasNav = true;
            bool hasQcInput = false;
            {
                std::lock_guard lk(task->core->qcInputMutex);
                hasQcInput = !task->core->qcInput.empty();
            }
            if (hasQcInput)
                Solvers::SolverPanel::LaunchQC(task->core); // 读完后触发质量分析
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

    // OEM7 观测文件自带广播星历快照（RINEX 模式 item.eph 为 nullptr，星历来自
    // initSolver 加载的 SP3/BRDC）：逐历元注入 solver.ephTable，与 SPP GUI 同一钩子语义。
    // 快照为空时保留上一帧非空表（toe 不变时各历元本就共享同一 shared_ptr）。
    void PppTask::beforeSolveEpoch(void *solverPtr, GnssTask::ObsItem *itemPtr) {
        if (!core->isRinex && itemPtr && itemPtr->eph) {
            auto &ppp = *static_cast<SPP *>(solverPtr); // PPPStatic/PPPKinematic 均继承 SPP，ephTable 在 SPP
            const EphemerisTable *tbl = itemPtr->eph.get();
            if (tbl->gps.empty() && tbl->bds.empty() && lastGoodEph) tbl = lastGoodEph.get();
            else if (!tbl->gps.empty() || !tbl->bds.empty()) lastGoodEph = itemPtr->eph;
            ppp.ephTable = *tbl;
            core->hasNav = true;
        }
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
        Solvers::Helpers::beginConfigBody("PPP 处理配置###ppp_cfg_", *this);


        Solvers::Helpers::fileRow("观测文件 (RINEX .??O / OEM7 .log)", &this->obsPathBuf, "ppp_obs", Solvers::Helpers::fObs, 3,
                            [&](const std::string &p) {
                                const std::string d = GnssTask::AutoDetectNav(p);
                                if (!d.empty()) this->brdcPathBuf = d;
                            }, "ppp_obs");
        Solvers::Helpers::fileRow("广播星历 BRDC", &this->brdcPathBuf, "ppp_brdc", Solvers::Helpers::fRnx, 2, {}, "ppp_obs");
        Solvers::Helpers::fileRow("精密轨道 SP3", &this->sp3PathBuf, "ppp_sp3", Solvers::Helpers::fSp3, 2, {}, "ppp_obs");
        Solvers::Helpers::fileRow("精密钟差 CLK", &this->clkPathBuf, "ppp_clk", Solvers::Helpers::fClk, 2, {}, "ppp_obs");
        Solvers::Helpers::fileRow("天线文件 ATX", &this->atxPathBuf, "ppp_atx", Solvers::Helpers::fAtx, 2, {}, "ppp_obs");
        Solvers::Helpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "ppp_osb", Solvers::Helpers::fOsb, 2, {}, "ppp_obs");
        Solvers::Helpers::fileRow("地球自转 ERP", &this->erpPathBuf, "ppp_erp", Solvers::Helpers::fErp, 2, {}, "ppp_obs");
        Solvers::Helpers::fileRow("对流层 TRO", &this->troPathBuf, "ppp_tro", Solvers::Helpers::fTro, 2, {}, "ppp_obs");
        Solvers::Helpers::fileRow("电离层 IONEX", &this->ionexPathBuf, "ppp_ionex", Solvers::Helpers::fIonex, 2, {}, "ppp_obs");
        ImGui::Separator();
        Solvers::Helpers::renderCutoffAndSystems(&this->core->cutoffDeg, &this->core->enabledSystems);
        ImGui::Separator();
        ImGui::Checkbox("动态解算 (Kinematic)", &this->kinematic);
        ImGui::Separator();

        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            const std::string path = this->obsPathBuf;
            // 格式判别是显式三值（与 SPP/RTK 同一入口 classifyObsPath）：无法识别时明确报错，
            // 不再默认落进 RINEX 读取器把 .log/.oem7 报成"无法打开 RINEX"误导用户
            const auto fmt = path.empty() ? GnssTask::ObsFormat::Unknown : GnssTask::classifyObsPath(path);
            if (path.empty()) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
            } else if (fmt == GnssTask::ObsFormat::Unknown) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1),
                                   "无法识别观测文件格式（支持 RINEX .??O/.rnx/.obs 与 OEM7 .log/.oem7）");
            } else {
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
                cfg.set("ppp_systems", GnssTask::sysSetToString(this->core->enabledSystems));
                this->stop = false;
                this->core->stop = false;

                this->core->filePath = path;
                this->core->fileName = Solvers::Helpers::baseName(path);
                this->core->obsPathBuf = path;
                this->core->isRinex = (fmt == GnssTask::ObsFormat::Rinex);
                LOG_INFO << "[PPP] 开始解算: 观测=" << path
                         << ", 格式=" << (this->core->isRinex ? "RINEX" : "OEM7")
                         << ", 动态=" << (this->kinematic ? "是" : "否");

                this->state = State::Running;
                this->loading = true;
                this->core->loading = true;
                this->worker = std::thread(SolveThread, std::static_pointer_cast<PppTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        Solvers::Helpers::renderCancelButton(this);

        Solvers::Helpers::endConfigBody();
    }
}
