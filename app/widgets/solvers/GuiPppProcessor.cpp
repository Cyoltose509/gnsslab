#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "GuiPppProcessor.h"
#include "GuiHelpers.h"
#include "core/AppConfig.h"
#include "imgui.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "Log.h"

#include <filesystem>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
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

    static void SolverThread(const std::shared_ptr<PppTask> &task) {
        auto runWith = [&]([[maybe_unused]] auto &solver) {
                            using SolverT = std::decay_t<decltype(solver)>;
                            GnssTask::RunSingleStationSolve<SolverT>(task->core,
                                                     [&](SolverT &ppp) {
                                                         // 超快/最终 SP3 自带钟差列，此时 CLK 文件可缺省。
                                                         if (!task->sp3PathBuf.empty()) {
                                                             const bool ok = ppp.loadSp3(task->sp3PathBuf);
                                                             LOG_INFO << "PPP 读取精密轨道 SP3: " << task->sp3PathBuf << (ok ? "" : "[失败]");
                                                         }
                                                         if (!task->clkPathBuf.empty()) {
                                                             const bool ok = ppp.loadClk(task->clkPathBuf);
                                                             LOG_INFO << "PPP 读取精密钟差 CLK: " << task->clkPathBuf << (ok ? "" : "[失败]");
                                                         }
                                                         if (!task->brdcPathBuf.empty()) {
                                                             const bool ok = ppp.loadBrdc(task->brdcPathBuf);
                                                             LOG_INFO << "PPP 读取广播星历 BRDC: " << task->brdcPathBuf << (ok ? "" : "[失败]");
                                                         }
                                                         if (!task->atxPathBuf.empty()) {
                                                             const bool ok = ppp.loadAtx(task->atxPathBuf);
                                                             LOG_INFO << "PPP 读取天线文件 ATX: " << task->atxPathBuf << (ok ? "" : "[失败]");
                                                         }
                                                         if (!task->osbPathBuf.empty()) {
                                                             const bool ok = ppp.loadOsb(task->osbPathBuf);
                                                             LOG_INFO << "PPP 读取码偏差 OSB: " << task->osbPathBuf << (ok ? "" : "[失败]");
                                                         }
                                                         if (!task->erpPathBuf.empty()) {
                                                             const bool ok = ppp.loadErp(task->erpPathBuf);
                                                             LOG_INFO << "PPP 读取地球自转参数 ERP: " << task->erpPathBuf << (ok ? "" : "[失败]");
                                                             ppp.mApplyPoleTide = true;
                                                         }
                                                         if (!task->troPathBuf.empty()) {
                                                             const bool ok = ppp.loadTro(task->troPathBuf);
                                                             LOG_INFO << "PPP 读取对流层产品 TRO: " << task->troPathBuf << (ok ? "" : "[失败]");
                                                             ppp.mUseTroZtd = true;
                                                         }
                                                         if (!task->ionexPathBuf.empty()) {
                                                             const bool ok = ppp.loadIonex(task->ionexPathBuf);
                                                             LOG_INFO << "PPP 读取电离层 IONEX: " << task->ionexPathBuf << (ok ? "" : "[失败]");
                                                             ppp.mApplyHigherOrderIono = true;
                                                         }
                                                         return true;
                                                     },
                                                     [&](SolverT &ppp, GnssTask::ObsItem &oi, EpochData &data, int) {
                                                         if (!ppp.processEpoch(oi.obs) || ppp.result.numSats <= 0) return false;
                                                         data.getFromSPP(ppp);
                                                         if (!task->core->initializedRefECEF) {
                                                             std::lock_guard lk(task->core->mutex);
                                                             task->core->refECEF = data.sppResult.xyz;
                                                             task->core->initializedRefECEF = true;
                                                         }
                                                         return true;
                                                     });
        };
        if (task->kinematic) {
            PPPKinematic k;
            runWith(k);
        } else {
            PPPStatic s;
            runWith(s);
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
        GuiHelpers::beginConfigWindow("PPP 处理配置###ppp_cfg_", *this);


        GuiHelpers::fileRow("观测文件", &this->obsPathBuf, "ppp_obs", GuiHelpers::fObs, 3, {});
        GuiHelpers::fileRow("广播星历 BRDC", &this->brdcPathBuf, "ppp_brdc", GuiHelpers::fRnx, 2, {});
        GuiHelpers::fileRow("精密轨道 SP3", &this->sp3PathBuf, "ppp_sp3", GuiHelpers::fSp3, 2, {});
        GuiHelpers::fileRow("精密钟差 CLK", &this->clkPathBuf, "ppp_clk", GuiHelpers::fClk, 2, {});
        GuiHelpers::fileRow("天线文件 ATX", &this->atxPathBuf, "ppp_atx", GuiHelpers::fAtx, 2, {});
        GuiHelpers::fileRow("码偏差 OSB/BIA", &this->osbPathBuf, "ppp_osb", GuiHelpers::fOsb, 2, {});
        GuiHelpers::fileRow("地球自转 ERP", &this->erpPathBuf, "ppp_erp", GuiHelpers::fErp, 2, {});
        GuiHelpers::fileRow("对流层 TRO", &this->troPathBuf, "ppp_tro", GuiHelpers::fTro, 2, {});
        GuiHelpers::fileRow("电离层 IONEX", &this->ionexPathBuf, "ppp_ionex", GuiHelpers::fIonex, 2, {});
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
                const auto &cfg = AppConfig::instance();
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

                this->state = State::Running;
                this->loading = true;
                this->core->loading = true;
                this->worker = std::thread(SolveThread, std::static_pointer_cast<PppTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        GuiHelpers::renderCancelButton(this);

        ImGui::End();
    }
}
