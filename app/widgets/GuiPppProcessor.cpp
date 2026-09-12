#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include "GuiPppProcessor.h"
#include "GuiHelpers.h"   // fileRow / renderCutoffAndSystems / InputTextStd / beginConfigWindow / renderCancelButton
#include "core/AppConfig.h"
#include "ui/Gui.h"
#include "imgui.h"
#include "PPP.h"          // 手写 PPP 解算器
#include "RinexObsReader.h"
#include "QualityControl.h"   // QC::makeQCObsEpoch / QualityReport
#include "Log.h"

#include <fstream>
#include <filesystem>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>
#include <string>
#include <vector>

namespace GuiPppProcessor {
    using GuiFileProcessor::SppEpochData;
    using GuiFileProcessor::SppTask;

    static void ReaderThread(const std::shared_ptr<PppTask> &task) {
        try {
            const auto &path = task->core->filePath;
            LOG_INFO << "PPP 读取文件: " << path;
            task->core->qcInput.clear();

            {
                std::lock_guard lk(task->core->queueMutex);
                task->core->navReady = true;   // PPP 自载星历，无外部 nav 依赖
            }
            task->core->queueCv.notify_all();

            RinexObsReader obsReader;
            std::fstream obsFile(path.c_str(), std::ios::in);
            if (!obsFile) {
                task->core->hasError = true;
                task->core->errorMsg = "无法打开 RINEX";
                task->core->readDone = true;
                task->core->queueCv.notify_all();
                task->core->loading = false;
                return;
            }
            obsReader.pFileStream = &obsFile;
            int safety = 0, ok = 0;
            while (++safety < 100000) {
                if (task->stop) break;
                try {
                    ObsData o = obsReader.parseRinexObs();
                    task->core->qcInput.push_back(QC::makeQCObsEpoch(o, o.weekSecond.sow));
                    {
                        std::lock_guard lk(task->core->queueMutex);
                        task->core->obsQueue.push({std::move(o), nullptr});
                    }
                    task->core->queueCv.notify_one();
                    ok++;
                } catch (const EndOfFile &) { break; }
                  catch (const std::exception &e) {
                    task->core->hasError = true;
                    task->core->errorMsg = "RINEX parse err: " + std::string(e.what());
                    task->core->readDone = true;
                    task->core->queueCv.notify_all();
                    task->core->loading = false;
                    return;
                }
            }
            if (ok == 0) {
                task->core->hasError = true;
                task->core->errorMsg = "RINEX 无历元";
            }
            task->core->totalEpochs = ok;
            task->core->hasNav = true;
            task->core->readDone = true;
            task->core->queueCv.notify_all();
            if (!task->core->qcInput.empty())
                GuiFileProcessor::LaunchQC(task->core);   // 读完后触发质量分析
            else
                task->core->qcReady = true;                // 无 QC 输入时直接标记完成
        } catch (const std::exception &e) {
            task->core->hasError = true;
            task->core->errorMsg = e.what();
        }
        task->core->loading = false;
    }

    static void SolverThread(const std::shared_ptr<PppTask> &task) {
        try {
            {
                std::unique_lock lk(task->core->queueMutex);
                task->core->queueCv.wait(lk, [&] { return task->core->navReady.load() || task->stop.load(); });
            }
            if (task->stop) { task->core->solvingDone = true; return; }

            PPP ppp;
            ppp.setCutoffElevDeg(task->core->cutoffDeg);
            ppp.enabledSystems = task->core->enabledSystems;

            bool sp3ok = false, clkok = false, brdcok = false, atxok = false, osbok = false;
            if (!task->sp3PathBuf.empty() && !task->clkPathBuf.empty()) {
                sp3ok = ppp.loadSp3(task->sp3PathBuf);
                clkok = ppp.loadClk(task->clkPathBuf);
            }
            if (!task->brdcPathBuf.empty()) brdcok = ppp.loadBrdc(task->brdcPathBuf);
            if (!task->atxPathBuf.empty()) atxok = ppp.loadAtx(task->atxPathBuf);
            if (!task->osbPathBuf.empty()) osbok = ppp.loadOsb(task->osbPathBuf);
            LOG_INFO << "[PPP] SP3=" << sp3ok << " CLK=" << clkok << " BRDC=" << brdcok
                     << " ATX=" << atxok << " OSB=" << osbok
                     << " ephTable gps=" << ppp.ephTable.gps.size()
                     << " bds=" << ppp.ephTable.bds.size();

            while (true) {
                SppEpochData item;
                GuiFileProcessor::ObsItem oi;
                {
                    std::unique_lock lk(task->core->queueMutex);
                    task->core->queueCv.wait(lk, [&] {
                        return !task->core->obsQueue.empty() || task->core->readDone.load() || task->stop.load();
                    });
                    if (task->core->obsQueue.empty() && (task->core->readDone.load() || task->stop.load())) break;
                    if (task->core->obsQueue.empty()) continue;
                    oi = std::move(task->core->obsQueue.front());
                    task->core->obsQueue.pop();
                }
                if (task->stop) break;

                bool ok = false;
                try { ok = ppp.process(oi.obs); }
                catch (const std::exception &e) { LOG_ERROR << "[PPP] process 异常: " << e.what(); ok = false; }
                catch (...) { LOG_ERROR << "[PPP] process 未知异常(非 std::exception，疑似访问违例/越界)"; ok = false; }

                {
                    std::lock_guard lk(task->core->mutex);
                    item.getFromObs(oi.obs);
                    if (ok && ppp.result.numSats > 0) {
                        item.getFromSPP(ppp);
                        if (!task->core->initializedRefECEF) {
                            task->core->refECEF = item.sppResult.xyz;
                            task->core->initializedRefECEF = true;
                        }
                    } else {
                        item.solved = false;
                    }
                    const int idx = static_cast<int>(task->core->epochs.size());
                    task->core->epochs.push_back(std::move(item));
                    {
                        std::lock_guard plk(task->core->plotMutex);
                        task->core->plotData.insert(idx, task->core->epochs.back(), task->core->refECEF);
                    }
                    if (task->core->selectedEpoch == -1 || task->core->selectedEpoch == idx - 1)
                        task->core->selectedEpoch = idx;
                    task->core->solvingProgress = idx + 1;
                    task->core->solvedCount = static_cast<int>(task->core->epochs.size());
                }
            }
        } catch (const std::exception &e) {
            task->core->hasError = true;
            task->core->errorMsg = e.what();
        }
        task->core->solvingDone = true;
    }

    void SolveThread(const std::shared_ptr<PppTask> &task) {
        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();
        task->core->state = SppTask::State::Done;
        task->state = PppTask::State::Done;
        task->done = true;
        task->loading = false;
        task->core->done = true;
        task->core->loading = false;
    }

    void RenderConfigPanel(const std::shared_ptr<PppTask> &task) {
        GuiHelpers::beginConfigWindow("PPP 处理配置###ppp_cfg_", task.get());

        if (ImGui::BeginTabBar("##ppp_cfg_tabs")) {
            if (ImGui::BeginTabItem("精密产品")) {
                GuiHelpers::fileRow("观测文件", &task->obsPathBuf, "ppp_obs", GuiHelpers::fObs, 3, {});
                GuiHelpers::fileRow("广播星历 BRDC", &task->brdcPathBuf, "ppp_brdc", GuiHelpers::fRnx, 2, {});
                GuiHelpers::fileRow("精密轨道 SP3", &task->sp3PathBuf, "ppp_sp3", GuiHelpers::fSp3, 2, {});
                GuiHelpers::fileRow("精密钟差 CLK", &task->clkPathBuf, "ppp_clk", GuiHelpers::fClk, 2, {});
                GuiHelpers::fileRow("天线文件 ATX", &task->atxPathBuf, "ppp_atx", GuiHelpers::fAtx, 2, {});
                GuiHelpers::fileRow("码偏差 OSB/BIA", &task->osbPathBuf, "ppp_osb", GuiHelpers::fOsb, 2, {});
                ImGui::Separator();
                GuiHelpers::renderCutoffAndSystems(&task->core->cutoffDeg, &task->core->enabledSystems);
                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (task->obsPathBuf.empty()) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择观测文件");
                    } else {
                        // 记忆上次选择（项目外 ini）
                        const auto &cfg = AppConfig::instance();
                        cfg.set("ppp_obs", task->obsPathBuf);
                        cfg.set("ppp_sp3", task->sp3PathBuf);
                        cfg.set("ppp_clk", task->clkPathBuf);
                        cfg.set("ppp_brdc", task->brdcPathBuf);
                        cfg.set("ppp_atx", task->atxPathBuf);
                        cfg.set("ppp_osb", task->osbPathBuf);

                        task->core->filePath = task->obsPathBuf;
                        task->core->fileName = GuiHelpers::baseName(task->obsPathBuf);
                        task->fileName = task->core->fileName;
                        task->core->obsPathBuf = task->obsPathBuf;
                        task->core->isRinex = true;

                        task->state = PppTask::State::Running;
                        task->core->state = SppTask::State::Running;
                        task->loading = true;
                        task->core->loading = true;
                        task->worker = std::thread(SolveThread, task);
                    }
                }
                ImGui::SameLine();
                GuiHelpers::renderCancelButton(task);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }

    void RenderTask(const std::shared_ptr<PppTask> &task) {
        if (task->state == PppTask::State::Config) {
            RenderConfigPanel(task);
            return;
        }

        task->core->loading.store(task->loading.load());
        task->core->done.store(task->done.load());
        task->core->hasError = task->hasError;
        task->core->errorMsg = task->errorMsg;
        task->core->processorLabel = "PPP";   // 导出文件名 / 标签用
        if (task->fileName.empty()) task->fileName = task->core->fileName;
        task->core->fileName = task->fileName;
        task->core->state = SppTask::State::Running;   // Running/Done 统一走渲染分支
        GuiFileProcessor::RenderTask(task->core, false);
    }
}
