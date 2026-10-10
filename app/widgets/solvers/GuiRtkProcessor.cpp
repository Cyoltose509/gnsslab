#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include "GuiRtkProcessor.h"
#include "GuiHelpers.h"   // fileRow / renderCutoffAndSystems / inputTextStd / beginConfigBody / renderCancelButton
#include "core/PathMemory.h"
#include "ui/Gui.h"
#include "imgui.h"
#include "RTK.h"               // 手写 RTK 双差非组合求解器（自举基准/流动站线性化初值）
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "OEM7Reader.h"
#include "CoordConvert.h"
#include "Exception.h"         // EndOfFile, SyncException
#include "QualityControl.h"    // QC::makeQCObsEpoch / QualityReport
#include "Log.h"
#include "SolveDriver.h"       // 解算编排共享层：feedRtk（CLI 与 GUI 共用回填实现）

#include <chrono>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <cmath>

using Eigen::Vector3d;

namespace GuiRtkProcessor {
    using GnssTask::EpochData;
    using GnssTask::SolveTask;
    using GnssTask::SolverMode;

    // 前向声明：feedRtkEpoch 定义在 SolveThread 之前，供 SolverThread 内调用点可见
    void feedRtkEpoch(EpochData &data, const RTK &rtk);

    // 是否 OEM7 日志（按扩展名判断；其余一律按 RINEX 观测处理）
    static bool isOem7Path(const std::string &p) {
        const auto dp = p.rfind('.');
        if (dp == std::string::npos) return false;
        const std::string ext = p.substr(dp + 1);
        return ext == "log" || ext == "LOG" || ext == "oem7" || ext == "OEM7";
    }

    // 在首个含有效星历的历元上做 SPP 冷启动（seed=0，地心），估计该站近似 ECEF 坐标。
    // OEM7 课程日志常无 BESTPOS，而前几个历元的星历快照可能仍空 → 跳过空快照。
    // 成功返回 true（|xyz|>1e6），失败返回 false。

    // 每星几何（仰角/方位角/位置）现由 RTK::processEpoch 直接存入 satElevData/satAzimData/
    // satPVTRecTime（与 SPP 同名字段），GUI 经 feedRtkEpoch 读取，不再在此重算。

    // ----------------------------------------------------------
    // 读取线程：基准站 + 流动站两路观测，按历元对齐为 (base,rover,eph) 对
    // ----------------------------------------------------------
    /// 读取线程失败收尾：置错误态、结束读取、唤醒等待中的解算线程。
    /// 原为 ReaderThread 内 8 处逐字重复，仅错误消息不同。
    static void failRead(const std::shared_ptr<RtkTask> &task, const std::string &msg) {
        task->core->setError(msg);
        task->readDone = true;
        task->core->readDone = true;
        task->core->loading = false;
        task->core->queueCv.notify_all();
    }

    static void ReaderThread(const std::shared_ptr<RtkTask> &task) {
        try {
            const bool oem7 = isOem7Path(task->baseObsPathBuf) || isOem7Path(task->roverObsPathBuf);

            // 质量分析分两标签：主页=流动站，附加页=基准站。每页自带 qcInput/qcReport。
            {
                std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                task->core->qcInput.clear();
            }
            task->core->extraQcTabs.clear();
            task->core->qcTabLabel = "质量分析[流动站]";
            task->core->qcLazy = true;   // 两份 QC 推迟到打开对应标签时算，加快首屏加载
            std::vector<QC::QCObsEpoch> baseQcInput;
            auto baseQc = std::make_shared<SolveTask>();
            baseQc->mode = SolverMode::Rtk;
            baseQc->filePath = task->baseObsPathBuf;
            baseQc->fileName = GuiHelpers::baseName(task->baseObsPathBuf);
            baseQc->qcLazy = true;

            if (oem7) {
                // ---- OEM7 流式路径（逐历元读取，避免 readAll 急切加载整文件；双站时间对齐与原 matchEpochPairs 等价）----
                OEM7Reader br, rr;
                if (!br.open(task->baseObsPathBuf) || !rr.open(task->roverObsPathBuf)) {
                    failRead(task, "无法打开 OEM7 日志");
                    return;
                }

                // 基准站/流动站近似坐标由 RTK::processEpoch 内部自举（BESTPOS/头坐标，缺失则 SPP 冷启动）。
                // 读取线程只做双站时间对齐，不再计算任何解算量。

                // 合并读取器：RTK 内部自举线性化初值，读取线程仅做双站时间对齐。
                auto nextBase = [&](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
                    if (!br.getNextEpoch(out)) return false;
                    eph = br.ephSnapshot(); return true;
                };
                auto nextRover = [&](ObsData &out) -> bool {
                    return rr.getNextEpoch(out);
                };

                const double tol = 0.001;   // 与 RTK::matchEpochPairs 默认一致
                int ok = 0;
                ObsData base, rover; std::shared_ptr<EphemerisTable> baseEph;
                if (!nextBase(base, baseEph)) { failRead(task, "OEM7 base 无历元"); return; }
                if (!nextRover(rover))      { failRead(task, "OEM7 rover 无历元"); return; }

                bool roverExhausted = false;
                while (true) {
                    if (task->stop) break;
                    // 推进流动站直到其时间 >= 基准站时间
                    while (!roverExhausted && rover.epoch < base.epoch) {
                        if (!nextRover(rover)) { roverExhausted = true; break; }
                    }
                    if (roverExhausted) break;
                    if (rover.epoch > base.epoch + tol) {
                        // 该基准站历元落在流动站时间窗之外 → 跳过，读下一基准站历元
                        if (!nextBase(base, baseEph)) break;
                        continue;
                    }
                    // 匹配成功，入队解算（近似坐标由 RTK 内部自举）
                    {
                        std::lock_guard lk(task->pairMtx);
                        task->pairQueue.push({base, rover, baseEph});
                    }
                    task->pairCv.notify_one();
                    {
                        std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                        task->core->qcInput.push_back(QC::makeQCObsEpoch(rover, rover.weekSecond.sow));
                    }
                    baseQcInput.push_back(QC::makeQCObsEpoch(base, base.weekSecond.sow));
                    ++ok;
                    if (!nextBase(base, baseEph)) break;
                    if (!nextRover(rover))   { roverExhausted = true; break; }
                }
                task->totalEpochs = ok;
            } else {
                // ---- RINEX 路径 ----
                if (task->navPathBuf.empty()) {
                    failRead(task, "RINEX 模式需要广播星历(导航)文件");
                    return;
                }
                {
                    RinexNavStore nav;
                    nav.loadFile(task->navPathBuf, task->navTable);
                }
                if (task->navTable.gps.empty() && task->navTable.bds.empty()) {
                    failRead(task, "广播星历无法加载");
                    return;
                }
                LOG_INFO << "RTK 读取广播星历: " << task->navPathBuf
                         << " gps=" << task->navTable.gps.size()
                         << " bds=" << task->navTable.bds.size();

                RinexObsReader baseRdr, roverRdr;
                std::fstream baseFile(task->baseObsPathBuf, std::ios::in);
                std::fstream roverFile(task->roverObsPathBuf, std::ios::in);
                if (!baseFile.is_open() || !roverFile.is_open()) {
                    failRead(task, "无法打开 RINEX 观测文件");
                    return;
                }
                baseRdr.pFileStream = &baseFile;
                roverRdr.pFileStream = &roverFile;

                ObsData base;
                try {
                    base = baseRdr.parseRinexObs();
                } catch (...) {
                    failRead(task, "基准站 RINEX 解析失败");
                    return;
                }
                // 基准站近似坐标由 RTK::processEpoch 内部自举（RINEX 头 APPROX 或 SPP 冷启动），
                // 读取线程不再计算。

                // RTK 启动加速：当基准站文件远大于流动站（如超大参考站 + rover 仅取尾部窗口）时，
                // 同步循环原本要逐历元解析几乎整个 base 才能与 rover 对齐（数万历元 → 数秒）。
                // 这里先探测 rover 首历元时间，再把 base 流跳到该历元附近（仅扫历元行，亚秒级）。
                // 只重定位流指针、不改任何已解析历元数据，对齐数据行为与原逐历元推进完全一致。
                {
                    RinexObsReader probeRdr;
                    std::fstream probeFile(task->roverObsPathBuf, std::ios::in);
                    if (probeFile.is_open()) {
                        probeRdr.pFileStream = &probeFile;
                        try {
                            const ObsData rover0 = probeRdr.parseRinexObs();
                            baseRdr.fastForwardToEpoch(rover0.epoch);
                        } catch (const std::exception &) {
                            // 探测失败不致命：退回原始逐历元同步
                        }
                    }
                }

                // 流动站线性化初值由 RTK::processEpoch 内部自举（首历元 antennaPosition 或 SPP）。
                int ok = 0;
                while (true) {
                    if (task->stop) break;
                    ObsData rover;
                    bool gotRover = false;
                    try {
                        rover = roverRdr.parseRinexObs(base.epoch);  // 同步到基准站历元
                        gotRover = true;
                    } catch (const SyncException &) {
                        // 流动站超前基准站 → 推进基准站后重试
                        try { base = baseRdr.parseRinexObs(); }
                        catch (const EndOfFile &) { break; }
                        catch (...) { break; }
                        continue;
                    } catch (const EndOfFile &) {
                        break;
                    } catch (...) {
                        break;
                    }

                    if (gotRover) {
                        {
                            std::lock_guard lk(task->pairMtx);
                            task->pairQueue.push({base, rover, nullptr});
                        }
                        task->pairCv.notify_one();
                        {
                            std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                            task->core->qcInput.push_back(QC::makeQCObsEpoch(rover, rover.weekSecond.sow));
                        }
                        baseQcInput.push_back(QC::makeQCObsEpoch(base, base.weekSecond.sow));
                        ++ok;
                        // 推进基准站
                        try { base = baseRdr.parseRinexObs(); }
                        catch (const EndOfFile &) { break; }
                        catch (...) { break; }
                    }
                }
                task->totalEpochs = ok;
            }

            task->readDone = true;
            task->core->readDone = true;
            task->core->queueCv.notify_all();
            task->pairCv.notify_all();
            // 质量分析（流动站主页 + 基准站附加页）改为「打开对应标签时再算」(懒加载)：
            // 避免初始加载阶段串行跑两份 QC::compute 拖慢首屏；渲染层 QualityControl::render
            // 在 tab 可见且尚未计算时自动触发一次 LaunchQC（SPP/PPP 仍是读取线程 eager 触发，不受影响）。
            {
                std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                if (task->core->qcInput.empty()) task->core->qcReady = true;
            }
            if (!baseQcInput.empty()) baseQc->qcInput = std::move(baseQcInput);
            baseQc->readDone = true;
            baseQc->done = true;
            {
                std::lock_guard lk(task->core->mutex);
                task->core->extraQcTabs.push_back({"质量分析[基准站]", baseQc});
            }
        } catch (const std::exception &e) {
            task->core->setError(std::string("RTK 读取错误: ") + e.what());
            task->readDone = true;
            task->core->readDone = true;
            task->core->queueCv.notify_all();
            task->pairCv.notify_all();
        }
        task->core->loading = false;
    }

    // ----------------------------------------------------------
    // 解算线程：逐对调用 RTK::processEpoch，结果写入 SolveTask 基础设施
    // ----------------------------------------------------------
    static void SolverThread(const std::shared_ptr<RtkTask> &task) {
        try {
            while (true) {
                RtkTask::RtkPair item;
                {
                    std::unique_lock lk(task->pairMtx);
                    task->pairCv.wait(lk, [&] {
                        return !task->pairQueue.empty() || task->readDone.load() || task->stop.load();
                    });
                    if (task->pairQueue.empty() && (task->readDone.load() || task->stop.load())) break;
                    if (task->pairQueue.empty()) continue;
                    item = std::move(task->pairQueue.front());
                    task->pairQueue.pop();
                }
                if (task->stop) break;

                // 基准/流动站线性化初值由 RTK::processEpoch 内部自举（BESTPOS/头坐标或 SPP 冷启动）。
                // task->rtk 仅解算线程访问，读取线程不再写任何解算量，无数据竞争。

                EphemerisTable *eph = item.eph ? item.eph.get() : &task->navTable;
                bool ok = false;
                try { ok = task->rtk.processEpoch(item.base, item.rover, *eph); }
                catch (const std::exception &e) { LOG_ERROR << "[RTK] processEpoch 异常: " << e.what(); ok = false; }
                catch (...) { LOG_ERROR << "[RTK] processEpoch 未知异常"; ok = false; }

                if (!ok) continue;

                const Result &r = task->rtk.result;
                if (r.numSats <= 0) continue;

                EpochData data;
                data.getFromObs(item.rover);
                feedRtkEpoch(data, task->rtk);   // 与 SPP/PPP/LEO 同形：收口 result/几何/排除标记

                // refECEF 取本历元解算后基准站坐标（解算线程内读取，安全）
                const Vector3d baseXyz = task->rtk.mBaseXyz;
                GnssTask::CommitEpoch(task->core, std::move(data), [&](int, EpochData &) {
                    if (!task->core->initializedRefECEF) {
                        task->core->refECEF = XYZ(baseXyz.x(), baseXyz.y(), baseXyz.z());
                        task->core->initializedRefECEF = true;
                    }
                });
            }
        } catch (const std::exception &e) {
            task->core->setError(std::string("RTK 解算错误: ") + e.what());
        }
        task->solvingDone = true;
        task->core->solvingDone = true;
    }

    // 委托给 SolveDriver 的唯一回填实现（CLI 与 GUI 共用，消灭重复）
    void feedRtkEpoch(EpochData &data, const RTK &rtk) {
        SolveDriver::feedRtk(data, rtk);
    }

    static void SolveThread(const std::shared_ptr<RtkTask> &task) {
        const auto tRtk0 = std::chrono::steady_clock::now();
        task->rtkCfg.cutoffElevRad = task->cutoffDeg * PI / 180.0;
        task->rtkCfg.enabledSystems = task->enabledSystems;
        task->rtk.setConfig(task->rtkCfg);

        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();

        size_t rtkEpochs = 0;
        {
            std::lock_guard lk(task->core->mutex);
            rtkEpochs = task->core->epochs.size();
        }
        LOG_INFO << "[RTK] 结束: " << rtkEpochs << " 历元, 耗时 "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - tRtk0).count() << " ms";
        task->state = RtkTask::State::Done;
        task->done = true;
        task->loading = false;
        GnssTask::SyncTaskStatus(task, task->core);
    }

    void RtkTask::renderConfigPanel() {
        GuiHelpers::beginConfigBody("RTK 处理配置###rtk_cfg_", *this);

        // 与 PPP/LEO 完全一致：直接复用 GuiHelpers 预置过滤器（fObs 已含 *.??O 与 *.log，fRnx 含广播星历）
        GuiHelpers::fileRow("基准站观测文件", &this->baseObsPathBuf, "rtk_base", GuiHelpers::fObs, 3, {});
        GuiHelpers::fileRow("流动站观测文件", &this->roverObsPathBuf, "rtk_rover", GuiHelpers::fObs, 3, {});
        GuiHelpers::fileRow("广播星历 BRDC", &this->navPathBuf, "rtk_nav", GuiHelpers::fRnx, 2, {}, "rtk_base");
        ImGui::Separator();
        GuiHelpers::renderCutoffAndSystems(&this->cutoffDeg, &this->enabledSystems);

        if (ImGui::Button("开始解算", ImVec2(200, 40))) {
            if (this->baseObsPathBuf.empty() || this->roverObsPathBuf.empty()) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择基准站与流动站观测文件");
            } else if (!isOem7Path(this->baseObsPathBuf) && !isOem7Path(this->roverObsPathBuf)
                       && this->navPathBuf.empty()) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "RINEX 模式需要广播星历文件");
            } else {
                // 记忆上次选择（项目外 ini）
                const auto &cfg = GnssCore::PathMemory::instance();
                cfg.set("rtk_base", this->baseObsPathBuf);
                cfg.set("rtk_rover", this->roverObsPathBuf);
                cfg.set("rtk_nav", this->navPathBuf);

                this->core->filePath = this->roverObsPathBuf;
                this->core->fileName = GuiHelpers::baseName(this->roverObsPathBuf) + " / "
                                        + GuiHelpers::baseName(this->baseObsPathBuf);
                LOG_INFO << "[RTK] 开始解算: 基准=" << this->baseObsPathBuf
                         << ", 流动站=" << this->roverObsPathBuf;

                this->state = State::Running;
                this->loading = true;
                this->core->loading = true;
                this->worker = std::thread(SolveThread, std::static_pointer_cast<RtkTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        GuiHelpers::renderCancelButton(this);

        GuiHelpers::endConfigBody();
    }

}
