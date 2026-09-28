#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include "GuiRtkProcessor.h"
#include "GuiHelpers.h"   // fileRow / renderCutoffAndSystems / inputTextStd / beginConfigWindow / renderCancelButton
#include "core/AppConfig.h"
#include "ui/Gui.h"
#include "imgui.h"
#include "RTK.h"               // 手写 RTK 双差非组合求解器
#include "SPP.h"               // OEM7 无 BESTPOS 时用 SPP 估基准/流动站近似坐标
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "OEM7Reader.h"
#include "CoordConvert.h"
#include "Exception.h"         // EndOfFile, SyncException
#include "QualityControl.h"    // QC::makeQCObsEpoch / QualityReport
#include "Log.h"

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

    // 取首个含有效 BESTPOS 天线位置的历元坐标（OEM7 首个 RANGE 常为 (0,0,0)）
    static bool firstBestpos(const std::vector<ObsData> &E, Vector3d &out) {
        for (const auto &e : E)
            if (e.antennaPosition.norm() > 1e3) { out = e.antennaPosition; return true; }
        return false;
    }

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
                // ---- OEM7 路径（星历由日志逐历元内联，广播星历可选）----
                OEM7Reader br, rr;
                if (!br.open(task->baseObsPathBuf) || !rr.open(task->roverObsPathBuf)) {
                    failRead(task, "无法打开 OEM7 日志");
                    return;
                }
                std::vector<ObsData> baseE, roverE;
                std::vector<EphemerisTable> baseEph, roverEph;
                br.readAll(baseE, baseEph);
                rr.readAll(roverE, roverEph);
                if (baseE.empty() || roverE.empty()) {
                    failRead(task, "OEM7 无历元");
                    return;
                }
                // 基准站坐标：优先首个有效 BESTPOS 历元；否则用 SPP 估（OEM7 课程日志常无 BESTPOS）。
                // 二者皆无 → 明确报错，不再静默 "无历元数据"。
                Vector3d baseXyz = Vector3d::Zero();
                bool haveBase = firstBestpos(baseE, baseXyz);
                if (!haveBase) {
                    // （原 bi0 索引循环已删：其赋值从未被使用，且 sppApprox 内部本就会跳过空星历）
                    haveBase = SPP::estimateApproxPosition(baseE, baseEph, task->enabledSystems, task->cutoffDeg, baseXyz);
                }
                if (!haveBase) {
                    failRead(task, "OEM7 基准站日志无有效 BESTPOS 且 SPP 无法定位；请用 RINEX 或在配置中提供基准站坐标");
                    return;
                }
                const Vector3d baseApprox = baseXyz;

                // 流动站线性化初值：优先 BESTPOS，否则 SPP 估（短基线二者相近）
                Vector3d roverXyz = Vector3d::Zero();
                bool haveRover = firstBestpos(roverE, roverXyz);
                if (!haveRover) {
                    // （原 ri0 索引循环已删：同上）
                    haveRover = SPP::estimateApproxPosition(roverE, roverEph, task->enabledSystems, task->cutoffDeg, roverXyz);
                }
                const Vector3d roverApprox = haveRover ? roverXyz : Vector3d::Zero();
                LOG_INFO << "[RTK] OEM7 base XYZ = " << baseXyz.transpose()
                         << (firstBestpos(baseE, baseXyz) ? " (BESTPOS)" : " (SPP 估算)");

                // 双站历元时间对齐由 RTK::matchEpochPairs 提供（原为 GUI 内联的双指针扫描）
                int ok = 0;
                for (const auto &pr: RTK::matchEpochPairs(baseE, roverE)) {
                    if (task->stop) break;
                    const size_t bi = pr.first, ri = pr.second;
                    if (bi >= baseEph.size()) continue; // 星历快照较短时跳过（原代码此处会越界）
                    // EphemerisTable::gps/bds 存的是 vector<shared_ptr<...>>，直接整体浅拷贝（共享星历）即可
                    auto eph = std::make_shared<EphemerisTable>(baseEph[bi]);
                    {
                        std::lock_guard lk(task->pairMtx);
                        task->pairQueue.push({baseE[bi], roverE[ri], eph, baseApprox, roverApprox});
                    }
                    task->pairCv.notify_one();
                    {
                        std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                        task->core->qcInput.push_back(QC::makeQCObsEpoch(roverE[ri], roverE[ri].weekSecond.sow));
                    }
                    baseQcInput.push_back(QC::makeQCObsEpoch(baseE[bi], baseE[bi].weekSecond.sow));
                    ++ok;
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
                Vector3d baseXyz = base.antennaPosition;
                if (baseXyz.norm() < 1e3) {
                    // RINEX 头缺 APPROX POSITION XYZ → 用 SPP 估（广播星历已加载）
                    ObsData work = base;
                    work.antennaPosition = Vector3d::Zero();
                    SPP spp;
                    spp.ephTable = task->navTable;
                    spp.setEnabledSystems(task->enabledSystems);
                    spp.setCutoffElevDeg(task->cutoffDeg);
                    try { if (spp.processEpoch(work)) baseXyz = spp.result.xyz; }
                    catch (const std::exception &e) { LOG_WARN << "[RTK] RINEX base SPP 失败: " << e.what(); }
                }
                if (baseXyz.norm() < 1e3) {
                    failRead(task, "RINEX 基准站无 APPROX POSITION XYZ 且 SPP 无法定位；请提供含坐标的观测文件");
                    return;
                }
                const Vector3d baseApprox = baseXyz;
                LOG_INFO << "[RTK] RINEX base XYZ = " << baseXyz.transpose() << " (RINEX 头 APPROX / SPP)";

                // 流动站线性化初值取首个历元，之后沿用；随历元入队，不再写共享的 RTK 成员
                Vector3d roverApprox = Vector3d::Zero();
                bool roverApproxSet = false;
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
                        if (!roverApproxSet) {
                            roverApprox = rover.antennaPosition;
                            roverApproxSet = true;
                        }
                        {
                            std::lock_guard lk(task->pairMtx);
                            task->pairQueue.push({base, rover, nullptr, baseApprox, roverApprox});
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

                // 近似坐标随历元传入；task->rtk 现为解算线程独占，读取线程不再触碰。
                task->rtk.mBaseXyz = item.baseApprox;
                task->rtk.mRoverApprox = item.roverApprox;

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

                const Vector3d baseXyz = item.baseApprox;
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

    void feedRtkEpoch(EpochData &data, const RTK &rtk) {
        const Result &r = rtk.result;
        if (r.numSats <= 0) { data.solved = false; return; }
        data.solved = true;
        data.sppResult = r;               // stat/ratio/numSats/sigmaP/sigmaXYZ/postRes/xyz 全带走
        data.rtkStat = r.stat;
        data.rtkRatio = r.ratio;
        data.rtkBaseFloat = rtk.mBaselineFloat;

        const int size = static_cast<int>(data.satIds.size());
        for (int i = 0; i < size; ++i) {
            const SatID &sat = data.satIds[i];
            auto itE = rtk.satElevData.find(sat);
            auto itA = rtk.satAzimData.find(sat);
            auto itP = rtk.satPVTRecTime.find(sat);
            if (itE != rtk.satElevData.end()) data.elevations[i] = itE->second;
            if (itA != rtk.satAzimData.end()) data.azimuths[i] = itA->second;
            if (itP != rtk.satPVTRecTime.end()) data.satPVTs[i] = itP->second;
        }
        // 排除判据与 getFromSPP 同源：未进入双差解算，或无有效星历/位置的卫星
        data.applySatRule([&](const SatID &sat, const PVT &pvt) {
            return rtk.mUsedSats.count(sat) > 0 && pvt.p.squaredNorm() > 1.0;
        });
    }

    static void SolveThread(const std::shared_ptr<RtkTask> &task) {
        task->rtkCfg.cutoffElevRad = task->cutoffDeg * PI / 180.0;
        task->rtkCfg.enabledSystems = task->enabledSystems;
        task->rtk.setConfig(task->rtkCfg);

        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();

        task->state = RtkTask::State::Done;
        task->done = true;
        task->loading = false;
        GnssTask::SyncTaskStatus(task, task->core);
    }

    void RtkTask::renderConfigPanel() {
        GuiHelpers::beginConfigWindow("RTK 处理配置###rtk_cfg_", *this);

        // 与 PPP/LEO 完全一致：直接复用 GuiHelpers 预置过滤器（fObs 已含 *.??O 与 *.log，fRnx 含广播星历）
        GuiHelpers::fileRow("基准站观测文件", &this->baseObsPathBuf, "rtk_base", GuiHelpers::fObs, 3, {});
        GuiHelpers::fileRow("流动站观测文件", &this->roverObsPathBuf, "rtk_rover", GuiHelpers::fObs, 3, {});
        GuiHelpers::fileRow("广播星历 BRDC", &this->navPathBuf, "rtk_nav", GuiHelpers::fRnx, 2, {});
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
                const auto &cfg = AppConfig::instance();
                cfg.set("rtk_base", this->baseObsPathBuf);
                cfg.set("rtk_rover", this->roverObsPathBuf);
                cfg.set("rtk_nav", this->navPathBuf);

                this->core->filePath = this->roverObsPathBuf;
                this->core->fileName = GuiHelpers::baseName(this->roverObsPathBuf) + " / "
                                        + GuiHelpers::baseName(this->baseObsPathBuf);

                this->state = State::Running;
                this->loading = true;
                this->core->loading = true;
                this->worker = std::thread(SolveThread, std::static_pointer_cast<RtkTask>(shared_from_this()));
            }
        }
        ImGui::SameLine();
        GuiHelpers::renderCancelButton(this);

        ImGui::End();
    }

}
