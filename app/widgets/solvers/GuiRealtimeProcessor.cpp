#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0
#define WIN32_LEAN_AND_MEAN
#include "SolverTask.h"
#include "OEM7SocketReader.h"
#include "SPP.h"
#include "imgui.h"
#include "Const.h"
#include "GuiRealtimeProcessor.h"
#include <chrono>
#include <thread>
#include "GuiSolverPanel.h"
#include "Log.h"

namespace Solvers::RealtimeProcessor {
    // 实时质量分析滑动窗口上限（与 RTK 实时流一致，约 2 小时@1Hz）：
    // 避免 QC::compute 随历元数无限增长，做增量式质量分析。
    static constexpr size_t kRealtimeQcWindow = 7200;

    void SolveRealtimeThread(const std::shared_ptr<GnssTask::SolveTask> &task, const ConnectionConfig &config) {
        SPP spp;
        spp.setCutoffElevDeg(task->cutoffDeg); // 截止高度角：低于该仰角的卫星在解算中被剔除
        spp.setEnabledSystems(task->enabledSystems); // IF 组合由 SPP::solve 每历元从观测自探测（与文件模式一致）

        std::chrono::steady_clock::time_point lastQcTime; // QC 节流计时
        bool loggedConnectFail = false;   // 连接失败每 2s 重试一次：只记首次，避免刷屏

        while (!task->stop) {
            try {
                OEM7SocketReader reader;
                if (!reader.connect(config.ip, static_cast<unsigned short>(config.port))) {
                    if (!loggedConnectFail) {
                        LOG_WARN << "[实时] 连接失败 " << config.ip << ":" << config.port
                                 << "，每 2s 重试";
                        loggedConnectFail = true;
                    }
                    task->setError("正在尝试连接 " + config.ip + ":" + std::to_string(config.port) + "...");
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }

                // 连接成功，重置错误状态
                task->clearError();
                if (loggedConnectFail) {
                    LOG_INFO << "[实时] 已连接 " << config.ip << ":" << config.port;
                    loggedConnectFail = false;
                }
                reader.setReceiveTimeout(200); // 200ms timeout

                ObsData obs;
                while (!task->stop) {
                    try {
                        if (reader.getNextEpoch(obs)) {
                            auto eph = reader.ephSnapshot(); // OEM7 内嵌星历统一经 ephTable 传入
                            spp.ephTable = *eph;
                            spp.preprocess(obs);

                            // 实时质量分析：把预处理后的原始观测并入 qcInput（与文件/OEM7 模式一致）。
                            // 之前这一环漏了，导致 QC 每次算的都是空输入 → 一直显示"无有效双频观测"。
                            {
                                std::lock_guard<std::mutex> lk(task->qcInputMutex);
                                task->qcInput.push_back(QC::makeQCObsEpoch(obs, obs.weekSecond.sow));
                                if (task->qcInput.size() > kRealtimeQcWindow)
                                    task->qcInput.erase(
                                        task->qcInput.begin(),
                                        task->qcInput.begin()
                                            + (task->qcInput.size() - kRealtimeQcWindow));
                            }

                            GnssTask::EpochData data;
                            data.getFromObs(obs);
                            if (!task->hasNav && !eph->empty())
                                task->hasNav = true;

                            try {
                                spp.processEpoch(obs);
                                data.getFromSPP(spp);
                                if (!task->initializedRefECEF) {
                                    task->refECEF = data.sppResult.xyz;
                                    task->initializedRefECEF = true;
                                }
                            } catch (...) {
                                data.solved = false;
                            }
                            // 统一提交入口：与 RTK（commitRtkEpoch → CommitEpoch）同形，
                            // 取代原先手写 push_back + plotData.insert + selectedEpoch 维护（且旧逻辑 index 计算有 off-by-one）。
                            GnssTask::CommitEpoch(task, std::move(data));

                            const auto nowQc = std::chrono::steady_clock::now();
                            if (nowQc - lastQcTime > std::chrono::milliseconds(500)) {
                                lastQcTime = nowQc;
                                Solvers::SolverPanel::LaunchQC(task);
                            }
                        } else {
                            // 检查连接是否依然有效
                            if (!reader.isConnected()) {
                                break; // 跳出内层循环进行重连
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        }
                    } catch (const std::exception &) {
                        if (!reader.isConnected()) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
                reader.close();
            } catch (const std::exception &e) {
                task->setError(std::string("运行异常: ") + e.what());
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        task->loading = false;
        task->done = true;
    }
}
