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
    static constexpr size_t kRealtimeQcWindow = 7200;

    void SolveRealtimeThread(const std::shared_ptr<GnssTask::SolveTask> &task, const ConnectionConfig &config) {
        SPP spp;
        spp.setCutoffElevDeg(task->cutoffDeg); // 截止高度角：低于该仰角的卫星在解算中被剔除
        spp.setEnabledSystems(task->enabledSystems); // IF 组合由 SPP::solve 每历元从观测自探测（与文件模式一致）

        std::chrono::steady_clock::time_point lastQcTime; // QC 节流计时
        bool loggedConnectFail = false;   // 连接失败每 2s 重试一次：只记首次，避免刷屏
        // 重启实时解算：复位参考点锁存（refECEF 否则残留上一轮值，ENU/导出全部锚错）
        task->initializedRefECEF = false;
        task->refAutoConfirmed = false;

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
                            {
                                std::lock_guard lk(task->qcInputMutex);
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
                 GnssTask::CommitEpoch(task, std::move(data));

                            if (const auto nowQc = std::chrono::steady_clock::now(); nowQc - lastQcTime > std::chrono::milliseconds(500)) {
                                lastQcTime = nowQc;
                                SolverPanel::LaunchQC(task);
                            }
                        } else {
                        if (!reader.isConnected() || reader.isStale(std::chrono::milliseconds(5000))) {
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
