#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0
#define WIN32_LEAN_AND_MEAN
#include "GuiRealtimeProcessor.h"
#include "OEM7SocketReader.h"
#include "SPP.h"
#include "imgui.h"
#include "Const.h"
#include <chrono>
#include <thread>

namespace GuiRealtimeProcessor {
    void SolveRealtimeThread(const std::shared_ptr<SppTask> &task, const ConnectionConfig &config) {
        SPP spp;
        spp.setCutoffElevDeg(task->cutoffDeg); // 截止高度角：低于该仰角的卫星在解算中被剔除
        spp.enabledSystems = task->enabledSystems; // IF 组合由 SPP::solve 每历元从观测自探测（与文件模式一致）

        std::chrono::steady_clock::time_point lastQcTime; // QC 节流计时

        while (!task->stop) {
            try {
                OEM7SocketReader reader;
                if (!reader.connect(config.ip, static_cast<unsigned short>(config.port))) {
                    task->hasError = true;
                    task->errorMsg = "正在尝试连接 " + config.ip + ":" + std::to_string(config.port) + "...";
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }

                // 连接成功，重置错误状态
                task->hasError = false;
                task->errorMsg = "";
                reader.setReceiveTimeout(200); // 200ms timeout

                ObsData obs;
                while (!task->stop) {
                    try {
                        if (reader.getNextEpoch(obs)) {
                            spp.preprocess(obs);

                            GuiFileProcessor::SppEpochData data;
                            data.getFromObs(obs);
                            if (!task->hasNav && !obs.satEphemerisData.empty())
                                task->hasNav = true;

                            try {
                                spp.solve(obs);
                                data.getFromSPP(spp);
                                if (!task->initializedRefECEF) {
                                    task->refECEF = data.sppResult.xyz;
                                    task->initializedRefECEF = true;
                                }
                            } catch (...) {
                                data.solved = false;
                            }
                            {
                                std::lock_guard lock(task->mutex);
                                const auto index = static_cast<int>(task->epochs.size()) - 1;
                                {
                                    std::lock_guard plk(task->plotMutex);
                                    task->plotData.insert(index, data, task->refECEF);
                                }
                                const bool wasAtEnd = task->selectedEpoch == -1 || task->selectedEpoch == index;
                                task->epochs.push_back(data);

                                if (wasAtEnd) {
                                    task->selectedEpoch = index+1;
                                }
                            }

                            const auto nowQc = std::chrono::steady_clock::now();
                            if (nowQc - lastQcTime > std::chrono::milliseconds(500)) {
                                lastQcTime = nowQc;
                                GuiFileProcessor::LaunchQC(task);
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
                task->hasError = true;
                task->errorMsg = std::string("运行异常: ") + e.what();
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        task->loading = false;
        task->done = true;
    }

    void RenderTask(const std::shared_ptr<SppTask> &task) {
        // 实时任务没有文件配置阶段，直接进入运行态（防御旧任务/状态异常）
        if (task->state == GuiFileProcessor::SppTask::State::Config) {
            task->state = GuiFileProcessor::SppTask::State::Running;
        }

        // 实时任务在顶部加一个"停止"按钮
        if (!task->done) {
            if (ImGui::Button("停止连接")) {
                task->stop = true;
            }
            ImGui::SameLine();
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "连接已断开");
            ImGui::SameLine();
        }

        // 复用 GuiFileProcessor 的渲染逻辑
        GuiFileProcessor::RenderTask(task, true);
    }
}
