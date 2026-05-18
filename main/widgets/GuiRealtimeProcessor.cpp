#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0
#define WIN32_LEAN_AND_MEAN
#include "GuiRealtimeProcessor.h"
#include "OEM7SocketReader.h"
#include "SPPIFCode.h"
#include "imgui.h"
#include "Const.h"
#include <chrono>
#include <thread>

namespace GuiRealtimeProcessor {
    void SolveRealtimeThread(const std::shared_ptr<SppTask> &task, const ConnectionConfig &config) {
        SPPIFCode spp;
        spp.setIFCodeTypes({
            {'G', {"C1", "C2"}},
            {'C', {"C2", "C6"}}
        });

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

                            GuiOem7Processor::SppEpochData data;
                            data.getFromObs(obs);

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
                                task->plotData.insert(index, data, task->refECEF);
                                const bool wasAtEnd = task->selectedEpoch == -1 || task->selectedEpoch == index;
                                task->epochs.push_back(data);

                                if (wasAtEnd) {
                                    task->selectedEpoch = index+1;
                                }
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

        // 复用 GuiOem7Processor 的渲染逻辑
        GuiOem7Processor::RenderTask(task, true);
    }
} // namespace GuiRealtimeProcessor
