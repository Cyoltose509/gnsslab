#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "GuiRealtimeProcessor.h"
#include "OEM7SocketReader.h"
#include "SPPIFCode.h"
#include "imgui.h"
#include "Const.h"
#include <chrono>
#include <thread>

namespace GuiRealtimeProcessor {
    void SolveRealtimeThread(const std::shared_ptr<SppTask> &task, const ConnectionConfig &config) {
        try {
            OEM7SocketReader reader;
            if (!reader.connect(config.ip, static_cast<unsigned short>(config.port))) {
                task->hasError = true;
                task->errorMsg = "无法连接到 " + config.ip + ":" + std::to_string(config.port);
                task->loading = false;
                task->done = true;
                return;
            }
            reader.setReceiveTimeout(100); // 100ms timeout

            SPPIFCode spp;
            spp.setIFCodeTypes({
                {'G', {"C1", "C2"}},
                {'C', {"C2", "C6"}}
            });

            ObsData obs;
            while (!task->stop) {
                try {
                    if (reader.getNextEpoch(obs)) {
                        // 构造星历映射
                        std::map<SatID, Ephemeris *> ephMap;
                        for (auto &[prn, eph]: reader.latestGps)
                            ephMap[SatID('G', prn)] = &eph;
                        for (auto &[prn, eph]: reader.latestBds)
                            ephMap[SatID('C', prn)] = &eph;

                        spp.setEphemeris(ephMap);
                        spp.preprocess(obs);

                        GuiOem7Processor::SppEpochData data;
                        data.getFromObs(obs);

                        try {
                            spp.solve(obs);
                            data.getFromSPP(spp);
                            if (!task->initializedRefECEF) {
                                task->refECEF = data.xyz;
                                task->initializedRefECEF = true;
                            }
                        } catch (...) {
                            data.solved = false;
                        }
                        {
                            std::lock_guard lock(task->mutex);
                            const bool wasAtEnd = task->selectedEpoch == -1 || task->selectedEpoch == static_cast<int>(task->epochs.size())
                                                  - 1;

                            if (task->epochs.empty()) {
                                task->weekFirst = data.week;
                                task->sowFirst = data.sow;
                            }
                            task->weekLast = data.week;
                            task->sowLast = data.sow;
                            task->epochs.push_back(data);

                            // 如果之前在最后一个历元，则自动跟随
                            if (wasAtEnd) {
                                task->selectedEpoch = static_cast<int>(task->epochs.size()) - 1;
                            }
                        }
                    } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                } catch (const std::exception &e) {
                    // 运行时错误但不中断连接
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            reader.close();
        } catch (const std::exception &e) {
            task->hasError = true;
            task->errorMsg = e.what();
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
