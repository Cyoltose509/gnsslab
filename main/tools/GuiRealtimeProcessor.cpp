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
                        data.week = obs.weekSecond.week;
                        data.sow = obs.weekSecond.sow;

                        for (auto &[sat, typeMap]: obs.satTypeValueData) {
                            data.satIds.push_back(sat);
                            auto it = typeMap.find("CC12");
                            if (it == typeMap.end()) it = typeMap.find("CC26");
                            data.pranges.push_back(it != typeMap.end() ? it->second : 0.0);
                            data.elevations.push_back(0.0);
                            data.azimuths.push_back(0.0);
                        }
                        data.numObs = static_cast<int>(data.satIds.size());

                        try {
                            spp.solve(obs);
                            auto &result = spp.result;

                            data.solved = true;
                            data.xyz = result.xyz;
                            data.blh = result.blh;
                            data.vel = result.vel;
                            data.pdop = result.pdop;
                            data.sigmaP = result.sigmaP;
                            data.sigmaV = result.sigmaV;
                            data.numSatsResult = result.numSats;

                            // 填充卫星高度角和方位角
                            for (int i = 0; i < static_cast<int>(data.satIds.size()); i++) {
                                if (auto it = spp.getElevData().find(data.satIds[i]); it != spp.getElevData().end())
                                    data.elevations[i] = it->second;
                                if (auto it2 = spp.getAzimData().find(data.satIds[i]); it2 != spp.getAzimData().end())
                                    data.azimuths[i] = it2->second;
                            }
                        } catch (...) {
                            data.solved = false;
                        }

                        {
                            std::lock_guard lock(task->mutex);
                            bool wasAtEnd = (task->selectedEpoch == -1 || task->selectedEpoch == static_cast<int>(task->epochs.size()) - 1);

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
