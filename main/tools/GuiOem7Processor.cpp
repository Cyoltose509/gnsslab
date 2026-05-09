#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>

#include "GuiOem7Processor.h"
#include "imgui.h"

#include "OEM7Reader.h"
#include "SPPIFCode.h"
#include "Const.h"
#include "CoordConvert.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstring>

namespace GuiOem7Processor {

    // ================================================================
    // 后台线程：读取 OEM7 文件并执行 SPP 定位
    // ================================================================
    void SolveThread(std::shared_ptr<SppTask> task) {
        try {
            OEM7Reader oem7;
            if (!oem7.open(task->filePath)) {
                task->hasError = true;
                task->errorMsg = "无法打开文件: " + task->filePath;
                task->loading = false;
                task->done = true;
                return;
            }

            SPPIFCode spp;
            spp.setIFCodeTypes({
                {'G', {"C1", "C2"}},
                {'C', {"C2", "C6"}}
            });

            ObsData obs;

            while (oem7.getNextEpoch(obs)) {
                // 构造星历映射
                std::map<SatID, Ephemeris *> ephMap;
                for (auto &[prn, eph]: oem7.latestGps)
                    ephMap[SatID('G', prn)] = &eph;
                for (auto &[prn, eph]: oem7.latestBds)
                    ephMap[SatID('C', prn)] = &eph;

                spp.setEphemeris(ephMap);
                spp.preprocess(obs);

                // 记录观测卫星数据（solve 前采集，因为 solve 会修改 obs）
                SppEpochObs epochObs;
                epochObs.week = obs.weekSecond.week;
                epochObs.sow = obs.weekSecond.sow;


                for (auto &[sat, typeMap]: obs.satTypeValueData) {
                    epochObs.satIds.push_back(sat);
                    auto it = typeMap.find("CC12");
                    if (it == typeMap.end()) it = typeMap.find("CC26");
                    epochObs.pranges.push_back(it != typeMap.end() ? it->second : 0.0);
                    epochObs.elevations.push_back(0.0);
                    epochObs.azimuths.push_back(0.0);
                }
                epochObs.numSats = (int)epochObs.satIds.size();

                try {
                    spp.solve(obs);

                    auto &result = spp.result;

                    // 填充卫星高度角和方位角（solve 后才有）
                    for (int i = 0; i < (int)epochObs.satIds.size(); i++) {
                        auto it = spp.getElevData().find(epochObs.satIds[i]);
                        if (it != spp.getElevData().end())
                            epochObs.elevations[i] = it->second;
                        auto it2 = spp.getAzimData().find(epochObs.satIds[i]);
                        if (it2 != spp.getAzimData().end())
                            epochObs.azimuths[i] = it2->second;
                    }

                    SppEpochResult er;
                    er.week = obs.weekSecond.week;
                    er.sow = obs.weekSecond.sow;
                    er.xyz = result.xyz;
                    er.blh = result.blh;
                    er.vel = result.vel;
                    er.pdop = result.pdop;
                    er.sigmaP = result.sigmaP;
                    er.sigmaV = result.sigmaV;
                    er.numSats = result.numSats;

                    {
                        std::lock_guard<std::mutex> lock(task->mutex);
                        task->results.push_back(er);
                        task->observations.push_back(epochObs);
                    }

                    // 更新时间范围（原子写入，单线程所以不需要锁）
                    if (task->results.size() == 1) {
                        task->weekFirst = er.week;
                        task->sowFirst = er.sow;
                    }
                    task->weekLast = er.week;
                    task->sowLast = er.sow;

                } catch (...) {
                    std::lock_guard<std::mutex> lock(task->mutex);
                    task->observations.push_back(epochObs);
                }
            }
        } catch (const std::exception &e) {
            task->hasError = true;
            task->errorMsg = e.what();
        }

        task->loading = false;
        task->done = true;
    }

    // ================================================================
    // 渲染任务内容（左右分栏）
    // ================================================================
    void RenderTask(std::shared_ptr<SppTask> task) {
        // 读取共享数据
        int resultCount = 0, obsCount = 0;
        int selectedIdx = task->selectedEpoch;
        bool isLoading = task->loading.load();
        bool isDone = task->done.load();
        bool hasError = task->hasError;

        {
            std::lock_guard<std::mutex> lock(task->mutex);
            resultCount = (int)task->results.size();
            obsCount = (int)task->observations.size();
            if (selectedIdx >= resultCount) selectedIdx = resultCount - 1;
            if (selectedIdx < 0) selectedIdx = 0;
        }

        // --- 左右分栏 ---
        float panelWidth = ImGui::GetContentRegionAvail().x * 0.55f;

        // ===== 左面板：卫星列表 =====
        ImGui::BeginChild("##sat_panel", ImVec2(panelWidth, 0), true);

        if (isLoading) {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, 1.0f), "正在解析并定位...");
            ImGui::SameLine();
            float t = (float)ImGui::GetTime();
            const char *spinChars = "|/-\\";
            int spinIdx = (int)(t * 4) % 4;
            ImGui::Text("%c", spinChars[spinIdx]);
            ImGui::SameLine();
            ImGui::Text("已处理 %d 个历元", resultCount);
        } else if (hasError && resultCount == 0) {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "错误: %s", task->errorMsg.c_str());
        }

        if (resultCount > 0 && selectedIdx >= 0 && selectedIdx < resultCount) {
            SppEpochResult curResult;
            SppEpochObs curObs;
            bool hasObs = false;

            {
                std::lock_guard<std::mutex> lock(task->mutex);
                curResult = task->results[selectedIdx];
                if (selectedIdx < obsCount) {
                    curObs = task->observations[selectedIdx];
                    hasObs = true;
                }
            }

            ImGui::Separator();
            ImGui::Text("历元 %d / %d", selectedIdx + 1, resultCount);
            ImGui::SameLine();
            ImGui::TextDisabled("Wk %u  SOW %.3f", curResult.week, curResult.sow);
            ImGui::Separator();

            // 历元导航
            if (ImGui::Button("<<")) { task->selectedEpoch = 0; }
            ImGui::SameLine();
            if (ImGui::Button("< ")) {
                if (task->selectedEpoch > 0) task->selectedEpoch--;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 100.0f);
            int ep = task->selectedEpoch + 1;
            if (ImGui::SliderInt("##epoch", &ep, 1, resultCount)) {
                task->selectedEpoch = ep - 1;
            }
            ImGui::SameLine();
            if (ImGui::Button(" >")) {
                if (task->selectedEpoch < resultCount - 1) task->selectedEpoch++;
            }
            ImGui::SameLine();
            if (ImGui::Button(">>")) { task->selectedEpoch = resultCount - 1; }

            ImGui::Spacing();
            ImGui::SeparatorText("卫星列表");

            if (hasObs && !curObs.satIds.empty()) {
                if (ImGui::BeginTable("##sats", 6,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                    ImGuiTableFlags_SizingStretchSame))
                {
                    ImGui::TableSetupColumn("PRN", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableSetupColumn("系统");
                    ImGui::TableSetupColumn("高度角(deg)");
                    ImGui::TableSetupColumn("方位角(deg)");
                    ImGui::TableSetupColumn("IF伪距(m)");
                    ImGui::TableSetupColumn("信号");
                    ImGui::TableHeadersRow();

                    for (int i = 0; i < (int)curObs.satIds.size(); i++) {
                        ImGui::TableNextRow();

                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%02d", curObs.satIds[i].id);

                        ImGui::TableSetColumnIndex(1);
                        const char *sysName = "?";
                        if (curObs.satIds[i].system == 'G') sysName = "GPS";
                        else if (curObs.satIds[i].system == 'C') sysName = "BDS";
                        ImGui::Text("%s", sysName);

                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%.2f", curObs.elevations[i] * RAD_TO_DEG);

                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%.2f", curObs.azimuths[i] * RAD_TO_DEG);

                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%.3f", curObs.pranges[i]);

                        ImGui::TableSetColumnIndex(5);
                        double elev = curObs.elevations[i];
                        if (elev > 60.0 * DEG_TO_RAD) {
                            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "优");
                        } else if (elev > 30.0 * DEG_TO_RAD) {
                            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "良");
                        } else {
                            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "低");
                        }
                    }
                    ImGui::EndTable();
                }
            } else {
                ImGui::TextDisabled("无卫星数据");
            }
        } else if (!isLoading && !hasError) {
            ImGui::TextDisabled("暂无解算结果");
        }

        // --- 底部时间轴 ---
        ImGui::Spacing();
        ImGui::Separator();
        if (resultCount > 0) {
            ImGui::Text("时间范围: Wk %u  SOW %.3f ~ %.3f  (共 %d 历元)",
                task->weekFirst, task->sowFirst, task->sowLast, resultCount);

            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            float normalized = (resultCount > 1)
                ? (float)(task->selectedEpoch) / (float)(resultCount - 1)
                : 0.0f;
            if (ImGui::SliderFloat("##timeline", &normalized, 0.0f, 1.0f, "")) {
                task->selectedEpoch = (int)(normalized * (resultCount - 1));
            }
        }

        ImGui::EndChild(); // sat_panel

        ImGui::SameLine();

        // ===== 右面板：解算结果 =====
        ImGui::BeginChild("##result_panel", ImVec2(0, 0), true);

        if (resultCount > 0 && selectedIdx >= 0 && selectedIdx < resultCount) {
            SppEpochResult curResult;
            {
                std::lock_guard<std::mutex> lock(task->mutex);
                curResult = task->results[selectedIdx];
            }

            ImGui::SeparatorText("ECEF 坐标 (m)");
            ImGui::Text("  X:  %.4f", curResult.xyz[0]);
            ImGui::Text("  Y:  %.4f", curResult.xyz[1]);
            ImGui::Text("  Z:  %.4f", curResult.xyz[2]);

            ImGui::Spacing();
            ImGui::SeparatorText("大地坐标");
            ImGui::Text("  B:  %.9f", curResult.blh[0] * RAD_TO_DEG);
            ImGui::Text("  L:  %.9f", curResult.blh[1] * RAD_TO_DEG);
            ImGui::Text("  H:  %.4f", curResult.blh[2]);

            ImGui::Spacing();
            ImGui::SeparatorText("速度 (m/s)");
            ImGui::Text("  VX: %.4f", curResult.vel[0]);
            ImGui::Text("  VY: %.4f", curResult.vel[1]);
            ImGui::Text("  VZ: %.4f", curResult.vel[2]);

            ImGui::Spacing();
            ImGui::SeparatorText("精度指标");
            ImGui::Text("  SigmaP: %.4f m", curResult.sigmaP);
            ImGui::Text("  SigmaV: %.4f m/s", curResult.sigmaV);
            ImGui::Text("  PDOP:    %.2f", curResult.pdop);
            ImGui::Text("  卫星数:  %d", curResult.numSats);

            ImGui::Spacing();
            ImGui::Separator();

            // 导出按钮
            if (isDone) {
                if (ImGui::Button("导出 CSV", ImVec2(120, 30))) {
                    HWND hwnd = (HWND)ImGui::GetMainViewport()->PlatformHandleRaw;
                    if (!hwnd) hwnd = GetActiveWindow();
                    ExportCsv(task, hwnd);
                }
                ImGui::SameLine();
                ImGui::TextDisabled("共 %d 个历元", resultCount);
            } else {
                ImGui::TextDisabled("解析完成后可导出...");
            }
        } else {
            ImGui::TextDisabled("等待解算结果...");
        }

        // 加载指示器（右下角）
        if (isLoading) {
            ImVec2 windowSize = ImGui::GetWindowSize();
            float t = (float)ImGui::GetTime();
            const char *spinChars = "|/-\\";
            int spinIdx = (int)(t * 4) % 4;
            ImGui::SetCursorPos(ImVec2(windowSize.x - 40, windowSize.y - 30));
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, 1.0f), "%c", spinChars[spinIdx]);
        }

        ImGui::EndChild(); // result_panel
    }

    // ================================================================
    // 文件打开对话框
    // ================================================================
    std::string ShowOpenFileDialog(HWND hwnd) {
        char filename[MAX_PATH] = "";

        OPENFILENAMEA ofn = {};
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = hwnd;
        ofn.lpstrFilter = "OEM7 Log Files (*.log)\0*.log\0All Files\0*.*\0";
        ofn.lpstrFile = filename;
        ofn.nMaxFile = MAX_PATH;
        ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
        ofn.lpstrDefExt = "log";

        if (GetOpenFileNameA(&ofn)) {
            return std::string(filename);
        }
        return "";
    }

    // ================================================================
    // 导出 CSV
    // ================================================================
    void ExportCsv(std::shared_ptr<SppTask> task, HWND hwnd) {
        char filename[MAX_PATH] = "";

        std::string defaultName = task->fileName;
        auto dotPos = defaultName.rfind('.');
        if (dotPos != std::string::npos) defaultName = defaultName.substr(0, dotPos);
        defaultName += "_spp.csv";
        if (defaultName.size() >= MAX_PATH) defaultName = "spp_results.csv";
        strncpy_s(filename, MAX_PATH, defaultName.c_str(), _TRUNCATE);

        OPENFILENAMEA ofn = {};
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = hwnd;
        ofn.lpstrFilter = "CSV Files (*.csv)\0*.csv\0All Files\0*.*\0";
        ofn.lpstrFile = filename;
        ofn.nMaxFile = MAX_PATH;
        ofn.Flags = OFN_OVERWRITEPROMPT;
        ofn.lpstrDefExt = "csv";

        if (!GetSaveFileNameA(&ofn)) return;

        std::lock_guard<std::mutex> lock(task->mutex);

        std::ofstream out(filename);
        if (!out.is_open()) return;

        out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,B/deg,L/deg,H/m,"
            << "VX/m,VY/m,VZ/m,PDOP,SigmaP,SigmaV,SatCount\n";

        for (const auto &r: task->results) {
            out << r.week << ','
                << std::fixed << std::setprecision(3) << r.sow << ','
                << std::setprecision(4)
                << r.xyz[0] << ',' << r.xyz[1] << ',' << r.xyz[2] << ','
                << std::setprecision(9)
                << r.blh[0] * RAD_TO_DEG << ',' << r.blh[1] * RAD_TO_DEG << ','
                << std::setprecision(4)
                << r.blh[2] << ','
                << r.vel[0] << ',' << r.vel[1] << ',' << r.vel[2] << ','
                << std::setprecision(4)
                << r.pdop << ',' << r.sigmaP << ',' << r.sigmaV << ','
                << r.numSats << '\n';
        }

        out.close();
    }

} // namespace GuiOem7Processor
