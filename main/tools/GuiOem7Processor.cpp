#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include "GuiOem7Processor.h"
#include "imgui.h"
#include "OEM7Reader.h"
#include "SPPIFCode.h"
#include "Const.h"

#include <fstream>
#include <iostream>

namespace GuiOem7Processor {

    void SolveThread(const std::shared_ptr<SppTask> &task) {
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

                SppEpochData data;
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
                    if (task->epochs.empty()) {
                        task->weekFirst = data.week;
                        task->sowFirst = data.sow;
                        task->selectedEpoch = 0;
                    }
                    task->weekLast = data.week;
                    task->sowLast = data.sow;
                    task->epochs.push_back(data);
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
    // 渲染任务内容（主布局使用 Table 以支持拖拽分栏）
    // ================================================================
    void RenderTask(const std::shared_ptr<SppTask> &task) {
        int epochCount = 0;
        int selectedIdx;
        bool isLoading = task->loading.load();
        bool isDone = task->done.load();
        bool hasError = task->hasError;

        {
            std::lock_guard lock(task->mutex);
            epochCount = static_cast<int>(task->epochs.size());
            if (task->selectedEpoch >= epochCount) task->selectedEpoch = epochCount - 1;
            selectedIdx = task->selectedEpoch;
        }

        // --- 顶部状态与导航 ---
        if (isLoading) {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, 1.0f), "正在处理: 已解析 %d 个历元...", epochCount);
        } else if (hasError && epochCount == 0) {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "错误: %s", task->errorMsg.c_str());
        }

        if (epochCount > 0) {
            if (ImGui::Button("<<")) { task->selectedEpoch = 0; }
            ImGui::SameLine();
            if (ImGui::Button("<")) { if (task->selectedEpoch > 0) task->selectedEpoch--; }
            ImGui::SameLine();

            ImGui::PushItemWidth(150.0f);
            int ep = selectedIdx + 1;
            if (ImGui::SliderInt("##epoch_slider", &ep, 1, epochCount)) {
                task->selectedEpoch = ep - 1;
            }
            ImGui::PopItemWidth();

            ImGui::SameLine();
            if (ImGui::Button(">")) { if (task->selectedEpoch < epochCount - 1) task->selectedEpoch++; }
            ImGui::SameLine();
            if (ImGui::Button(">>")) { task->selectedEpoch = epochCount - 1; }

            ImGui::SameLine();
            {
                std::lock_guard lock(task->mutex);
                auto &cur = task->epochs[selectedIdx];
                ImGui::TextDisabled("| Wk %u SOW %.3f | %s", cur.week, cur.sow, cur.solved ? "已定位" : "未定位");
            }
        }

        ImGui::Separator();

        // --- 主分栏布局 (使用 Table 实现可拖拽分栏) ---
        if (ImGui::BeginTable("##main_split", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn("LeftPanel", ImGuiTableColumnFlags_WidthStretch, 0.6f);
            ImGui::TableSetupColumn("RightPanel", ImGuiTableColumnFlags_WidthStretch, 0.4f);

            ImGui::TableNextRow(ImGuiTableRowFlags_None, ImGui::GetContentRegionAvail().y - 35.0f);

            // ===== 左面板：卫星列表 =====
            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("##sat_list_child");
            if (epochCount > 0 && selectedIdx >= 0) {
                SppEpochData curData;
                {
                    std::lock_guard lock(task->mutex);
                    curData = task->epochs[selectedIdx];
                }

                ImGui::SeparatorText("卫星观测数据");
                if (ImGui::BeginTable("##sats", 6,
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_ScrollY)) {
                    ImGui::TableSetupColumn("PRN", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                    ImGui::TableSetupColumn("系统", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableSetupColumn("高度角(°)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("方位角(°)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("IF伪距(m)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("信号", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableHeadersRow();

                    for (int i = 0; i < static_cast<int>(curData.satIds.size()); i++) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%c%02d", curData.satIds[i].system, curData.satIds[i].id);

                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%s", curData.satIds[i].system == 'G' ? "GPS" : curData.satIds[i].system == 'C' ? "BDS" : "Other");

                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%.2f", curData.elevations[i] * RAD_TO_DEG);

                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%.2f", curData.azimuths[i] * RAD_TO_DEG);

                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%.3f", curData.pranges[i]);

                        ImGui::TableSetColumnIndex(5);
                        if (double elev = curData.elevations[i]; elev > 60.0 * DEG_TO_RAD) {
                            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "优");
                        } else if (elev > 30.0 * DEG_TO_RAD) {
                            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "良");
                        } else {
                            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "低");
                        }
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::EndChild();

            // ===== 右面板：解算结果 =====
            ImGui::TableSetColumnIndex(1);
            ImGui::BeginChild("##res_panel_child");
            if (epochCount > 0 && selectedIdx >= 0) {
                SppEpochData curData;
                {
                    std::lock_guard lock(task->mutex);
                    curData = task->epochs[selectedIdx];
                }

                if (curData.solved) {
                    ImGui::SeparatorText("位置 (WGS84)");
                    ImGui::Text("  X: %.4f m", curData.xyz[0]);
                    ImGui::Text("  Y: %.4f m", curData.xyz[1]);
                    ImGui::Text("  Z: %.4f m", curData.xyz[2]);
                    ImGui::Spacing();
                    ImGui::Text("  B: %.9f °", curData.blh[0] * RAD_TO_DEG);
                    ImGui::Text("  L: %.9f °", curData.blh[1] * RAD_TO_DEG);
                    ImGui::Text("  H: %.4f m", curData.blh[2]);

                    ImGui::Spacing();
                    ImGui::SeparatorText("速度");
                    ImGui::Text("  Vx: %.4f m/s", curData.vel[0]);
                    ImGui::Text("  Vy: %.4f m/s", curData.vel[1]);
                    ImGui::Text("  Vz: %.4f m/s", curData.vel[2]);
                    ImGui::Spacing();
                    ImGui::SeparatorText("精度");
                    ImGui::Text("  SigmaP: %.3f m", curData.sigmaP);
                    ImGui::Text("  SigmaV: %.3f m/s", curData.sigmaV);
                    ImGui::Text("  PDOP:   %.2f", curData.pdop);
                    ImGui::Text("  卫星数: %d", curData.numSatsResult);
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "该历元未获得定位解");
                    ImGui::TextWrapped("可能原因: 可用卫星不足或观测质量过差。");
                }

                ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 40.0f);
                if (isDone) {
                    if (ImGui::Button("导出结果 (CSV)", ImVec2(-FLT_MIN, 30.0f))) {
                        auto hwnd = static_cast<HWND>(ImGui::GetMainViewport()->PlatformHandleRaw);
                        if (!hwnd) hwnd = GetActiveWindow();
                        ExportCsv(task, hwnd);
                    }
                } else {
                    ImGui::Button("解析中...", ImVec2(-FLT_MIN, 30.0f));
                }
            }
            ImGui::EndChild();

            ImGui::EndTable();
        }

        // --- 底部时间轴 ---
        // ImGui::Separator();
        // if (epochCount > 0) {
        //     float normalized = (epochCount > 1) ? (float)selectedIdx / (float)(epochCount - 1) : 0.0f;
        //     ImGui::SetNextItemWidth(-FLT_MIN);
        //     if (ImGui::SliderFloat("##timeline", &normalized, 0.0f, 1.0f, "")) {
        //         task->selectedEpoch = (int)(normalized * (epochCount - 1) + 0.5f);
        //     }
        //     if (ImGui::IsItemHovered())
        //         ImGui::SetTooltip("拖动快速切换历元");
        // }
    }

    // ================================================================
    // 文件打开对话框
    // ================================================================
    std::string ShowOpenFileDialog(HWND hwnd) {//NOLINT
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
            return filename;
        }
        return "";
    }

    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND hwnd) { //NOLINT
        char filename[MAX_PATH] = "";

        std::string defaultName = task->fileName;
        if (const auto dotPos = defaultName.rfind('.'); dotPos != std::string::npos) defaultName = defaultName.substr(0, dotPos);
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

        std::lock_guard lock(task->mutex);

        std::ofstream out(filename);
        if (!out.is_open()) return;

        out << "Wk,SOW,Solved,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,B/deg,L/deg,H/m,"
                << "VX/m,VY/m,VZ/m,PDOP,SigmaP,SigmaV,SatCount\n";

        for (const auto &r: task->epochs) {
            out << r.week << ','
                    << std::fixed << std::setprecision(3) << r.sow << ','
                    << (r.solved ? "1" : "0") << ',';

            if (r.solved) {
                out << std::setprecision(4)
                        << r.xyz[0] << ',' << r.xyz[1] << ',' << r.xyz[2] << ','
                        << std::setprecision(9)
                        << r.blh[0] * RAD_TO_DEG << ',' << r.blh[1] * RAD_TO_DEG << ','
                        << std::setprecision(4)
                        << r.blh[2] << ','
                        << r.vel[0] << ',' << r.vel[1] << ',' << r.vel[2] << ','
                        << std::setprecision(4)
                        << r.pdop << ',' << r.sigmaP << ',' << r.sigmaV << ','
                        << r.numSatsResult;
            } else {
                out << ",,,,,,,,,,,,0";
            }
            out << '\n';
        }

        out.close();
    }
} // namespace GuiOem7Processor
