#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include "GuiOem7Processor.h"
#include "imgui.h"
#include "OEM7Reader.h"
#include "SPPIFCode.h"
#include "Const.h"
#include "implot.h"
#include "implot3d.h"

#include <fstream>
#include <iostream>

namespace GuiOem7Processor {
    void SppEpochData::getFromSPP(const SPPIFCode &spp) {
        if (auto &result = spp.result; result.numSats > 0) {
            solved = true;
            xyz = result.xyz;
            blh = result.blh;
            vel = result.vel;
            pdop = result.pdop;
            sigmaP = result.sigmaP;
            sigmaV = result.sigmaV;
            numSatsResult = result.numSats;
            const auto size = static_cast<int>(satIds.size());
            for (int i = 0; i < size; i++) {
                if (auto it = spp.satElevData.find(satIds[i]); it != spp.satElevData.end())
                    elevations[i] = it->second;
                if (auto it2 = spp.satAzimData.find(satIds[i]); it2 != spp.satAzimData.end())
                    azimuths[i] = it2->second;
                if (spp.satRejected.count(satIds[i])) {
                    rejected[i] = true;
                    numSatsResult--;
                }
            }
        } else {
            solved = false;
        }
    }

    void SppEpochData::getFromObs(const ObsData &obs) {
        week = obs.weekSecond.week;
        sow = obs.weekSecond.sow;
        for (auto &[sat, typeMap]: obs.satTypeValueData) {
            satIds.push_back(sat);
            auto it = typeMap.find("CC12");
            if (it == typeMap.end()) it = typeMap.find("CC26");
            pranges.push_back(it != typeMap.end() ? it->second : 0.0);
            elevations.push_back(0.0);
            azimuths.push_back(0.0);
            rejected.push_back(false);
        }
        numObs = static_cast<int>(satIds.size());
    }

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
                    const bool wasAtEnd = (task->selectedEpoch == -1 || task->selectedEpoch == static_cast<int>(task->epochs.size()) - 1);
                    if (task->epochs.empty()) {
                        task->weekFirst = data.week;
                        task->sowFirst = data.sow;
                        task->selectedEpoch = 0;
                    }
                    task->weekLast = data.week;
                    task->sowLast = data.sow;
                    task->epochs.push_back(data);
                    if (wasAtEnd) {
                        task->selectedEpoch = static_cast<int>(task->epochs.size()) - 1;
                    }
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
    void RenderTask(const std::shared_ptr<SppTask> &task, const bool isRealtime) {
        int epochCount = 0;
        int selectedIdx;
        const bool isLoading = task->loading.load();
        const bool isDone = task->done.load();
        const bool hasError = task->hasError;

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
        {
            ImGui::Text("参考真值：");
            ImGui::SameLine();
            ImGui::PushItemWidth(200);
            ImGui::InputDouble("X/m", &task->refECEF[0]);
            ImGui::SameLine();
            ImGui::InputDouble("Y/m", &task->refECEF[1]);
            ImGui::SameLine();
            ImGui::InputDouble("Z/m", &task->refECEF[2]);
            ImGui::SameLine();
            if (ImGui::Button("从剪贴板读取")) {
                if (const char *clipText = ImGui::GetClipboardText(); clipText && clipText[0] != '\0') {
                    double vals[3] = {0, 0, 0};
                    int count = 0;
                    const char *p = clipText;
                    char *end;
                    while (*p && count < 3) {
                        // 跳过非数字字符（包括空格、逗号等）
                        while (*p && !(*p == '-' || (*p >= '0' && *p <= '9'))) p++;
                        if (!*p) break;

                        vals[count] = strtod(p, &end);
                        if (p == end) break; // 没解析到数字
                        p = end;
                        count++;
                    }
                    if (count >= 1) task->refECEF[0] = vals[0];
                    if (count >= 2) task->refECEF[1] = vals[1];
                    if (count >= 3) task->refECEF[2] = vals[2];
                }
            }
        }

        ImGui::Separator();

        // --- 主分栏布局 (使用 Table 实现可拖拽分栏) ---
        float availY = ImGui::GetContentRegionAvail().y;
        float plotRegionH = (epochCount > 0) ? 380.0f : 0.0f;
        float mainTableH = availY - plotRegionH - ImGui::GetStyle().ItemSpacing.y;
        if (mainTableH < 200.0f) mainTableH = 200.0f;

        if (ImGui::BeginTable("##main_split", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn("LeftPanel", ImGuiTableColumnFlags_WidthStretch, 0.6f);
            ImGui::TableSetupColumn("RightPanel", ImGuiTableColumnFlags_WidthStretch, 0.4f);

            ImGui::TableNextRow(ImGuiTableRowFlags_None, mainTableH);

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
                if (ImGui::BeginTable("##sats", 7,
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_ScrollY)) {
                    ImGui::TableSetupColumn("序号", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableSetupColumn("PRN", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                    ImGui::TableSetupColumn("高度角(°)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("方位角(°)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("IF伪距(m)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("信号", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableHeadersRow();

                    for (int i = 0; i < static_cast<int>(curData.satIds.size()); i++) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%d", i + 1);


                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%c%02d", curData.satIds[i].system, curData.satIds[i].id);

                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%.2f", curData.elevations[i] * RAD_TO_DEG);

                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%.2f", curData.azimuths[i] * RAD_TO_DEG);

                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%.3f", curData.pranges[i]);

                        ImGui::TableSetColumnIndex(5);
                        if (const double elev = curData.elevations[i]; elev > 60.0 * DEG_TO_RAD) {
                            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "优");
                        } else if (elev > 30.0 * DEG_TO_RAD) {
                            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "良");
                        } else {
                            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "低");
                        }
                        ImGui::TableSetColumnIndex(6);
                        if (curData.rejected[i]) {
                            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "排除");
                        } else {
                            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "参与");
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
                    auto enu = XYZtoENU(curData.xyz, task->refECEF);
                    ImGui::SeparatorText("位置 (WGS84)");
                    ImGui::Text("  ECEF: (%.4f, %.4f, %.4f) m", curData.xyz[0], curData.xyz[1], curData.xyz[2]);
                    ImGui::Text("    REF: (%.4f, %.4f, %.4f) m", task->refECEF[0], task->refECEF[1], task->refECEF[2]);
                    ImGui::Text("  ENU: (%.4f, %.4f, %.4f) m", enu[0], enu[1], enu[2]);
                    ImGui::Text("  BLH: (%.8f°, %.8f°, %.4f m)", curData.blh[0] * RAD_TO_DEG, curData.blh[1] * RAD_TO_DEG, curData.blh[2]);
                    ImGui::Text("  SigmaP: %.3f m", curData.sigmaP);
                    ImGui::Spacing();
                    ImGui::SeparatorText("速度");
                    ImGui::Text("  (%.4f, %.4f, %.4f) m/s", curData.vel[0], curData.vel[1], curData.vel[2]);
                    ImGui::Text("  SigmaV: %.3f m/s", curData.sigmaV);
                    ImGui::Spacing();
                    ImGui::SeparatorText("精度");
                    ImGui::Text("  PDOP:   %.2f", curData.pdop);
                    ImGui::Text("  卫星数: %d/%d", curData.numSatsResult, curData.numObs);
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "该历元未获得定位解");
                    ImGui::TextWrapped("可能原因: 可用卫星不足或观测质量过差。");
                }
                ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 50.0f);
                if (isDone || isRealtime) {
                    if (ImGui::Button("导出结果 (CSV)", ImVec2(-FLT_MIN, 40.0f))) {
                        auto hwnd = static_cast<HWND>(ImGui::GetMainViewport()->PlatformHandleRaw);
                        if (!hwnd) hwnd = GetActiveWindow();
                        ExportCsv(task, hwnd);
                    }
                } else {
                    ImGui::Button("解析中...", ImVec2(-FLT_MIN, 40.0f));
                }
            }
            ImGui::EndChild();

            ImGui::EndTable();
        }

        // --- 绘图区域 ---

        if (epochCount > 0) {
            std::vector<double> times;
            std::vector<double> sigmaPs;
            std::vector<double> sigmaVs;
            std::vector<double> pdops;
            std::vector<double> enu_e;
            std::vector<double> enu_n;
            std::vector<double> enu_u;

            // 提取绘图数据
            {
                std::lock_guard lock(task->mutex);
                const int n = static_cast<int>(task->epochs.size());
                times.resize(n);
                sigmaPs.resize(n);
                sigmaVs.resize(n);
                pdops.resize(n);
                enu_e.resize(n);
                enu_n.resize(n);
                enu_u.resize(n);
                for (int i = 0; i < n; i++) {
                    const auto &ep = task->epochs[i];
                    times[i] = i;
                    sigmaPs[i] = ep.solved ? ep.sigmaP : 0.0;
                    sigmaVs[i] = ep.solved ? ep.sigmaV : 0.0;
                    pdops[i] = ep.solved ? ep.pdop : 0.0;
                    auto enu = XYZtoENU(ep.xyz, task->refECEF);
                    enu_e[i] = enu[0];
                    enu_n[i] = enu[1];
                    enu_u[i] = enu[2];
                }
                //-------------------------------
                ImGui::Separator();
                if (ImPlot::BeginPlot("ENU时序图", ImVec2(-1, 400))) {
                    ImPlot::SetupAxes("Epoch", "E/N/U (m)");
                    ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Once);
                    if (task->loading) {
                        ImPlot::SetupAxisLimits(ImAxis_X1, 0, times.empty() ? 10 : times.back(),
                                                ImPlotCond_Always);
                    }
                    if (!times.empty()) {
                        ImPlot::PlotLine("ENU-E", times.data(), enu_e.data(), static_cast<int>(times.size()));
                        ImPlot::PlotLine("ENU-N", times.data(), enu_n.data(), static_cast<int>(times.size()));
                        ImPlot::PlotLine("ENU-U", times.data(), enu_u.data(), static_cast<int>(times.size()));
                        // 标记当前选中历元
                        if (selectedIdx >= 0 && selectedIdx < static_cast<int>(times.size())) {
                            double curTime = times[selectedIdx];
                            if (ImPlot::DragLineX(1, &curTime, ImVec4(255, 0, 0, 255))) {
                                task->selectedEpoch = static_cast<int>(std::clamp(curTime, 0.0, times.back()));
                            }
                        }
                    }
                    ImPlot::EndPlot();
                }
                ImGui::Separator();
                if (ImGui::BeginTable("##main_split", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
                    ImGui::TableSetupColumn("LeftPanel", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                    ImGui::TableSetupColumn("MiddlePanel", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                    ImGui::TableSetupColumn("RightPanel", ImGuiTableColumnFlags_WidthStretch, 0.33f);
                    ImGui::TableNextRow(ImGuiTableRowFlags_None, mainTableH);
                    vector<double> *data[3] = {
                        &sigmaPs,
                        &sigmaVs,
                        &pdops
                    };
                    for (int i = 0; i < 3; i++) {
                        const char *names[3] = {
                            "SigmaP",
                            "SigmaV",
                            "PDOP"
                        };
                        ImGui::TableSetColumnIndex(i);
                        if (ImPlot::BeginPlot(names[i], ImVec2(-1, 400), ImPlotFlags_NoLegend)) {
                            ImPlot::SetupAxes("Epoch", "σ (m)");
                            if (task->loading) {
                                ImPlot::SetupAxisLimits(ImAxis_X1, 0, times.empty() ? 10 : times.back(),
                                                        ImPlotCond_Always);
                            }
                            if (!times.empty()) {
                                ImPlot::PlotLine(names[i], times.data(), data[i]->data(), static_cast<int>(times.size()));
                                if (selectedIdx >= 0 && selectedIdx < static_cast<int>(times.size())) {
                                    double curTime = times[selectedIdx];
                                    if (ImPlot::DragLineX(1, &curTime, ImVec4(255, 0, 0, 255))) {
                                        task->selectedEpoch = static_cast<int>(std::clamp(curTime, 0.0, times.back()));
                                    }
                                }
                            }
                            ImPlot::EndPlot();
                        }
                    }
                    ImGui::EndTable();
                }

                //-------------------------------
            }
        }
    }

    // ================================================================
    // 文件打开对话框
    // ================================================================
    std::string ShowOpenFileDialog(HWND hwnd) { //NOLINT
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

        out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-ECEF-X/m,REF-ECEF-Y/m,REF-ECEF-Z/m,EAST/m,NORTH/m,UP/m,B/deg,L/deg,H/m,"
                << "VX/m,VY/m,VZ/m,PDOP,SigmaP,SigmaV,SatCount\n";

        for (const auto &r: task->epochs) {
            out << r.week << ','
                    << std::fixed << std::setprecision(3) << r.sow << ',';

            if (r.solved) {
                auto enu = XYZtoENU(r.xyz, task->refECEF);
                out << std::setprecision(4)
                        << r.xyz[0] << ',' << r.xyz[1] << ',' << r.xyz[2] << ','
                        << task->refECEF.X() << ','
                        << task->refECEF.Y() << ','
                        << task->refECEF.Z() << ','
                        << enu.E() << ','
                        << enu.N() << ','
                        << enu.U() << ','
                        << std::setprecision(8)
                        << r.blh[0] * RAD_TO_DEG << ',' << r.blh[1] * RAD_TO_DEG << ','
                        << std::setprecision(3)
                        << r.blh[2] << ','
                        << r.vel[0] << ',' << r.vel[1] << ',' << r.vel[2] << ','
                        << std::setprecision(4)
                        << r.pdop << ',' << r.sigmaP << ',' << r.sigmaV << ','
                        << r.numSatsResult;
            } else {
                //out << ",,,,,,,,,,,,0";
            }
            out << '\n';
        }

        out.close();
    }
} // namespace GuiOem7Processor
