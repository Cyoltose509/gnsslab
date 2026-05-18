#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include "GuiOem7Processor.h"
#include "imgui.h"
#include "OEM7Reader.h"
#include "SPPIFCode.h"
#include "Const.h"
#include "implot.h"

#include <fstream>
#include <iostream>
#include <set>
#include <algorithm>


namespace GuiOem7Processor {
    auto oldTime = CommonTime(61120);

    void SppEpochData::getFromSPP(const SPPIFCode &spp) {
        if (auto &result = spp.result; result.numSats > 0) {
            solved = true;
            sppResult = result;

            numSatsResult = result.numSats;
            const auto size = static_cast<int>(satIds.size());
            for (int i = 0; i < size; i++) {
                if (auto it = spp.satElevData.find(satIds[i]); it != spp.satElevData.end())
                    elevations[i] = it->second;
                if (auto it2 = spp.satAzimData.find(satIds[i]); it2 != spp.satAzimData.end())
                    azimuths[i] = it2->second;
                if (auto it3 = spp.satPVTTransTime.find(satIds[i]); it3 != spp.satPVTTransTime.end())
                    satPVTs[i] = it3->second;
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
            elevations.push_back(0.0);
            azimuths.push_back(0.0);
            satPVTs.emplace_back();
            rejected.push_back(false);
            allObs.push_back(typeMap);
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
                if (task->stop) break;
                spp.oldVersion = obs.epoch < oldTime;
                spp.preprocess(obs);


                SppEpochData data;
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
                    const bool wasAtEnd = task->selectedEpoch == -1 || task->selectedEpoch == static_cast<int>(task->epochs.size()) - 1;
                    if (task->epochs.empty()) {
                        task->selectedEpoch = 0;
                    }
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
        if (hasError) {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "状态: %s", task->errorMsg.c_str());
            if (epochCount > 0) ImGui::SameLine();
        }

        if (isLoading && !hasError) {
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, 1.0f), "正在处理: 已解析 %d 个历元...", epochCount);
        } else if (epochCount > 0 && !hasError) {
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "处理完成: 共 %d 个历元", epochCount);
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
        float plotRegionH = epochCount > 0 ? 380.0f : 0.0f;
        float mainTableH = availY - plotRegionH - ImGui::GetStyle().ItemSpacing.y;
        if (mainTableH < 200.0f) mainTableH = 200.0f;

        if (ImGui::BeginTable("##main_split", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn("LeftPanel", ImGuiTableColumnFlags_WidthStretch, 0.6f);
            ImGui::TableSetupColumn("RightPanel", ImGuiTableColumnFlags_WidthStretch, 0.4f);

            ImGui::TableNextRow(ImGuiTableRowFlags_None, mainTableH);

            // ===== 左面板：卫星列表 (Master-Detail View) =====
            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("##sat_list_child");
            if (epochCount > 0 && selectedIdx >= 0) {
                SppEpochData curData;
                {
                    std::lock_guard lock(task->mutex);
                    curData = task->epochs[selectedIdx];
                }

                // 1. Master Table (Satellite Overview)
                ImGui::SeparatorText("卫星概览");
                if (ImGui::BeginTable("##sats_master", 9,
                                      ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_ScrollY, ImVec2(0, ImGui::GetContentRegionAvail().y * 0.8f))) {
                    ImGui::TableSetupColumn("序号", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableSetupColumn("卫星", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableSetupColumn("X坐标(m)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Y坐标(m)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Z坐标(m)", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("高度角(°)", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                    ImGui::TableSetupColumn("方位角(°)", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                    ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                    ImGui::TableSetupColumn("残差(m)", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                    ImGui::TableHeadersRow();

                    for (int i = 0; i < static_cast<int>(curData.satIds.size()); i++) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%d", i + 1);
                        ImGui::TableSetColumnIndex(1);

                        if (ImGui::Selectable(curData.satIds[i].toString().c_str(), task->selectedSatIdx == i,
                                              ImGuiSelectableFlags_SpanAllColumns)) {
                            task->selectedSatIdx = i;
                        }
                        auto showText = [](const double val) {
                            if (val != 0.0f) {
                                ImGui::Text("%.1f", val);
                            } else {
                                ImGui::TextUnformatted("未知");
                            }
                        };
                        ImGui::TableSetColumnIndex(2);
                        showText(curData.satPVTs[i].p[0]);
                        ImGui::TableSetColumnIndex(3);
                        showText(curData.satPVTs[i].p[1]);
                        ImGui::TableSetColumnIndex(4);
                        showText(curData.satPVTs[i].p[2]);
                        ImGui::TableSetColumnIndex(5);
                        showText(curData.elevations[i] * RAD_TO_DEG);
                        ImGui::TableSetColumnIndex(6);
                        showText(curData.azimuths[i] * RAD_TO_DEG);
                        ImGui::TableSetColumnIndex(7);
                        if (curData.rejected[i] || !curData.solved) ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "排除");
                        else ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "参与");

                        ImGui::TableSetColumnIndex(8);
                        if (auto it = curData.sppResult.postRes.find(curData.satIds[i]); it != curData.sppResult.postRes.end()) {
                            ImGui::Text("%.4f", it->second);
                        } else {
                            ImGui::TextDisabled("-");
                        }
                    }
                    ImGui::EndTable();
                }

                ImGui::Spacing();

                // 2. Detail Table (Frequency Observations)

                if (task->selectedSatIdx >= 0 && task->selectedSatIdx < static_cast<int>(curData.satIds.size())) {
                    ImGui::SeparatorText(("观测详情 (" + curData.satIds[task->selectedSatIdx].toString() + ")").c_str());
                    int satIdx = task->selectedSatIdx;
                    const auto &typeMap = curData.allObs[satIdx];

                    if (ImGui::BeginTable("##obs_detail", 5, ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                        ImGui::TableSetupColumn("频段", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                        ImGui::TableSetupColumn("伪距(m)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("载波(cycle)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("多普勒(Hz)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("强度(dB/Hz)", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableHeadersRow();

                        std::set<std::string> freqs;
                        for (auto const &[type, val]: typeMap) {
                            if (type.length() > 1) freqs.insert(type.substr(1));
                        }
                        if (freqs.empty()) freqs.insert("?");

                        for (const auto &f: freqs) {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            std::string freqLabel = f;
                            if (curData.satIds[satIdx].system == 'G') freqLabel = "L" + f;
                            else if (curData.satIds[satIdx].system == 'C') freqLabel = "B" + f;
                            ImGui::Text("%s", freqLabel.c_str());

                            ImGui::TableSetColumnIndex(1);
                            if (auto itC = typeMap.find("C" + f); itC != typeMap.end()) ImGui::Text("%.3f", itC->second);
                            else ImGui::TextDisabled("-");

                            ImGui::TableSetColumnIndex(2);
                            if (auto itL = typeMap.find("L" + f); itL != typeMap.end()) ImGui::Text("%.3f", itL->second);
                            else ImGui::TextDisabled("-");

                            ImGui::TableSetColumnIndex(3);
                            if (auto itD = typeMap.find("D" + f); itD != typeMap.end()) ImGui::Text("%.3f", itD->second);
                            else ImGui::TextDisabled("-");

                            ImGui::TableSetColumnIndex(4);
                            if (auto itS = typeMap.find("S" + f); itS != typeMap.end()) ImGui::Text("%.1f", itS->second);
                            else ImGui::TextDisabled("-");
                        }
                        ImGui::EndTable();
                    }
                } else {
                    ImGui::SeparatorText("请在上方列表中选择一颗卫星查看详情...");
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
                    auto &result = curData.sppResult;
                    auto enu = XYZtoENU(result.xyz, task->refECEF);
                    ImGui::SeparatorText("定位位置 (WGS84)");
                    ImGui::Text("  ECEF: (%.4f, %.4f, %.4f) m", result.xyz[0], result.xyz[1], result.xyz[2]);
                    ImGui::Text("    REF: (%.4f, %.4f, %.4f) m", task->refECEF[0], task->refECEF[1], task->refECEF[2]);
                    ImGui::Text("  ENU: (%.4f, %.4f, %.4f) m", enu[0], enu[1], enu[2]);
                    ImGui::Text("  BLH: (%.8f°, %.8f°, %.4f m)", result.blh[0] * RAD_TO_DEG, result.blh[1] * RAD_TO_DEG, result.blh[2]);
                    ImGui::Text("  SigmaP: (%.3f, %.3f, %.3f) m (%.3f m)", result.sigmaXYZ[0], result.sigmaXYZ[1], result.sigmaXYZ[2],
                                result.sigmaP);
                    ImGui::Spacing();
                    ImGui::SeparatorText("速度");
                    ImGui::Text("  (%.4f, %.4f, %.4f) m/s", result.vel[0], result.vel[1], result.vel[2]);
                    ImGui::Text("  SigmaV: (%.3f, %.3f, %.3f) m/s (%.3f m/s)", result.sigmaVel[0], result.sigmaVel[1], result.sigmaVel[2],
                                result.sigmaV);
                    ImGui::Spacing();
                    ImGui::SeparatorText("精度");
                    ImGui::Text("  PDOP:   %.2f", result.pdop);
                    ImGui::Text("  GDOP:   %.2f", result.gdop);
                    ImGui::Text("  HDOP:   %.2f", result.hdop);
                    ImGui::Text("  VDOP:   %.2f", result.vdop);
                    ImGui::Text("  TDOP:   %.2f", result.tdop);
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
                    const auto &result = ep.sppResult;
                    times[i] = i;
                    sigmaPs[i] = ep.solved ? result.sigmaP : 0.0;
                    sigmaVs[i] = ep.solved ? result.sigmaV : 0.0;
                    pdops[i] = ep.solved ? result.pdop : 0.0;
                    if (ep.solved) {
                        auto enu = XYZtoENU(result.xyz, task->refECEF);
                        enu_e[i] = enu[0];
                        enu_n[i] = enu[1];
                        enu_u[i] = enu[2];
                    } else {
                        enu_e[i] = 0;
                        enu_n[i] = 0;
                        enu_u[i] = 0;
                    }
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
        ofn.lpstrFilter = "OEM7 Log Files (*.log)\0*.log\0";
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
                << "VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";

        for (const auto &r: task->epochs) {
            out << r.week << ','
                    << std::fixed << std::setprecision(3) << r.sow << ',';

            if (r.solved) {
                auto &result = r.sppResult;
                auto enu = XYZtoENU(result.xyz, task->refECEF);
                out << std::setprecision(4)
                        << result.xyz[0] << ',' << result.xyz[1] << ',' << result.xyz[2] << ','
                        << task->refECEF.X() << ','
                        << task->refECEF.Y() << ','
                        << task->refECEF.Z() << ','
                        << enu.E() << ','
                        << enu.N() << ','
                        << enu.U() << ','
                        << std::setprecision(8)
                        << result.blh[0] * RAD_TO_DEG << ',' << result.blh[1] * RAD_TO_DEG << ','
                        << std::setprecision(3)
                        << result.blh[2] << ','
                        << result.vel[0] << ',' << result.vel[1] << ',' << result.vel[2] << ','
                        << std::setprecision(4)
                        << result.pdop << ',' << result.gdop << ',' << result.hdop << ',' << result.vdop << ',' << result.tdop << ','
                        << result.sigmaP << ',' << result.sigmaV << ','
                        << r.numSatsResult;
            } else {
                //out << ",,,,,,,,,,,,0";
            }
            out << '\n';
        }

        out.close();
    }
} // namespace GuiOem7Processor
