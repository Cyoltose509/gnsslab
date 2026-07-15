#include "GuiLsqSolver.h"
#include "imgui.h"
#include "SolverLSQ.h"
#include <vector>
#include <string>
#include <algorithm>

namespace GuiLsqSolver {
    static int s_NumObs = 4;
    static int s_NumUnk = 3;
    static std::vector<double> s_H; // m * n
    static std::vector<double> s_L; // m
    static std::vector<double> s_W; // m * m (Full weight matrix)

    static VectorXd s_X;
    static MatrixXd s_Q;
    static double s_Sigma0 = 0;
    static std::string s_Error;

    static bool s_AutoSolve = true;
    static bool s_Initialized = false;

    static void ResizeMatrices(int newObs, int newUnk) {
        if (newObs < 1) newObs = 1;
        if (newUnk < 1) newUnk = 1;

        // Backup old data
        const std::vector<double> oldH = s_H;
        const std::vector<double> oldL = s_L;
        const std::vector<double> oldW = s_W;
        const int oldObs = s_NumObs;
        const int oldUnk = s_NumUnk;

        // On first run, old vectors are empty. We should handle this.
        const bool hasOldData = !oldH.empty() && !oldL.empty() && !oldW.empty();

        s_NumObs = newObs;
        s_NumUnk = newUnk;

        // Resize H (m x n)
        s_H.assign(s_NumObs * s_NumUnk, 0.0);
        if (hasOldData) {
            for (int i = 0; i < std::min(oldObs, s_NumObs); ++i) {
                for (int j = 0; j < std::min(oldUnk, s_NumUnk); ++j) {
                    s_H[i * s_NumUnk + j] = oldH[i * oldUnk + j];
                }
            }
        }

        // Resize L (m x 1)
        s_L.assign(s_NumObs, 0.0);
        if (hasOldData) {
            for (int i = 0; i < std::min(oldObs, s_NumObs); ++i) {
                s_L[i] = oldL[i];
            }
        }

        // Resize W (m x m)
        s_W.assign(s_NumObs * s_NumObs, 0.0);
        if (!hasOldData) {
            // Initial identity
            for (int i = 0; i < s_NumObs; ++i) s_W[i * s_NumObs + i] = 1.0;
        } else {
            for (int i = 0; i < std::min(oldObs, s_NumObs); ++i) {
                for (int j = 0; j < std::min(oldObs, s_NumObs); ++j) {
                    s_W[i * s_NumObs + j] = oldW[i * oldObs + j];
                }
            }
            // If expanded, set new diagonal to 1.0
            if (s_NumObs > oldObs) {
                for (int i = oldObs; i < s_NumObs; ++i) s_W[i * s_NumObs + i] = 1.0;
            }
        }

        s_X = VectorXd::Zero(s_NumUnk);
        s_Q = MatrixXd::Zero(s_NumUnk, s_NumUnk);
        s_Sigma0 = 0;
    }

    static void Solve() {
        if (s_NumObs < s_NumUnk) {
            s_Error = "观测数 (m) 必须 >= 未知数 (n)";
            return;
        }
        try {
            s_Error.clear();
            EquSys sys;
            sys.station = "GUI";

            VariableSet vars;
            for (int j = 0; j < s_NumUnk; ++j) {
                Variable v("GUI", SatID('G', j), Parameter(Parameter::Unknown), ObsID("G", "GUI"));
                vars.insert(v);
            }
            sys.varSet = vars;

            std::vector<Variable> varList;
            varList.assign(sys.varSet.begin(), sys.varSet.end());

            if (varList.size() != static_cast<size_t>(s_NumUnk)) {
                s_Error = "内部错误: 变量生成失败";
                return;
            }

            for (int i = 0; i < s_NumObs; ++i) {
                EquData data;
                data.prefit = s_L[i];
                data.weight = s_W[i * s_NumObs + i];
                for (int j = 0; j < s_NumUnk; ++j) {
                    data.varCoeffData[varList[j]] = s_H[i * s_NumUnk + j];
                }
                sys.obsEquData[EquID(SatID('G', i), "GUI")] = data;
            }

            SolverLSQ solver;
            solver.solve(sys);

            s_X = solver.state;
            s_Q = solver.covMatrix;
            s_Sigma0 = solver.sigma0;
        } catch (const std::exception &e) {
            s_Error = e.what();
        } catch (...) {
            s_Error = "解算失败 (请检查矩阵是否满秩)";
        }
    }

    void Render(bool *p_open) {
        if (!s_Initialized) {
            s_Initialized = true;
            ResizeMatrices(4, 3);
        }

        ImGui::SetNextWindowSize(ImVec2(1000, 750), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("最小二乘工具", p_open)) {
            ImGui::End();
            return;
        }

        // --- Toolbar ---
        ImGui::BeginChild("Toolbar", ImVec2(0, 45), true);
        ImGui::AlignTextToFramePadding();

        ImGui::Text("观测数 (%d):", s_NumObs);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        int nObs = s_NumObs;
        if (ImGui::InputInt("##m_input", &nObs) && nObs >= 1) ResizeMatrices(nObs, s_NumUnk);

        ImGui::SameLine();
        ImGui::Text("未知数 (%d):", s_NumUnk);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        int nUnk = s_NumUnk;
        if (ImGui::InputInt("##n_input", &nUnk) && nUnk >= 1) ResizeMatrices(s_NumObs, nUnk);

        ImGui::SameLine(0, 30);
        ImGui::Checkbox("自动计算", &s_AutoSolve);
        ImGui::SameLine();
        if (ImGui::Button("计算", ImVec2(80, 0))) Solve();
        ImGui::SameLine();
        if (ImGui::Button("清空", ImVec2(60, 0))) {
            std::fill(s_H.begin(), s_H.end(), 0.0);
            std::fill(s_L.begin(), s_L.end(), 0.0);
            std::fill(s_W.begin(), s_W.end(), 0.0);
            for (int i = 0; i < s_NumObs; ++i) s_W[i * s_NumObs + i] = 1.0;
            if (s_AutoSolve) Solve();
        }
        ImGui::EndChild();

        ImGui::Spacing();

        // --- Layout: L = H * X ---
        bool changed = false;
        ImGui::SeparatorText("方程模型: Z = H * X + △");

        if (ImGui::BeginTable("##math_layout", 2,
                              ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupColumn("L_col", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("H_col", ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableNextRow();

            // L column
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("L (%d x 1)", s_NumObs);
            if (ImGui::BeginTable("##L_table", 1, ImGuiTableFlags_Borders)) {
                for (int i = 0; i < s_NumObs; ++i) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushID(100 + i);
                    ImGui::SetNextItemWidth(-1);
                    if (ImGui::InputDouble("##li", &s_L[i], 0, 0, "%.3f")) changed = true;
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }


            // H matrix
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("H (%d x %d)", s_NumObs, s_NumUnk);
            if (ImGui::BeginTable("##H_table", s_NumUnk, ImGuiTableFlags_Borders)) {
                for (int i = 0; i < s_NumObs; ++i) {
                    ImGui::TableNextRow();
                    for (int j = 0; j < s_NumUnk; ++j) {
                        ImGui::TableSetColumnIndex(j);
                        ImGui::PushID(1000 + i * 100 + j);
                        ImGui::SetNextItemWidth(-1);
                        if (ImGui::InputDouble("##hij", &s_H[i * s_NumUnk + j], 0, 0, "%.3f")) changed = true;
                        ImGui::PopID();
                    }
                }
                ImGui::EndTable();
            }
            ImGui::EndTable();
        }

        ImGui::Spacing();
        ImGui::SeparatorText("权阵 W (m x m)");

        if (ImGui::BeginTable("##W_table", s_NumObs, ImGuiTableFlags_Resizable)) {
            for (int i = 0; i < s_NumObs; ++i) {
                ImGui::TableNextRow();
                for (int j = 0; j < s_NumObs; ++j) {
                    ImGui::TableSetColumnIndex(j);
                    ImGui::PushID(5000 + i * 100 + j);
                    ImGui::SetNextItemWidth(-1);
                    if (i == j) ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.1f, 0.3f, 0.1f, 1.0f));
                    if (ImGui::InputDouble("##wij", &s_W[i * s_NumObs + j], 0, 0, "%.2f")) changed = true;
                    if (i == j) ImGui::PopStyleColor();
                    ImGui::PopID();
                }
            }
            ImGui::EndTable();
        }

        if (s_AutoSolve && changed) Solve();

        ImGui::Spacing();
        ImGui::SeparatorText("解算结果");

        if (!s_Error.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "状态: %s", s_Error.c_str());
        } else {
            ImGui::Text("单位权中误差 (Sigma0): %.6f", s_Sigma0);
            if (ImGui::BeginTabBar("##ResultTabs")) {
                ImGui::EndTabBar();
            }
            if (ImGui::BeginTable("##result", 2,
                                  ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_Resizable)) {
                ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch,0.3);
                ImGui::TableSetupColumn("Q", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                if (ImGui::BeginTable("##X_res", 2, ImGuiTableFlags_Borders)) {
                    //ImGui::TableSetupColumn("参数", ImGuiTableColumnFlags_WidthFixed, 80);
                    //ImGui::TableSetupColumn("估值");
                    ImGui::TableNextRow();
                    for (int j = 0; j < s_NumUnk; ++j) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("x%d", j + 1);
                        ImGui::TableSetColumnIndex(1);
                        if (j < s_X.size()) ImGui::Text("%.7f", s_X(j));
                        else ImGui::Text("-");
                    }
                    ImGui::EndTable();
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::BeginTable("##Q_res", s_NumUnk, ImGuiTableFlags_Borders)) {
                    for (int i = 0; i < s_NumUnk; ++i) {
                        ImGui::TableNextRow();
                        for (int j = 0; j < s_NumUnk; ++j) {
                            ImGui::TableSetColumnIndex(j);
                            if (i < s_Q.rows() && j < s_Q.cols()) ImGui::Text("%.4f", s_Q(i, j));
                            else ImGui::Text("-");
                        }
                    }
                    ImGui::EndTable();
                }
                ImGui::EndTable();
            }
        }

        ImGui::End();
    }
}
