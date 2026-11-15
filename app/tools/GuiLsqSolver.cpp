#include "GuiLsqSolver.h"
#include "imgui.h"
#include "ui/GuiColors.h"
#include "SolverLSQ.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "i18n/I18n.h"

namespace Tools::LsqSolver {
    static int s_NumObs = 4;
    static int s_NumUnk = 3;
    static std::vector<double> s_H;
    static std::vector<double> s_L;
    static std::vector<double> s_W;

    static Eigen::VectorXd s_X;
    static Eigen::MatrixXd s_Q;
    static double s_Sigma0 = 0;
    static std::string s_Error;

    static bool s_AutoSolve = true;
    static bool s_Initialized = false;
    static bool s_Solved = false;

    static void ResizeMatrices(int newObs, int newUnk) {
        // 上限 64：法方程矩阵是 m×m 与 n×n，手滑输个 100000 会直接吃掉几十 GB
        newObs = std::clamp(newObs, 1, 64);
        newUnk = std::clamp(newUnk, 1, 64);


        const std::vector<double> oldH = s_H;
        const std::vector<double> oldL = s_L;
        const std::vector<double> oldW = s_W;
        const int oldObs = s_NumObs;
        const int oldUnk = s_NumUnk;


        const bool hasOldData = !oldH.empty() && !oldL.empty() && !oldW.empty();

        s_NumObs = newObs;
        s_NumUnk = newUnk;


        s_H.assign(s_NumObs * s_NumUnk, 0.0);
        if (hasOldData) {
            for (int i = 0; i < std::min(oldObs, s_NumObs); ++i) {
                for (int j = 0; j < std::min(oldUnk, s_NumUnk); ++j) {
                    s_H[i * s_NumUnk + j] = oldH[i * oldUnk + j];
                }
            }
        }


        s_L.assign(s_NumObs, 0.0);
        if (hasOldData) {
            for (int i = 0; i < std::min(oldObs, s_NumObs); ++i) {
                s_L[i] = oldL[i];
            }
        }


        s_W.assign(s_NumObs * s_NumObs, 0.0);
        if (!hasOldData) {

            for (int i = 0; i < s_NumObs; ++i) s_W[i * s_NumObs + i] = 1.0;
        } else {
            for (int i = 0; i < std::min(oldObs, s_NumObs); ++i) {
                for (int j = 0; j < std::min(oldObs, s_NumObs); ++j) {
                    s_W[i * s_NumObs + j] = oldW[i * oldObs + j];
                }
            }

            if (s_NumObs > oldObs) {
                for (int i = oldObs; i < s_NumObs; ++i) s_W[i * s_NumObs + i] = 1.0;
            }
        }

        s_X = Eigen::VectorXd::Zero(s_NumUnk);
        s_Q = Eigen::MatrixXd::Zero(s_NumUnk, s_NumUnk);
        s_Sigma0 = 0;
        s_Solved = false;
        s_Error.clear();
    }

    static void Solve() {
        if (s_NumObs < s_NumUnk) {
            s_Error = I18n::tr("tool.lsq.err_obs_lt_unk");
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

            if (varList.size() != static_cast<std::size_t>(s_NumUnk)) {
                s_Error = I18n::tr("tool.lsq.err_var_fail");
                return;
            }

            Eigen::MatrixXd W(s_NumObs, s_NumObs);
            for (int i = 0; i < s_NumObs; ++i)
                for (int j = 0; j < s_NumObs; ++j)
                    W(i, j) = s_W[i * s_NumObs + j];

            for (int i = 0; i < s_NumObs; ++i) {
                EquData data;
                data.prefit = s_L[i];
                for (int j = 0; j < s_NumUnk; ++j) {
                    data.varCoeffData[varList[j]] = s_H[i * s_NumUnk + j];
                }
                sys.obsEquData[EquID(SatID('G', i), "GUI")] = data;
            }

            SolverLSQ solver;
            solver.solve(sys, W);

            s_X = solver.state;
            s_Q = solver.Q;
            s_Sigma0 = solver.sigma0;
            // 法方程奇异时 LSQ 会推出 inf/nan，界面上得说清楚，别让人当成有效解
            if (!s_X.allFinite() || !s_Q.allFinite() || !std::isfinite(s_Sigma0)) {
                s_Error = I18n::tr("tool.lsq.err_singular");
                s_Solved = false;
            } else {
                s_Solved = true;
            }
        } catch (const std::exception &e) {
            s_Error = e.what();
        } catch (...) {
            s_Error = I18n::tr("tool.lsq.err_failed");
        }
    }

    void Render(bool *p_open) {
        if (!s_Initialized) {
            s_Initialized = true;
            ResizeMatrices(4, 3);
        }

        ImGui::SetNextWindowSize(ImVec2(1000, 750), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin(I18n::trc("tool.lsq.title"), p_open)) {
            ImGui::End();
            return;
        }


        ImGui::BeginChild("Toolbar", ImVec2(0, 45), true);
        ImGui::AlignTextToFramePadding();

        ImGui::TextUnformatted(I18n::tr("tool.lsq.num_obs", s_NumObs).c_str());
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        int nObs = s_NumObs;
        if (ImGui::InputInt("##m_input", &nObs)) ResizeMatrices(nObs, s_NumUnk);

        ImGui::SameLine();
        ImGui::TextUnformatted(I18n::tr("tool.lsq.num_unk", s_NumUnk).c_str());
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        int nUnk = s_NumUnk;
        if (ImGui::InputInt("##n_input", &nUnk)) ResizeMatrices(s_NumObs, nUnk);

        ImGui::SameLine(0, 30);
        ImGui::Checkbox(I18n::trc("tool.lsq.auto_solve"), &s_AutoSolve);
        ImGui::SameLine();
        if (ImGui::Button(I18n::trc("tool.lsq.solve"), ImVec2(80, 0))) Solve();
        ImGui::SameLine();
        if (ImGui::Button(I18n::trc("common.clear"), ImVec2(60, 0))) {
            std::fill(s_H.begin(), s_H.end(), 0.0);
            std::fill(s_L.begin(), s_L.end(), 0.0);
            std::fill(s_W.begin(), s_W.end(), 0.0);
            for (int i = 0; i < s_NumObs; ++i) s_W[i * s_NumObs + i] = 1.0;
            if (s_AutoSolve) Solve();
        }
        ImGui::EndChild();

        ImGui::Spacing();


        bool changed = false;
        ImGui::SeparatorText(I18n::trc("tool.lsq.model"));

        if (ImGui::BeginTable("##math_layout", 2,
                              ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupColumn("L_col", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("H_col", ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableNextRow();


            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(I18n::tr("tool.lsq.shape_l", s_NumObs).c_str());
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



            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(I18n::tr("tool.lsq.shape_h", s_NumObs, s_NumUnk).c_str());
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
        ImGui::SeparatorText(I18n::trc("tool.lsq.weight"));

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
        ImGui::SeparatorText(I18n::trc("pos.section.result"));

        if (!s_Error.empty()) {
            ImGui::TextColored(UI::kColFail, "%s", I18n::tr("common.status", s_Error).c_str());
        } else if (!s_Solved) {
            ImGui::TextDisabled(I18n::trc("tool.lsq.not_solved"));
        } else {
            char sigma[32];
            std::snprintf(sigma, sizeof(sigma), "%.6f", s_Sigma0);
            ImGui::TextUnformatted(I18n::tr("tool.lsq.sigma0", sigma).c_str());
            if (ImGui::BeginTable("##result", 2,
                                  ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_Resizable)) {
                ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch,0.3);
                ImGui::TableSetupColumn("Q", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                if (ImGui::BeginTable("##X_res", 2, ImGuiTableFlags_Borders)) {
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
