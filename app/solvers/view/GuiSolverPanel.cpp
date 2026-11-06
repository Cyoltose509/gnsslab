#include "ui/GuiColors.h"
#include <windows.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "GuiSolverPanel.h"
#include "imgui.h"
#include "StringUtils.h"
#include "Log.h"
#include "Const.h"
#include "QCProcessor.h"
#include "ui/FileDialog.h"
#include "solvers/view/GuiCharts.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiPosTab.h"
#include "solvers/view/QualityControl.h"

namespace Solvers::SolverPanel {
    void launchQC(View &view) {
        static std::mutex launchMutex;
        std::lock_guard lk(launchMutex);
        if (view.qcComputing.load()) return;
        if (view.qcWorker.joinable()) view.qcWorker.join();
        LOG_INFO << "[QC] 开始质量分析";
        view.qcComputing = true;
        view.qcWorker = std::thread([&view] {
            try {
                {
                    QualityReport rep = view.qcSnapshot();
                    std::lock_guard lk1(view.qcMutex);
                    view.qcReport = std::make_shared<QualityReport>(std::move(rep));
                }
                view.qcReady = true;
                LOG_INFO << "[QC] 质量分析结束";
            } catch (...) {
                view.qcReady = true;
            }
            view.qcComputing = false;
        });
    }

    static std::vector<std::pair<SatID, SkyPoint>> currentSkyPoints(
        Core &core, const int sel) {
        std::lock_guard lk(core.mutex);
        if (sel < 0 || sel >= static_cast<int>(core.epochs.size())) return {};
        auto &ep = core.epochs[sel];
        return Charts::buildCurSkyPoints(sel, ep.satIds.size(),
                                         [&](const std::size_t i) {
                                             Charts::SkyView v;
                                             v.sat = ep.satIds[i];
                                             v.azimRad = ep.azimuths[i];
                                             v.elevRad = ep.elevations[i];
                                             v.used = ep.solved && !ep.rejected[i];
                                             return v;
                                         }, 0.0);
    }

    static void fillStatusLine(Charts::PosTabView &v, const Core &core,
                               const bool hasError, const bool busy, const int epochCount) {
        if (hasError) {
            v.statusText = std::string("状态: ") + core.errorMessage();
            v.statusColor = UI::kColFail;
        } else if (epochCount > 0) {
            v.statusText = "解算：共 " + std::to_string(epochCount) + " 个历元"
                           + (busy ? "（进行中…）" : "");
            v.statusColor = UI::kColWarn;
        } else if (busy) {
            v.statusText = "正在读取观测 / 加载星历与产品…";
        } else {
            v.statusText = "处理结束：未解出任何历元，请检查观测文件与产品是否匹配";
            v.statusColor = UI::kColFail;
        }
    }

    static void fillCurrentEpoch(Charts::PosTabView &v, Core &core, const int selectedIdx) {
        std::lock_guard lk(core.mutex);
        auto &cur = core.epochs[selectedIdx];

        std::vector<int> visIdx;
        visIdx.reserve(static_cast<int>(cur.satIds.size()));
        for (int i = 0; i < static_cast<int>(cur.satIds.size()); ++i)
            if (core.opts.systems.count(cur.satIds[i].system)) visIdx.push_back(i);

        v.satRows = Charts::buildSatRows(visIdx.size(),
                                         [&](const std::size_t i) {
                                             const int oi = visIdx[i];
                                             Charts::SatRowView _v;
                                             _v.sat = cur.satIds[oi];
                                             _v.used = !cur.rejected[oi] && cur.solved;
                                             _v.elevRad = cur.elevations[oi];
                                             _v.azimRad = cur.azimuths[oi];
                                             const bool haveXYZ = cur.satPos[oi].squaredNorm() > 1.0;
                                             _v.hasXYZ = haveXYZ;
                                             if (haveXYZ) {
                                                 _v.x = cur.satPos[oi][0];
                                                 _v.y = cur.satPos[oi][1];
                                                 _v.z = cur.satPos[oi][2];
                                             }
                                             _v.codeRes = v.s.codeResidual(cur.satIds[oi], selectedIdx);
                                             _v.phaseRes = v.s.phaseResidual(cur.satIds[oi], selectedIdx);
                                             return _v;
                                         });
        v.hasSatRows = true;

        if (core.view.selectedSatIdx >= 0 && core.view.selectedSatIdx < static_cast<int>(visIdx.size())) {
            const int oi = visIdx[core.view.selectedSatIdx];
            v.showObsDetail = true;
            v.detailSystem = cur.satIds[oi].system;
            v.detailObs = cur.rawObs.of(static_cast<std::size_t>(oi));
        }

        if (cur.solved) {
            const auto &r = cur.sppResult;
            v.solved = true;
            v.pdop = r.pdop;
            v.gdop = r.gdop;
            v.hdop = r.hdop;
            v.vdop = r.vdop;
            v.tdop = r.tdop;
            v.numSatsResult = cur.numSatsResult;
            v.numObs = cur.numObs;

            const bool isRtkMode = core.mode == Mode::RTK;
            const bool isLeoMode = core.mode == Mode::LEO;
            const bool canFixed = isRtkMode || isLeoMode ||
                                  core.mode == Mode::PPPStatic ||
                                  core.mode == Mode::PPPKinematic;
            const bool isFixed = cur.isFixed(core.mode);
            v.hasFixed = isFixed;

            Eigen::Vector3d xyzFloat = r.xyzFloat;
            Eigen::Vector3d xyzFixed = r.xyzFixed.squaredNorm() > 1.0 ? r.xyzFixed : r.xyz;
            if (isRtkMode) {
                xyzFloat = cur.rtkFloatXyz;
                xyzFixed = cur.rtkStat == 1 ? r.xyz : cur.rtkFloatXyz;
            }
            if (xyzFloat.squaredNorm() < 1.0) xyzFloat = r.xyz;
            v.xyzFloatDisp = xyzFloat;
            v.xyzFixedDisp = xyzFixed;
            v.sigmaFloatDisp = r.sigmaPFloat > 0 ? r.sigmaPFloat : r.sigmaP;
            v.sigmaFixedDisp = r.sigmaPFixed > 0 ? r.sigmaPFixed : r.sigmaP;

            if (isRtkMode) {
                v.hasBaseline = true;
                v.baseFloatDisp = cur.rtkBaseFloat;
                if (cur.rtkStat == 1)
                    v.baseFixedDisp = cur.rtkFixedBaseline.norm() > 0
                                          ? cur.rtkFixedBaseline
                                          : r.xyz - core.refECEF;
            }

            if (r.vel.squaredNorm() > 1e-4) {
                v.hasVelDisp = true;
                v.velDisp = r.vel;
                v.sigmaVDisp = r.sigmaV;
            }

            v.showRefTable = canFixed && (!isLeoMode || !core.view.leoRefTraj.empty());
            v.isLeoRtn = isLeoMode;
            v.refDisp = core.refECEF;
            const Eigen::Vector3d xyzPrim = isFixed ? xyzFixed : xyzFloat;
            v.abs3dDisp = Coord::XYZToENU(xyzPrim, core.refECEF).norm();
            if (isLeoMode) {
                v.rtnDisp = Eigen::Vector3d(cur.rtnR, cur.rtnT, cur.rtnN);
                v.rtn3dDisp = std::sqrt(cur.rtnR * cur.rtnR + cur.rtnT * cur.rtnT + cur.rtnN * cur.rtnN);
            } else {
                v.enuFloatDisp = Coord::XYZToENU(xyzFloat, core.refECEF);
                v.enuFixedDisp = Coord::XYZToENU(xyzFixed, core.refECEF);
                v.enuFloat3d = v.enuFloatDisp.norm();
                v.enuFixed3d = v.enuFixedDisp.norm();
            }

            v.solStatShort = isFixed ? "固定(Fixed)" : "浮点(Float)";
            v.solveStatusColor = isFixed ? UI::kColPass : UI::kColWarn;
            v.solRatio = isRtkMode ? cur.rtkRatio : r.ratio;
            v.showSolveStatus = canFixed;
        } else {
            v.noSolveMsg = core.noEphSolve ? "无定位解（缺少星历文件）" : "无定位解";
        }

        char buf[160];
        std::snprintf(buf, sizeof buf, "Wk %u SOW %.3f | %s", cur.week, cur.sow, cur.solved ? "定位" : "无解");
        v.epochInfo = buf;
    }

    static void fillPlots(Charts::PosTabView &v, Core &core,
                          const int selectedIdx, const int epochCount, const bool isDone) {
        {
            std::lock_guard plk(core.view.plotMutex);
            core.view.plotSeries.sync(core.view.plotData);
        }
        auto &s = core.view.plotSeries;
        if (isDone && !s.hasResRange) {
            const auto [lo, hi] = Helpers::computeRobustResRange(s.data.satResVals);
            s.resYlo = lo;
            s.resYhi = hi;
            s.hasResRange = true;
        }
        if (isDone && !s.hasResPhaseRange) {
            const auto [lo, hi] = Helpers::computeRobustResRange(s.data.satResPhaseVals, 0.01);
            s.resPhaseYlo = lo;
            s.resPhaseYhi = hi;
            s.hasResPhaseRange = true;
        }

        v.showSigmaDop = true;
        v.showSigmaV = s.data.solvedEpochs > 0 &&
                       static_cast<double>(s.data.velEpochs) / s.data.solvedEpochs > 0.05 &&
                       s.data.sigmaVNonZero;
        v.hasFloatSeries = core.mode != Mode::SPP;
        v.isRtk = core.mode == Mode::RTK;
        v.curSkyPts = currentSkyPoints(core, selectedIdx);

        if (epochCount > 0 && s.data.solvedEpochs > 0) {
            v.showFixRate = true;
            char buf[64];
            std::snprintf(buf, sizeof buf, "固定率: %.1f%% (%d/%d)",
                          100.0 * s.data.fixedEpochs / s.data.solvedEpochs,
                          s.data.fixedEpochs, s.data.solvedEpochs);
            v.fixRateText = buf;
        }
    }

    static void fillLEO3D(Charts::PosTabView &v, Core &core) {
        std::lock_guard lk(core.mutex);
        if (!core.view.hasLeo) return;
        v.show3d = true;
        v.showRtn = !core.view.leoRefTraj.empty();
        v.showRefEnu = false;
        v.leoPos3d = core.view.leoPos3d;
        copySampled(core.view.leoTraj, v.leoTraj);
        copySampled(core.view.leoRefTraj, v.leoRefTraj);
        copySampled(core.view.rtnR, v.rtnR);
        copySampled(core.view.rtnT, v.rtnT);
        copySampled(core.view.rtnN, v.rtnN);
        copySampled(core.view.rtnD3, v.rtnD3);
        copySampled(core.view.rtnTimes, v.rtnTimes);
    }



    void renderTask(const std::shared_ptr<Task> &job) {
        if (job->state == Task::State::Config) {
            job->renderConfigPanel();
            return;
        }

        job->renderResult();
    }

    void exportCSV(const std::shared_ptr<Task> &task) {
        const std::string tag = modeLabel(task->mode);
        std::string dn = Str::stripExt(Str::baseName(task->fileName));
        dn += "_" + tag + "_result.csv";
        if (dn.size() >= MAX_PATH) dn = tag + "_result.csv";
        std::wstring path;
        if (!FileDialog::saveCSV(dn, path)) return;
        LOG_INFO << "[导出] CSV (" << tag << ") → "
                << std::filesystem::path(path).string();

        std::string csvText;
        {
            std::lock_guard lk(task->mutex);
            std::ostringstream oss;
            task->exportCSV(oss);
            csvText = oss.str();
        }
        std::ofstream out(std::filesystem::path(path), std::ios::out | std::ios::binary);
        if (!out) {
            LOG_WARN << "[导出] 写入失败: " << std::filesystem::path(path).string();
            return;
        }
        out << csvText;
        if (!out)
            LOG_WARN << "[导出] 写入失败: " << std::filesystem::path(path).string();
    }
}

namespace Solvers {
    void Task::renderResult() {
        const bool isRealtime = this->isRealtime;

        if (isRealtime) {
            if (!done) {
                if (ImGui::Button("停止连接")) cancel();
                ImGui::SameLine();
            } else {
                ImGui::TextColored(UI::kColFail, "连接已断开");
                ImGui::SameLine();
            }
        }

        const bool isDone = done.load();
        const bool hasError = errorFlag();

        const bool busy = !isDone && !hasError;
        auto [epochCount, selectedIdx] = readEpochView();
        int selectedEpochLocal = selectedIdx;

        if (!isRealtime && busy) {
            ImGui::TextColored(UI::kColInfo,
                               "正在处理 [%s]: 已解算 %d 历元",
                               fileName.c_str(), epochCount);
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem("定位解算")) {
                Charts::PosTabView v(view.plotSeries, view.skyTracks);
                v.epochCount = epochCount;
                v.selectedEpoch = &selectedEpochLocal;
                v.selectedSatIdx = &view.selectedSatIdx;
                v.resMode = &view.resMode;
                v.refECEF = &refECEF;
                v.onRefChanged = [&] {
                    XYZ ref;
                    {
                        std::lock_guard lk(mutex);
                        ref = refECEF;
                    }
                    std::lock_guard plk(view.plotMutex);
                    view.plotData.refreshENU(ref);
                };
                v.showEnPlan = mode != Mode::LEO;

                SolverPanel::fillStatusLine(v, *this, hasError, busy, epochCount);
                v.busy = busy;

                if (epochCount > 0) {
                    std::lock_guard lk(mutex);
                    view.ensureSkyTracks(epochs);
                }
                if (epochCount > 0 && selectedIdx >= 0) SolverPanel::fillCurrentEpoch(v, *this, selectedIdx);
                if (epochCount > 0 && selectedIdx >= 0) SolverPanel::fillPlots(v, *this, selectedIdx, epochCount, isDone);
                SolverPanel::fillLEO3D(v, *this);

                v.onExportCSV = [&] {
                    SolverPanel::exportCSV(shared_from_this());
                };

                try {
                    Charts::renderPositioningTab(v);
                } catch (const std::exception &e) {
                    ImGui::TextColored(UI::kColFail, "渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(UI::kColFail, "渲染发生未知异常");
                }
                if (selectedEpochLocal != selectedIdx) {
                    std::lock_guard lk(mutex);
                    if (selectedEpochLocal >= 0 && selectedEpochLocal < static_cast<int>(epochs.size()))
                        view.selectedEpoch = selectedEpochLocal;
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(view.qcTabLabel.c_str())) {
                try {
                    QualityControl::render(view, fileName);
                } catch (const std::exception &e) {
                    ImGui::TextColored(UI::kColFail, "质量分析渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(UI::kColFail, "质量分析渲染发生未知异常");
                }
                ImGui::EndTabItem();
            }
            for (const auto &tab: view.extraQcTabs) {
                if (!tab) continue;
                if (ImGui::BeginTabItem(tab->label.c_str())) {
                    try {
                        QualityControl::render(tab->view, tab->fileName);
                    } catch (const std::exception &e) {
                        ImGui::TextColored(UI::kColFail, "质量分析渲染异常: %s", e.what());
                    } catch (...) {
                        ImGui::TextColored(UI::kColFail, "质量分析渲染发生未知异常");
                    }
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
    }
}
