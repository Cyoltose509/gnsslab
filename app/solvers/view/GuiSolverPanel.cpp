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
#include "core/geom/CoordConvert.h"
#include "Log.h"
#include "core/Const.h"
#include "QCProcessor.h"
#include "ui/FileDialog.h"
#include "solvers/view/GuiCharts.h"
#include "solvers/view/GuiExport.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiPosTab.h"
#include "solvers/view/QualityControl.h"
#include "download/ui/Widgets.h"
#include "i18n/I18n.h"

namespace Solvers::SolverPanel {
    void launchQC(View &view) {
        static std::mutex launchMutex;
        std::lock_guard lk(launchMutex);
        if (view.qcComputing.load()) return;
        if (view.qcWorker.joinable()) view.qcWorker.join();
        LOG_INFO << I18n::trc("log.qc.start");
        view.qcComputing = true;
        view.qcWorker = std::thread([&view] {
            try {
                {
                    QualityReport rep = view.qcSnapshot();
                    std::lock_guard lk1(view.qcMutex);
                    view.qcReport = std::make_shared<QualityReport>(std::move(rep));
                }
                view.qcReady = true;
                LOG_INFO << I18n::trc("log.qc.done");
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
                               const bool hasError, const bool busy, const int epochCount,
                               const std::string &liveMsg) {
        if (hasError) {
            v.statusText = I18n::tr("common.status", core.errorMessage());
            v.statusColor = UI::kColFail;
        } else if (!liveMsg.empty()) {
            // 解算进行中：把逐帧推进的进度放到状态行，并带总历元数。
            v.statusText = liveMsg;
            if (epochCount > 0) v.statusText += I18n::tr("solver.epochs_paren", epochCount);
            v.statusColor = UI::kColInfo;
        } else if (epochCount > 0) {
            v.statusText = I18n::tr("solver.solve_epochs", epochCount)
                           + (busy ? I18n::tr("solver.in_progress") : std::string());
            v.statusColor = UI::kColWarn;
        } else if (busy) {
            v.statusText = I18n::trc("solver.reading");
        } else {
            v.statusText = I18n::trc("solver.no_epoch_solved");
            v.statusColor = UI::kColFail;
        }
    }

    static void fillCurrentEpoch(Charts::PosTabView &v, Core &core, const int selectedIdx) {
        std::lock_guard lk(core.mutex);
        std::lock_guard plk(core.view.plotMutex);
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
                                             _v.codeRes = core.view.plotData.codeResidual(cur.satIds[oi], selectedIdx);
                                             _v.phaseRes = core.view.plotData.phaseResidual(cur.satIds[oi], selectedIdx);
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
            const bool canFixed = Solvers::modeFixesAmbiguities(core.mode);
            const bool isFixed = canFixed && cur.isFixed(core.mode);
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

            v.velDisp = r.vel;
            v.sigmaVDisp = r.sigmaV;

            // 有没有参考由调用方按 exportRefReady() 裁：无真值时 core.refECEF 是首历元自己的
            // 位置，拿来当参考会显示一堆「看着像零残差」的 ENU。RTK 例外：它的参考是基准站
            // 坐标（基线行已覆盖相对基准的信息），参考表只在挂了真值时才有意义。
            v.showRefTable = isRtkMode ? core.truthReady()
                                       : (!isLeoMode || !core.view.leoRefTraj.empty());
            v.isLeoRtn = isLeoMode;
            // RTK 挂了真值时逐历元参考在 ep.refECEF（与导出 REF 列同口径），否则用任务级参考。
            const Eigen::Vector3d dispRef = isRtkMode && cur.refECEF.squaredNorm() > 1.0
                                                ? cur.refECEF
                                                : core.refECEF;
            v.refDisp = dispRef;
            const Eigen::Vector3d xyzPrim = isFixed ? xyzFixed : xyzFloat;
            v.abs3dDisp = Coord::XYZToENU(xyzPrim, dispRef).norm();
            if (isLeoMode) {
                v.rtnDisp = Eigen::Vector3d(cur.rtnR, cur.rtnT, cur.rtnN);
                v.rtn3dDisp = std::sqrt(cur.rtnR * cur.rtnR + cur.rtnT * cur.rtnT + cur.rtnN * cur.rtnN);
            } else {
                v.enuFloatDisp = Coord::XYZToENU(xyzFloat, dispRef);
                v.enuFixedDisp = Coord::XYZToENU(xyzFixed, dispRef);
                v.enuFloat3d = v.enuFloatDisp.norm();
                v.enuFixed3d = v.enuFixedDisp.norm();
            }

            v.solStatShort = isFixed ? I18n::trc("solver.fixed") : I18n::trc("solver.float");
            v.solveStatusColor = isFixed ? UI::kColPass : UI::kColWarn;
            v.solRatio = isRtkMode ? cur.rtkRatio : r.ratio;
            v.showSolveStatus = canFixed;

        } else {
            v.noSolveMsg = core.noEphSolve ? I18n::trc("solver.no_solution_no_eph") : I18n::trc("solver.no_solution");
        }

        char buf[160];
        std::snprintf(buf, sizeof buf, "Wk %u SOW %.3f | %s", cur.week, cur.sow, cur.solved ? I18n::trc("solver.state_solved") : I18n::trc("solver.state_unsolved"));
        v.epochInfo = buf;
    }

    static void fillPlots(Charts::PosTabView &v, Core &core,
                          const int selectedIdx, const int epochCount, const bool isDone) {
        {
            std::lock_guard plk(core.view.plotMutex);
            core.view.plotSeries.sync(core.view.plotData);
            // 静态解画 ENU 误差散点，动态解画轨迹；切口径会重建散点，故放在画图之前。
            core.view.plotSeries.setPlanTrack(!core.enPlanStatic());
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

        if (modeFixesAmbiguities(core.mode) && epochCount > 0 && s.data.solvedEpochs > 0) {
            v.showFixRate = true;
            char buf[64];
            std::snprintf(buf, sizeof buf, I18n::trc("solver.fix_rate"),
                          100.0 * s.data.fixedEpochs / s.data.solvedEpochs,
                          s.data.fixedEpochs, s.data.solvedEpochs);
            v.fixRateText = buf;
        }
    }

    static void fillLEO3D(Charts::PosTabView &v, Core &core) {
        std::lock_guard lk(core.mutex);
        // LEO 三维轨迹只给 LEO 模式：PPP/SPP/RTK 即便误置了 hasLeo 也绝不能画轨迹图、吞掉 ENU。
        if (!core.view.hasLeo || core.mode != Mode::LEO) return;
        v.show3d = true;
        v.showRtn = !core.view.rtnTimes.empty();
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

    void openExport(const std::shared_ptr<Task> &task) {
        ExportUI::open(task);
    }
}

namespace Solvers {
    void Task::renderResult() {
        const bool isRealtime = this->isRealtime;

        if (isRealtime) {
            if (!done) {
                if (ImGui::Button(I18n::trc("solver.stop_connection"))) cancel();
                ImGui::SameLine();
            } else {
                ImGui::TextColored(UI::kColFail, I18n::trc("solver.disconnected"));
                ImGui::SameLine();
            }
        }

        const bool isDone = done.load();
        const bool hasError = errorFlag();

        const bool busy = !isDone && !hasError;
        auto [epochCount, selectedIdx] = readEpochView();
        int selectedEpochLocal = selectedIdx;

        // 解算进度文案统一落到结果页状态行（箭头所指「解算：共 N 个历元」处），逐帧更新更有“正在算”的感觉。
        std::string liveMsg;
        if (!isRealtime && busy) {
            const int cur = solveEpoch.load();
            const auto phase = solvePhase.load();
            // 前向完成、后向解算器正在重新初始化（重载星历/格网）时 solveEpoch 归零：明确提示，别让人以为卡死。
            if (cur <= 0 && phase == Core::SolvePhase::Backward)
                liveMsg = I18n::trc("solver.forward_done");
            else if (cur <= 0)
                liveMsg = I18n::trc("solver.reading");
            else if (phase == Core::SolvePhase::Scan)
                liveMsg = I18n::tr("solver.scanning_epoch", cur);
            else if (phase == Core::SolvePhase::Backward)
                liveMsg = I18n::tr("solver.reverse_epoch", cur);
            else
                liveMsg = I18n::tr("solver.solving_epoch", cur);
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem(I18n::trc("solver.tab.position"))) {
                Charts::PosTabView v(view.plotSeries, view.skyTracks);
                v.epochCount = epochCount;
                if (modeHasFilter(mode))
                    v.filterDirectionText = I18n::tr("solver.filter_direction", filterLabel(opts.filter));
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
                v.enPlanTrack = !enPlanStatic();
                v.onPlanOriginChanged = [&](const Eigen::Vector3d &dEnu) {
                    std::lock_guard plk(view.plotMutex);
                    view.plotSeries.setPlanOrigin(Coord::ENUToXYZ(dEnu, view.plotSeries.planOrigin));
                };

                SolverPanel::fillStatusLine(v, *this, hasError, busy, epochCount, liveMsg);
                v.busy = busy;
                v.solvingEpoch = busy && !isRealtime ? solveEpoch.load() - 1 : -1;

                if (epochCount > 0) {
                    std::lock_guard lk(mutex);
                    view.ensureSkyTracks(epochs);
                }
                if (epochCount > 0 && selectedIdx >= 0) {
                    SolverPanel::fillPlots(v, *this, selectedIdx, epochCount, isDone);
                    SolverPanel::fillCurrentEpoch(v, *this, selectedIdx);
                }
                v.showRefTable = v.showRefTable && truthReady();
                SolverPanel::fillLEO3D(v, *this);

                v.onExport = [&] {
                    SolverPanel::openExport(shared_from_this());
                };

                try {
                    Charts::renderPositioningTab(v);
                } catch (const std::exception &e) {
                    ImGui::TextColored(UI::kColFail, I18n::trc("solver.render_error"), e.what());
                } catch (...) {
                    ImGui::TextColored(UI::kColFail, I18n::trc("solver.render_unknown"));
                }
                if (selectedEpochLocal != selectedIdx) {
                    std::lock_guard lk(mutex);
                    if (selectedEpochLocal >= 0 && selectedEpochLocal < static_cast<int>(epochs.size()))
                        view.selectedEpoch = selectedEpochLocal;
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(I18n::trc(view.qcTabKey))) {
                try {
                    QualityControl::render(view, exportStem());
                } catch (const std::exception &e) {
                    ImGui::TextColored(UI::kColFail, I18n::trc("solver.qc_render_error"), e.what());
                } catch (...) {
                    ImGui::TextColored(UI::kColFail, I18n::trc("solver.qc_render_unknown"));
                }
                ImGui::EndTabItem();
            }
            for (const auto &tab: view.extraQcTabs) {
                if (!tab) continue;
                if (ImGui::BeginTabItem(I18n::trc(tab->labelKey))) {
                    try {
                        QualityControl::render(tab->view, tab->fileName);
                    } catch (const std::exception &e) {
                        ImGui::TextColored(UI::kColFail, I18n::trc("solver.qc_render_error"), e.what());
                    } catch (...) {
                        ImGui::TextColored(UI::kColFail, I18n::trc("solver.qc_render_unknown"));
                    }
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        ExportUI::render();
    }
}
