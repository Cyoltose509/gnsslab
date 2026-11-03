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
#include "view/GuiCharts.h"
#include "view/GuiHelpers.h"
#include "view/GuiPosTab.h"
#include "view/QualityControl.h"
#include "widgets/tools/GuiInsPanel.h"
#include "solvers/spp/Task.h"
#include "solvers/ppp/Task.h"
#include "solvers/leo/Task.h"
#include "solvers/rtk/Task.h"

namespace Solvers::SolverPanel {
    static QualityReport buildReport(Solvers::Core &task) {
        std::vector<QC::QCObsEpoch> qcEpochs;
        {
            std::lock_guard lk(task.view.qcInputMutex);
            qcEpochs = task.view.qcInput;
        }
        QualityReport rep = QC::compute(qcEpochs, task.sampleInterval);
        rep.totalInputEpochs = static_cast<int>(qcEpochs.size());
        return rep;
    }

    void launchQC(const std::shared_ptr<Solvers::Core> &task) {
        static std::mutex launchMutex;
        std::lock_guard lk(launchMutex);
        if (task->view.qcComputing.load()) return;
        if (task->view.qcWorker.joinable()) task->view.qcWorker.join();
        LOG_INFO << "[QC] 开始质量分析";
        task->view.qcComputing = true;
        task->view.qcWorker = std::thread([task] {
            try {
                {
                    QualityReport rep = buildReport(*task);
                    std::lock_guard lk(task->view.qcMutex);
                    task->view.qcReport = std::make_shared<QualityReport>(std::move(rep));
                }
                task->view.qcReady = true;
                LOG_INFO << "[QC] 质量分析结束";
            } catch (...) {
                task->view.qcReady = true;
            }
            task->view.qcComputing = false;
        });
    }

    using StatusLine = void (*)(std::string &, ImVec4 &, const Solvers::Epoch &);

    static const char *solveStatLabel(const unsigned char stat) {
        return stat == 1 ? "固定(Fixed)" : "浮点(Float)";
    }

    static ImVec4 solveStatColor(const unsigned char stat) {
        return stat == 1 ? UI::kColPass : UI::kColWarn;
    }

    static void statusNone(std::string &, ImVec4 &, const Solvers::Epoch &) {
    }

    static void statusRtk(std::string &text, ImVec4 &color, const Solvers::Epoch &cur) {
        char buf[200];
        std::snprintf(buf, sizeof buf, "%s  ratio=%.2f  卫星=%d",
                 solveStatLabel(cur.rtkStat), cur.rtkRatio, cur.numSatsResult);
        text = buf;
        color = solveStatColor(cur.rtkStat);
    }

    static void statusAmbig(std::string &text, ImVec4 &color, const Solvers::Epoch &cur) {
        unsigned char st = cur.sppResult.stat;
        if (st == 0) st = 2;
        char buf[200];
        std::snprintf(buf, sizeof buf, "%s  ratio=%.2f  卫星=%d",
                 solveStatLabel(st), cur.sppResult.ratio, cur.numSatsResult);
        text = buf;
        color = solveStatColor(st);
    }

    static StatusLine statusLineFor(const Solvers::Mode m) {
        switch (m) {
            case Solvers::Mode::LEO:
            case Solvers::Mode::PPP:
                return &statusAmbig;
            case Solvers::Mode::RTK:
                return &statusRtk;
            default:
                return &statusNone;
        }
    }

    static void ensureSkyTracks(Solvers::Core &task) {
        std::lock_guard lk(task.mutex);
        const int totalEp = static_cast<int>(task.epochs.size());
        for (int e = task.view.skyTracksBuilt; e < totalEp; ++e) {
            const auto &ep = task.epochs[e];
            for (int j = 0; j < static_cast<int>(ep.satIds.size()); ++j) {
                const double el = ep.elevations[j] * RAD_TO_DEG;
                if (el <= 0.0) continue;
                task.view.skyTracks[ep.satIds[j]].push_back(
                    {e, static_cast<float>(ep.azimuths[j] * RAD_TO_DEG), static_cast<float>(el), ep.solved && !ep.rejected[j]});
            }
        }
        task.view.skyTracksBuilt = totalEp;
    }

    static std::vector<std::pair<SatID, Solvers::SkyPoint>> currentSkyPoints(
        Solvers::Core &task, const int sel) {
        std::lock_guard lk(task.mutex);
        if (sel < 0 || sel >= static_cast<int>(task.epochs.size())) return {};
        auto &ep = task.epochs[sel];
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

    static void fillStatusLine(Charts::PosTabView &v, Solvers::Core &task,
                               const bool hasError, const bool busy, const int epochCount) {
        if (hasError) {
            v.statusText = std::string("状态: ") + task.errorMessage();
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

    static void fillCurrentEpoch(Charts::PosTabView &v, Solvers::Core &task, const int selectedIdx) {
        std::lock_guard lk(task.mutex);
        auto &cur = task.epochs[selectedIdx];

        std::vector<int> visIdx;
        visIdx.reserve(static_cast<int>(cur.satIds.size()));
        for (int i = 0; i < static_cast<int>(cur.satIds.size()); ++i)
            if (task.enabledSystems.count(cur.satIds[i].system)) visIdx.push_back(i);

        v.satRows = Charts::buildSatRows(visIdx.size(),
                                         [&](const std::size_t i) {
                                             const int oi = visIdx[i];
                                             Charts::SatRowView _v;
                                             _v.sat = cur.satIds[oi];
                                             _v.used = !cur.rejected[oi] && cur.solved;
                                             _v.elevRad = cur.elevations[oi];
                                             _v.azimRad = cur.azimuths[oi];
                                             const bool haveXYZ = cur.satPVTs[oi].p.squaredNorm() > 1.0;
                                             _v.hasXYZ = haveXYZ;
                                             if (haveXYZ) {
                                                 _v.x = cur.satPVTs[oi].p[0];
                                                 _v.y = cur.satPVTs[oi].p[1];
                                                 _v.z = cur.satPVTs[oi].p[2];
                                             }
                                             const auto itC = cur.sppResult.postRes.find(cur.satIds[oi]);
                                             if (itC != cur.sppResult.postRes.end()) _v.codeRes = itC->second;
                                             const auto itP = cur.sppResult.postResPhase.find(cur.satIds[oi]);
                                             if (itP != cur.sppResult.postResPhase.end()) _v.phaseRes = itP->second;
                                             return _v;
                                         });
        v.hasSatRows = true;

        if (task.view.selectedSatIdx >= 0 && task.view.selectedSatIdx < static_cast<int>(visIdx.size())) {
            const int oi = visIdx[task.view.selectedSatIdx];
            v.showObsDetail = true;
            v.detailSystem = cur.satIds[oi].system;
            v.detailObs = cur.allObs[oi];
        }

        if (cur.solved) {
            const auto &r = cur.sppResult;
            v.solved = true;
            v.xyz = r.xyz;
            v.enu = Coord::XYZToENU(r.xyz, task.refECEF);
            v.blh = r.blh;
            v.sigmaP = r.sigmaP;
            v.showVelDop = true;
            v.showVel = r.vel.squaredNorm() > 1e-4;
            v.vel = r.vel;
            v.pdop = r.pdop;
            v.gdop = r.gdop;
            v.hdop = r.hdop;
            v.vdop = r.vdop;
            v.tdop = r.tdop;
            v.numSatsResult = cur.numSatsResult;
            v.numObs = cur.numObs;

            const bool isRtkMode = task.mode == Solvers::Mode::RTK;
            const bool isLeoMode = task.mode == Solvers::Mode::LEO;
            const bool canFixed = isRtkMode || isLeoMode ||
                                  task.mode == Solvers::Mode::PPP;
            const bool isFixed = isRtkMode ? cur.rtkStat == 1 : r.stat == 1;
            v.hasFixedDistinction = canFixed;
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
                                          : r.xyz - task.refECEF;
            }

            if (r.vel.squaredNorm() > 1e-4) {
                v.hasVelDisp = true;
                v.velDisp = r.vel;
                v.sigmaVDisp = r.sigmaV;
            }

            v.showRefTable = canFixed && (!isLeoMode || !task.view.leoRefTraj.empty());
            v.isLeoRtn = isLeoMode;
            v.refDisp = task.refECEF;
            const Eigen::Vector3d xyzPrim = isFixed ? xyzFixed : xyzFloat;
            v.abs3dDisp = Coord::XYZToENU(xyzPrim, task.refECEF).norm();
            if (isLeoMode) {
                v.rtnDisp = Eigen::Vector3d(cur.rtnR, cur.rtnT, cur.rtnN);
                v.rtn3dDisp = std::sqrt(cur.rtnR * cur.rtnR + cur.rtnT * cur.rtnT + cur.rtnN * cur.rtnN);
            } else {
                v.enuFloatDisp = Coord::XYZToENU(xyzFloat, task.refECEF);
                v.enuFixedDisp = Coord::XYZToENU(xyzFixed, task.refECEF);
                v.enuFloat3d = v.enuFloatDisp.norm();
                v.enuFixed3d = v.enuFixedDisp.norm();
            }

            v.solStatShort = isFixed ? "固定(Fixed)" : "浮点(Float)";
            v.solRatio = isRtkMode ? cur.rtkRatio : r.ratio;
            v.showSolveStatus = canFixed;
        } else {
            v.noSolveMsg = task.noEphSolve ? "无定位解（缺少星历文件）" : "无定位解";
        }

        char buf[160];
        std::snprintf(buf, sizeof buf, "Wk %u SOW %.3f | %s", cur.week, cur.sow, cur.solved ? "定位" : "无解");
        v.epochInfo = buf;
    }

    static void fillPlots(Charts::PosTabView &v, Solvers::Core &task,
                          const int selectedIdx, const int epochCount, const bool isDone) {
        bool sigmaVNonZero = false;
        {
            std::lock_guard plk(task.view.plotMutex);
            auto &pp = task.view.plotData;
            v.times = pp.times;
            v.enu_e = pp.enu_e;
            v.enu_n = pp.enu_n;
            v.enu_u = pp.enu_u;
            {
                std::vector<double> solvedFlag;
                solvedFlag.reserve(static_cast<int>(task.epochs.size()));
                for (const auto &ep : task.epochs) solvedFlag.push_back(ep.solved ? 1.0 : 0.0);
                v.enu_solved = std::move(solvedFlag);
            }
            v.newed = pp.newed;
            pp.newed = false;
            v.fixedY = true;
            v.hasResidualData = !pp.satResVals.empty() || !pp.satResPhaseVals.empty();
            if (task.view.resMode == 1) {
                v.resPhaseT = pp.satResPhaseTimes;
                v.resPhaseV = pp.satResPhaseVals;
            } else {
                v.resT = pp.satResTimes;
                v.resV = pp.satResVals;
            }
            if (isDone && !pp.resRangeReady) {
                auto [lo, hi] = Helpers::computeRobustResRange(pp.satResVals);
                pp.resYlo = lo;
                pp.resYhi = hi;
                pp.resRangeReady = true;
            }

            if (isDone && !pp.resPhaseRangeReady) {
                auto [lo, hi] = Helpers::computeRobustResRange(pp.satResPhaseVals, 0.01);
                pp.resPhaseYlo = lo;
                pp.resPhaseYhi = hi;
                pp.resPhaseRangeReady = true;
            }
            v.robustRes = pp.resRangeReady;
            v.resYlo = pp.resYlo;
            v.resYhi = pp.resYhi;
            v.robustResPhase = pp.resPhaseRangeReady;
            v.resPhaseYlo = pp.resPhaseYlo;
            v.resPhaseYhi = pp.resPhaseYhi;
            v.sigmaPs = pp.sigmaPs;
            v.sigmaPFloatVals = pp.sigmaPFloatVals;
            v.sigmaVs = pp.sigmaVs;
            sigmaVNonZero = std::any_of(pp.sigmaVs.begin(), pp.sigmaVs.end(),
                                        [](const double x) { return x > 1e-6; });
            v.pdops = pp.pdops;
            v.gdops = pp.gdops;
            v.hdops = pp.hdops;
            v.vdops = pp.vdops;
            v.tdops = pp.tdops;
            v.ratioVals = pp.ratioVals;
            v.float_e = pp.float_e;
            v.float_n = pp.float_n;
            v.float_u = pp.float_u;
            v.showRatio = task.mode == Solvers::Mode::RTK ||
                          task.mode == Solvers::Mode::PPP ||
                          task.mode == Solvers::Mode::LEO;
            v.hasFloatSeries = v.showRatio;
            v.showBaseXyz = task.mode == Solvers::Mode::RTK;
            v.isRtk = task.mode == Solvers::Mode::RTK;
            v.baseXyz = &task.refECEF;
        }
        {
            std::lock_guard lk(task.mutex);
            v.skyTracks = task.view.skyTracks;
            int solvedCnt = 0, velCnt = 0;
            for (const auto &ep: task.epochs) {
                if (!ep.solved) continue;
                ++solvedCnt;
                if (ep.sppResult.vel.squaredNorm() > 1e-4) ++velCnt;
            }
            v.showSigmaV = solvedCnt > 0 && static_cast<double>(velCnt) / solvedCnt > 0.05
                           && sigmaVNonZero;
        }
        v.curSkyPts = currentSkyPoints(task, selectedIdx);
        v.showSigmaDop = true;

        if (v.showRatio && epochCount > 0) {
            int fixedCnt = 0, totalCnt = 0;
            {
                std::lock_guard lk(task.mutex);
                for (const auto &ep: task.epochs) {
                    if (!ep.solved) continue;
                    ++totalCnt;
                    const bool isFixed = task.mode == Solvers::Mode::RTK
                                             ? ep.rtkStat == 1
                                             : ep.sppResult.stat == 1;
                    if (isFixed) ++fixedCnt;
                }
            }
            if (totalCnt > 0) {
                v.showFixRate = true;
                char buf[64];
                std::snprintf(buf, sizeof buf, "固定率: %.1f%% (%d/%d)",
                         100.0 * fixedCnt / totalCnt, fixedCnt, totalCnt);
                v.fixRateText = buf;
            }
        }
    }

    static void fillLeo3d(Charts::PosTabView &v, Solvers::Core &task) {
        std::lock_guard lk(task.mutex);
        if (!task.view.hasLeo) return;
        v.show3d = true;
        v.showRtn = !task.view.leoRefTraj.empty();
        v.hasRef = !task.view.leoRefTraj.empty();
        v.showRefEnu = false;
        v.leoPos3d = task.view.leoPos3d;
        v.leoRef3d = task.view.leoRef3d;
        copySampled(task.view.leoTraj, v.leoTraj);
        copySampled(task.view.leoRefTraj, v.leoRefTraj);
        v.gnssVis = task.view.gnssVis;
        for (const auto &[sat, traj]: task.view.gnssTraj)
            copySampled(traj, v.gnssTraj[sat], 800);
        copySampled(task.view.rtnR, v.rtnR);
        copySampled(task.view.rtnT, v.rtnT);
        copySampled(task.view.rtnN, v.rtnN);
        copySampled(task.view.rtnD3, v.rtnD3);
        copySampled(task.view.rtnTimes, v.rtnTimes);
    }

    static void fillSolveStatus(Charts::PosTabView &v, Solvers::Core &task, const int selectedIdx) {
        std::lock_guard lk(task.mutex);
        const StatusLine solveStatusLine = statusLineFor(task.mode);
        std::string line;
        ImVec4 col;
        solveStatusLine(line, col, task.epochs[selectedIdx]);
        if (!line.empty()) {
            v.showSolveStatus = true;
            v.solveStatusText = std::string(Solvers::modeLabel(task.mode)) + ": " + line;
            v.solveStatusColor = col;
        }
    }

    static void renderTaskImpl(const std::shared_ptr<Solvers::Core> &task) {
        const bool isRealtime = task->isRealtime;

        if (isRealtime) {
            if (!task->done) {
                if (ImGui::Button("停止连接")) task->stop = true;
                ImGui::SameLine();
            } else {
                ImGui::TextColored(UI::kColFail, "连接已断开");
                ImGui::SameLine();
            }
        }

        const bool isDone = task->done.load();
        const bool hasError = task->errorFlag();

        const bool busy = !isDone && !hasError;
        auto [epochCount, selectedIdx] = Helpers::readEpochView(task);
        int selectedEpochLocal = selectedIdx;

        if (!isRealtime && busy) {
            ImGui::TextColored(UI::kColInfo,
                               "正在处理 [%s]: 已解算 %d 历元",
                               task->fileName.c_str(), task->solvedCount.load());
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem("定位解算")) {
                Charts::PosTabView v;
                v.epochCount = epochCount;
                v.selectedEpoch = &selectedEpochLocal;
                v.selectedSatIdx = &task->view.selectedSatIdx;
                v.resMode = &task->view.resMode;
                v.refECEF = &task->refECEF;
                v.onRefChanged = [&] {
                    std::lock_guard lk(task->mutex);
                    std::lock_guard plk(task->view.plotMutex);
                    task->view.plotData.refreshENU(task->epochs, task->refECEF);
                };
                v.showEnPlan = task->mode != Solvers::Mode::LEO;

                fillStatusLine(v, *task, hasError, busy, epochCount);
                v.busy = busy;

                if (epochCount > 0) ensureSkyTracks(*task);
                if (epochCount > 0 && selectedIdx >= 0) fillCurrentEpoch(v, *task, selectedIdx);
                if (epochCount > 0 && selectedIdx >= 0) fillPlots(v, *task, selectedIdx, epochCount, isDone);
                fillLeo3d(v, *task);
                if (epochCount > 0 && selectedIdx >= 0 && v.solved)
                    fillSolveStatus(v, *task, selectedIdx);

                v.isDone = isDone || isRealtime;
                v.onExportCsv = [&] {
                    exportCSV(task);
                };

                try {
                    Charts::renderPositioningTab(v);
                } catch (const std::exception &e) {
                    ImGui::TextColored(UI::kColFail, "渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(UI::kColFail, "渲染发生未知异常");
                }
                if (selectedEpochLocal != selectedIdx) {
                    std::lock_guard lk(task->mutex);
                    if (selectedEpochLocal >= 0 && selectedEpochLocal < static_cast<int>(task->epochs.size()))
                        task->view.selectedEpoch = selectedEpochLocal;
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(task->view.qcTabLabel.c_str())) {
                try {
                    QualityControl::render(task);
                } catch (const std::exception &e) {
                    ImGui::TextColored(UI::kColFail, "质量分析渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(UI::kColFail, "质量分析渲染发生未知异常");
                }
                ImGui::EndTabItem();
            }
            for (auto &[qcLabel, qcSub]: task->view.extraQcTabs) {
                if (!qcSub) continue;
                if (ImGui::BeginTabItem(qcLabel.c_str())) {
                    try {
                        QualityControl::render(qcSub);
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

    static void renderConfigPanel(Solvers::Task &job) {
        switch (job.mode()) {
            case Solvers::Mode::SPP: static_cast<Solvers::SPP::Task &>(job).renderConfigPanel(); break;
            case Solvers::Mode::PPP: static_cast<Solvers::PPP::Task &>(job).renderConfigPanel(); break;
            case Solvers::Mode::LEO: static_cast<Solvers::LEO::Task &>(job).renderConfigPanel(); break;
            case Solvers::Mode::RTK: static_cast<Solvers::RTK::Task &>(job).renderConfigPanel(); break;
            case Solvers::Mode::INS: Tools::InsPanel::RenderInsConfig(job); break;
            default: break;
        }
    }

    void renderTask(const std::shared_ptr<Solvers::Task> &job) {
        if (job->state == Solvers::Task::State::Config) {
            renderConfigPanel(*job);
            return;
        }

        if (Tools::InsPanel::RenderInsResult(job)) return;
        renderTaskImpl(job->solveResult());
    }

    void exportCSV(const std::shared_ptr<Solvers::Core> &task) {
        std::string dn = Str::stripExt(Str::baseName(task->fileName));
        dn += "_" + std::string(Solvers::modeLabel(task->mode)) + "_result.csv";
        if (dn.size() >= MAX_PATH) dn = std::string(Solvers::modeLabel(task->mode)) + "_result.csv";
        std::wstring path;
        if (!FileDialog::saveCsv(dn, path)) return;
        LOG_INFO << "[导出] CSV (" << Solvers::modeLabel(task->mode) << ") → "
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
