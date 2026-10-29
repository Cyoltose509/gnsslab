#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include "GuiSolverPanel.h"
#include "GuiHelpers.h"
#include "imgui.h"
#include "QualityControl.h"
#include "Const.h"
#include "QCProcessor.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>
#include "GuiCharts.h"
#include "StringUtils.h"
#include "Log.h"
#include "widgets/tools/GuiInsPanel.h"

namespace Solvers::SolverPanel {
    template<typename T>
    static void copySampled(const std::vector<T> &src, std::vector<T> &dst,
                            const size_t maxPts = 3000) {
        if (src.size() <= maxPts) {
            dst = src;
            return;
        }
        size_t step = 1;
        while (step * maxPts < src.size()) step <<= 1; // ≥ ceil(size/maxPts) 的最小 2 的幂
        dst.clear();
        dst.reserve(src.size() / step + 2);
        for (size_t i = 0; i < src.size(); i += step)
            dst.push_back(src[i]);
        if ((src.size() - 1) % step != 0)
            dst.push_back(src.back()); // 保留末点，保证画到最新历元
    }

    static QualityReport buildReport(GnssTask::SolveTask &task) {
        std::vector<QC::QCObsEpoch> qcEpochs;
        {
            std::lock_guard lk(task.qcInputMutex);
            qcEpochs = task.qcInput;
        }
        QualityReport rep = QC::compute(qcEpochs, task.sampleInterval);
        rep.totalInputEpochs = static_cast<int>(qcEpochs.size());
        return rep;
    }

    void LaunchQC(const std::shared_ptr<GnssTask::SolveTask> &task) {
        static std::mutex launchMutex;
        std::lock_guard lk(launchMutex);
        if (task->qcComputing.load()) return;
        if (task->qcWorker.joinable()) task->qcWorker.join();
        LOG_INFO << "[QC] 开始质量分析";
        task->qcComputing = true;
        task->qcWorker = std::thread([task] {
            try {
                {
                    QualityReport rep = buildReport(*task);
                    std::lock_guard lk(task->qcMutex);
                    task->qcReport = std::make_shared<QualityReport>(std::move(rep));
                }
                task->qcReady = true;
                LOG_INFO << "[QC] 质量分析结束";
            } catch (...) {
                task->qcReady = true;
            }
            task->qcComputing = false;
        });
    }

    struct SolverUi {
        void (*solveStatusLine)(std::string &, ImVec4 &, const GnssTask::EpochData &);
    };


    static const char *solveStatLabel(const unsigned char stat) {
        return stat == 1 ? "固定(Fixed)" : "浮点(Float)";
    }

    static ImVec4 solveStatColor(const unsigned char stat) {
        return stat == 1 ? ImVec4(0.3f, 1, 0.3f, 1) : ImVec4(1, 0.8f, 0.2f, 1);
    }

    static void statusNone(std::string &, ImVec4 &, const GnssTask::EpochData &) {
    }

    static void statusRtk(std::string &text, ImVec4 &color, const GnssTask::EpochData &cur) {
        char buf[200];
        snprintf(buf, sizeof buf, "%s  ratio=%.2f  卫星=%d",
                 solveStatLabel(cur.rtkStat), cur.rtkRatio, cur.numSatsResult);
        text = buf;
        color = solveStatColor(cur.rtkStat);
    }

    // PPP/LEO：统一用 sppResult.stat/ratio（无 AR 时 stat 未置位，按浮点显示）
    static void statusAmbig(std::string &text, ImVec4 &color, const GnssTask::EpochData &cur) {
        unsigned char st = cur.sppResult.stat;
        if (st == 0) st = 2; // 未置位（PPP/LEO 无 AR）→ 浮点
        char buf[200];
        snprintf(buf, sizeof buf, "%s  ratio=%.2f  卫星=%d",
                 solveStatLabel(st), cur.sppResult.ratio, cur.numSatsResult);
        text = buf;
        color = solveStatColor(st);
    }

    static const SolverUi &solverUi(const GnssTask::SolverMode m) {
        static constexpr SolverUi generic{&statusNone}; // SPP
        static constexpr SolverUi ppp{&statusAmbig};
        static constexpr SolverUi leo{&statusAmbig};
        static constexpr SolverUi rtk{&statusRtk};
        switch (m) {
            case GnssTask::SolverMode::Leo: return leo;
            case GnssTask::SolverMode::Rtk: return rtk;
            case GnssTask::SolverMode::Ppp: return ppp;
            default: return generic;
        }
    }

    static void EnsureSkyTracksBuilt(const std::shared_ptr<GnssTask::SolveTask> &task) {
        {
            int totalEp = 0;
            std::lock_guard lk(task->mutex);
            totalEp = static_cast<int>(task->epochs.size());
            for (int e = task->skyTracksBuilt; e < totalEp; ++e) {
                auto &ep = task->epochs[e];
                for (int j = 0; j < static_cast<int>(ep.satIds.size()); ++j) {
                    const double el = ep.elevations[j] * RAD_TO_DEG;
                    if (el <= 0.0) continue;
                    task->skyTracks[ep.satIds[j]].push_back(
                        {e, static_cast<float>(ep.azimuths[j] * RAD_TO_DEG), static_cast<float>(el), ep.solved && !ep.rejected[j]});
                }
            }
            task->skyTracksBuilt = totalEp;
        }
    }

    static std::vector<std::pair<SatID, GnssTask::SkyPoint> > BuildCurrentSkyPoints(
        const std::shared_ptr<GnssTask::SolveTask> &task, const int sel) {
        std::vector<std::pair<SatID, GnssTask::SkyPoint> > curPts;
        std::lock_guard lk(task->mutex);
        if (sel < 0 || sel >= static_cast<int>(task->epochs.size())) return curPts;
        auto &ep = task->epochs[sel];
        return Charts::buildCurSkyPoints(sel, ep.satIds.size(),
                                                  [&](const size_t i) {
                                                      Charts::SkyView v;
                                                      v.sat = ep.satIds[i];
                                                      v.azimRad = ep.azimuths[i];
                                                      v.elevRad = ep.elevations[i];
                                                      v.used = ep.solved && !ep.rejected[i];
                                                      return v;
                                                  }, 0.0);
    }

    void RenderTaskImpl(const std::shared_ptr<GnssTask::SolveTask> &task);

    void RenderTask(const std::shared_ptr<GnssTask::SolverTaskBase> &job) {
        if (job->state == GnssTask::SolverTaskBase::State::Config) {
            job->renderConfigPanel();
            return;
        }
        // 惯导标定/对准任务有独立的结果面板（不画 GNSS 定位/质量分析页）
        if (Tools::InsPanel::RenderInsResult(job)) return;
        const auto core = job->solveResult();
        RenderTaskImpl(core);
    }

    void RenderTaskImpl(const std::shared_ptr<GnssTask::SolveTask> &task) {
        const bool isRealtime = task->isRealtime;
        // 实时流没有「取消」按钮，改由这里提供「停止连接」；批量解算的取消按钮在配置面板里。
        if (isRealtime) {
            if (!task->done) {
                if (ImGui::Button("停止连接")) task->stop = true;
                ImGui::SameLine();
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "连接已断开");
                ImGui::SameLine();
            }
        }

        const bool isDone = task->done.load();
        const bool hasError = task->errorFlag();

        const bool busy = !isDone && !hasError;
        auto [epochCount, selectedIdx] = Helpers::readEpochView(task);
        int selectedEpochLocal = selectedIdx;

        if (!isRealtime && busy) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f),
                               "正在处理 [%s]: 已解算 %d 历元",
                               task->fileName.c_str(), task->solvedCount.load());
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem("定位解算")) {
                Charts::PosTabView v;
                v.epochCount = epochCount;
                v.selectedEpoch = &selectedEpochLocal;
                v.selectedSatIdx = &task->selectedSatIdx;
                v.resMode = &task->resMode;
                v.refECEF = &task->refECEF;
                v.onRefChanged = [&] {
                    std::lock_guard lk(task->mutex);
                    std::lock_guard plk(task->plotMutex);
                    task->plotData.refreshENU(task->epochs, task->refECEF);
                };
                v.showEnPlan = task->mode != GnssTask::SolverMode::Leo;

                if (hasError) {
                    v.statusText = std::string("状态: ") + task->errorMessage();
                    v.statusColor = ImVec4(1, 0.3f, 0.3f, 1);
                } else if (epochCount > 0) {
                    v.statusText = "解算：共 " + std::to_string(epochCount) + " 个历元"
                                   + (busy ? "（进行中…）" : "");
                    v.statusColor = ImVec4(1, 0.6f, 0.2f, 1);
                } else if (busy) {
                    v.statusText = "正在读取观测 / 加载星历与产品…";
                } else {
                    v.statusText = "处理结束：未解出任何历元，请检查观测文件与产品是否匹配";
                    v.statusColor = ImVec4(1, 0.3f, 0.3f, 1);
                }
                v.busy = busy;

                // 天顶图轨迹增量构建（独立于下方锁，内部自行持锁）
                if (epochCount > 0) EnsureSkyTracksBuilt(task);

                if (epochCount > 0 && selectedIdx >= 0) {
                    std::lock_guard lk(task->mutex);
                    auto &cur = task->epochs[selectedIdx]; //NOLINT

                    std::vector<int> visIdx;
                    visIdx.reserve(static_cast<int>(cur.satIds.size()));
                    for (int i = 0; i < static_cast<int>(cur.satIds.size()); ++i)
                        if (task->enabledSystems.count(cur.satIds[i].system)) visIdx.push_back(i);

                    v.satRows = Charts::buildSatRows(visIdx.size(),
                                                              [&](const size_t i) {
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
                                                                  return _v;
                                                              },
                                                              [&](Charts::SatRow &sr, const size_t i) {
                                                                  const int oi = visIdx[i];
                                                                  const auto &pr = cur.sppResult.postRes;
                                                                  const auto &pp = cur.sppResult.postResPhase;
                                                                  const auto itC = pr.find(cur.satIds[oi]);
                                                                  const auto itP = pp.find(cur.satIds[oi]);
                                                                  sr.extra.emplace_back(
                                                                      "伪距残差(m)", itC != pr.end()
                                                                                       ? fmt4(itC->second)
                                                                                       : std::string("-"));
                                                                  // SPP 不做载波相位 → postResPhase 恒空，显示 "-"
                                                                  sr.extra.emplace_back(
                                                                      "相位残差(m)", itP != pp.end()
                                                                                         ? fmt4(itP->second)
                                                                                         : std::string("-"));
                                                              });
                    v.hasSatRows = true;

                    if (task->selectedSatIdx >= 0 && task->selectedSatIdx < static_cast<int>(visIdx.size())) {
                        const int oi = visIdx[task->selectedSatIdx];
                        v.showObsDetail = true;
                        v.detailSystem = cur.satIds[oi].system;
                        v.detailObs = cur.allObs[oi];
                    }

                    if (cur.solved) {
                        const auto &r = cur.sppResult;
                        v.solved = true;
                        v.xyz = r.xyz;
                        v.enu = XYZToENU(r.xyz, task->refECEF);
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


                        const bool isRtkMode = task->mode == GnssTask::SolverMode::Rtk;
                        const bool isLeoMode = task->mode == GnssTask::SolverMode::Leo;
                        const bool canFixed = isRtkMode || isLeoMode ||
                                              task->mode == GnssTask::SolverMode::Ppp;
                        const bool isFixed = isRtkMode ? cur.rtkStat == 1 : r.stat == 1;
                        v.hasFixedDistinction = canFixed;
                        v.hasFixed = isFixed;

                        Eigen::Vector3d xyzFloat = r.xyzFloat;
                        Eigen::Vector3d xyzFixed = r.xyzFixed.squaredNorm() > 1.0 ? r.xyzFixed : r.xyz;
                        if (isRtkMode) {
                            xyzFloat = cur.rtkFloatXyz;
                            xyzFixed = cur.rtkStat == 1 ? r.xyz : cur.rtkFloatXyz;
                        }
                        if (xyzFloat.squaredNorm() < 1.0) xyzFloat = r.xyz; // 兜底：无浮点解时用当前解
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
                                                      : r.xyz - task->refECEF;
                        }

                        // 速度
                        if (r.vel.squaredNorm() > 1e-4) {
                            v.hasVelDisp = true;
                            v.velDisp = r.vel;
                            v.sigmaVDisp = r.sigmaV;
                        }

                        v.showRefTable = canFixed && (!isLeoMode || !task->leoRefTraj.empty());
                        v.isLeoRtn = isLeoMode;
                        v.refDisp = task->refECEF;
                        const Eigen::Vector3d xyzPrim = isFixed ? xyzFixed : xyzFloat;
                        v.abs3dDisp = XYZToENU(xyzPrim, task->refECEF).norm();
                        if (isLeoMode) {
                            v.rtnDisp = Eigen::Vector3d(cur.rtnR, cur.rtnT, cur.rtnN);
                            v.rtn3dDisp = std::sqrt(cur.rtnR * cur.rtnR + cur.rtnT * cur.rtnT + cur.rtnN * cur.rtnN);
                        } else {
                            v.enuFloatDisp = XYZToENU(xyzFloat, task->refECEF);
                            v.enuFixedDisp = XYZToENU(xyzFixed, task->refECEF);
                            v.enuFloat3d = v.enuFloatDisp.norm();
                            v.enuFixed3d = v.enuFixedDisp.norm();
                        }

                        // 解状态 / ratio（用于 DOP 表追加列与解状态文本块）
                        v.solStatShort = isFixed ? "固定(Fixed)" : "浮点(Float)";
                        v.solRatio = isRtkMode ? cur.rtkRatio : r.ratio;
                        v.showSolveStatus = canFixed;
                    } else {
                        v.noSolveMsg = task->noEphSolve ? "无定位解（缺少星历文件）" : "无定位解";
                    }

                    char buf[160];
                    snprintf(buf, sizeof buf, "Wk %u SOW %.3f | %s", cur.week, cur.sow, cur.solved ? "定位" : "无解");
                    v.epochInfo = buf;
                }

                // 图表数据：在各自锁内快照为值，避免渲染期与解算线程竞争
                if (epochCount > 0 && selectedIdx >= 0) {
                    bool sigmaVNonZero = false;
                    {
                        std::lock_guard plk(task->plotMutex);
                        auto &pp = task->plotData; //NOLINT
                        v.times = pp.times;
                        v.enu_e = pp.enu_e;
                        v.enu_n = pp.enu_n;
                        v.enu_u = pp.enu_u;
                        {
                            std::vector<double> solvedFlag;
                            solvedFlag.reserve(static_cast<int>(task->epochs.size()));
                            for (const auto &ep : task->epochs) solvedFlag.push_back(ep.solved ? 1.0 : 0.0);
                            v.enu_solved = std::move(solvedFlag);
                        }
                        v.newed = pp.newed;
                        pp.newed = false;
                        v.fixedY = true;
                        v.hasResidualData = !pp.satResVals.empty() || !pp.satResPhaseVals.empty();
                        if (task->resMode == 1) {
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
                        // 相位残差量级 ~cm：半宽下限给 0.01 m，否则会被码残差的 1 m 下限压成直线
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
                        v.showRatio = task->mode == GnssTask::SolverMode::Rtk ||
                                      task->mode == GnssTask::SolverMode::Ppp ||
                                      task->mode == GnssTask::SolverMode::Leo;
                        v.hasFloatSeries = v.showRatio;
                        v.showBaseXyz = task->mode == GnssTask::SolverMode::Rtk;
                        v.isRtk = task->mode == GnssTask::SolverMode::Rtk;
                        v.baseXyz = &task->refECEF;
                    }
                    {
                        std::lock_guard lk(task->mutex);
                        v.skyTracks = task->skyTracks; // 全量（同样不抽稀）
                        int solvedCnt = 0, velCnt = 0;
                        for (const auto &ep: task->epochs) {
                            if (!ep.solved) continue;
                            ++solvedCnt;
                            if (ep.sppResult.vel.squaredNorm() > 1e-4) ++velCnt;
                        }
                        v.showSigmaV = solvedCnt > 0 && static_cast<double>(velCnt) / solvedCnt > 0.05
                                       && sigmaVNonZero;
                    }
                    v.curSkyPts = BuildCurrentSkyPoints(task, selectedIdx); // 内部自行持锁
                    v.showSigmaDop = true;

                    if (v.showRatio && epochCount > 0) {
                        int fixedCnt = 0, totalCnt = 0;
                        {
                            std::lock_guard lk(task->mutex);
                            for (const auto &ep: task->epochs) {
                                if (!ep.solved) continue;
                                ++totalCnt;
                                const bool isFixed = task->mode == GnssTask::SolverMode::Rtk
                                                         ? ep.rtkStat == 1
                                                         : ep.sppResult.stat == 1;
                                if (isFixed) ++fixedCnt;
                            }
                        }
                        if (totalCnt > 0) {
                            v.showFixRate = true;
                            char buf[64];
                            snprintf(buf, sizeof buf, "固定率: %.1f%% (%d/%d)",
                                     100.0 * fixedCnt / totalCnt, fixedCnt, totalCnt);
                            v.fixRateText = buf;
                        }
                    }
                }
                {
                    std::lock_guard lk(task->mutex);
                    if (task->hasLeo) {
                        v.show3d = true;
                        v.showRtn = !task->leoRefTraj.empty(); // 无参考轨道则不画 RTN 序列
                        v.hasRef = !task->leoRefTraj.empty(); // 无参考轨道则不显示 RTN 面板
                        v.showRefEnu = false; // LEO 不编辑/显示参考真值
                        v.leoPos3d = task->leoPos3d;
                        v.leoRef3d = task->leoRef3d;
                        copySampled(task->leoTraj, v.leoTraj);
                        copySampled(task->leoRefTraj, v.leoRefTraj);
                        v.gnssVis = task->gnssVis;
                        for (const auto &[sat, traj]: task->gnssTraj)
                            copySampled(traj, v.gnssTraj[sat], 800); // 卫星轨迹更粗即可
                        copySampled(task->rtnR, v.rtnR);
                        copySampled(task->rtnT, v.rtnT);
                        copySampled(task->rtnN, v.rtnN);
                        copySampled(task->rtnD3, v.rtnD3);
                        copySampled(task->rtnTimes, v.rtnTimes); // 必须与 RTN 数组同采样，否则索引错位
                    }
                }

                if (epochCount > 0 && selectedIdx >= 0 && v.solved) {
                    std::lock_guard lk(task->mutex);
                    const auto &[solveStatusLine] = solverUi(task->mode);
                    std::string line;
                    ImVec4 col;
                    solveStatusLine(line, col, task->epochs[selectedIdx]);
                    if (!line.empty()) {
                        v.showSolveStatus = true;
                        v.solveStatusText = std::string(modeLabel(task->mode)) + ": " + line;
                        v.solveStatusColor = col;
                    }
                }

                v.isDone = isDone || isRealtime;
                v.onExportCsv = [&] {
                    ExportCsv(task);
                };


                try {
                    Solvers::Charts::RenderPositioningTab(v);
                } catch (const std::exception &e) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "渲染发生未知异常");
                }
                if (selectedEpochLocal != selectedIdx) {
                    std::lock_guard lk(task->mutex);
                    if (selectedEpochLocal >= 0 && selectedEpochLocal < static_cast<int>(task->epochs.size()))
                        task->selectedEpoch = selectedEpochLocal;
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(task->qcTabLabel.c_str())) {
                try {
                    QualityControl::render(task);
                } catch (const std::exception &e) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "质量分析渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "质量分析渲染发生未知异常");
                }
                ImGui::EndTabItem();
            }
            for (auto &[qcLabel, qcSub]: task->extraQcTabs) {
                if (!qcSub) continue;
                if (ImGui::BeginTabItem(qcLabel.c_str())) {
                    try {
                        QualityControl::render(qcSub);
                    } catch (const std::exception &e) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "质量分析渲染异常: %s", e.what());
                    } catch (...) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "质量分析渲染发生未知异常");
                    }
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
    }

    void ExportCsv(const std::shared_ptr<GnssTask::SolveTask> &task) {
        std::string dn = Helpers::baseName(task->fileName);
        if (const auto dp = dn.rfind('.'); dp != std::string::npos) dn = dn.substr(0, dp);
        dn += "_" + std::string(modeLabel(task->mode)) + "_result.csv";
        if (dn.size() >= MAX_PATH) dn = std::string(modeLabel(task->mode)) + "_result.csv";
        std::wstring path;
        if (!Helpers::saveCSVDialog(dn, path)) return;
        LOG_INFO << "[导出] CSV (" << modeLabel(task->mode) << ") → "
                << std::filesystem::path(path).string();

        std::string csvText;
        {
            std::lock_guard lk(task->mutex);
            std::ostringstream oss;
            task->exportCsv(oss);
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
