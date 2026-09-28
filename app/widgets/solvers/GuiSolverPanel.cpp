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
#include <iomanip>
#include <iostream>
#include <set>
#include <thread>
#include <utility>
#include <vector>
#include "GuiCharts.h"
#include "StringUtils.h"

namespace GuiSolverPanel {
    static QualityReport buildReport(GnssTask::SolveTask &task) {
        std::vector<QC::QCObsEpoch> qcEpochs;
        {
            std::lock_guard lk(task.qcInputMutex);
            qcEpochs = task.qcInput;
        }
        QualityReport rep = QC::compute(qcEpochs);
        rep.totalInputEpochs = static_cast<int>(qcEpochs.size());
        return rep;
    }

    void LaunchQC(const std::shared_ptr<GnssTask::SolveTask> &task) {
        if (task->qcWorker.joinable()) task->qcWorker.join(); // 确保上一轮已结束，再起新的一轮
        task->qcComputing = true;
        task->qcWorker = std::thread([task] {
            try {
                {
                    QualityReport rep = buildReport(*task);
                    std::lock_guard lk(task->qcMutex);
                    task->qcReport = std::make_shared<QualityReport>(std::move(rep));
                }
                task->qcReady = true;
            } catch (...) {
                task->qcReady = true;
            }
            task->qcComputing = false;
        });
    }

    // ------------------------------------------------------------------
    // 各解算系统的"结果呈现"差异只有两处：CSV 列定义、状态行后缀。
    // 原先它们分别被写成 ExportCsv 里的 isLeo/isRtk/else 两层嵌套分支和
    // RenderTaskImpl 里一句 `if (mode == Rtk)`，三种格式互相缠绕、加一种解算就要再插一支。
    // 这里把它们收成"按 mode 查表"的描述符，主流程只剩通用骨架（写 Wk/SOW → 委派 → 换行）。
    // ------------------------------------------------------------------
    struct SolverUi {
        void (*csvHeader)(std::ostream &, const GnssTask::SolveTask &);
        void (*csvRow)(std::ostream &, const GnssTask::EpochData &, const GnssTask::SolveTask &);
        void (*statusSuffix)(std::string &, ImVec4 &, const GnssTask::EpochData &);
    };

    // SPP/PPP/LEO 共用的尾部列：B/L/H、速度、各类 DOP、SigmaP/SigmaV、卫星数
    static void csvRowTail(std::ostream &out, const GnssTask::EpochData &r) {
        const auto &res = r.sppResult;
        out << ',' << std::setprecision(8) << res.blh[0] * RAD_TO_DEG << ',' << res.blh[1] * RAD_TO_DEG << ','
            << std::setprecision(3) << res.blh[2] << ',' << res.vel[0] << ',' << res.vel[1] << ',' << res.vel[2]
            << ',' << std::setprecision(4) << res.pdop << ',' << res.gdop << ',' << res.hdop << ',' << res.vdop
            << ',' << res.tdop << ',' << res.sigmaP << ',' << res.sigmaV << ',' << r.numSatsResult;
    }

    // --- SPP / PPP：ENU + 常数 REF ---
    static void csvHeaderGeneric(std::ostream &out, const GnssTask::SolveTask &) {
        out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-X/m,REF-Y/m,REF-Z/m,EAST/m,NORTH/m,UP/m,"
               "B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
    }
    static void csvRowGeneric(std::ostream &out, const GnssTask::EpochData &r,
                             const GnssTask::SolveTask &task) {
        const auto &res = r.sppResult;
        out << std::setprecision(4) << res.xyz[0] << ',' << res.xyz[1] << ',' << res.xyz[2];
        const ENU enu = XYZtoENU(res.xyz, task.refECEF);
        out << ',' << task.refECEF.X() << ',' << task.refECEF.Y() << ',' << task.refECEF.Z();
        out << ',' << enu.E() << ',' << enu.N() << ',' << enu.U();
        csvRowTail(out, r);
    }

    // --- LEO：逐历元参考轨道 REF + RTN(径向/沿迹/法向)；无参考轨道则两者都略过 ---
    static void csvHeaderLeo(std::ostream &out, const GnssTask::SolveTask &task) {
        if (task.hasRefOrbit)
            out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-X/m,REF-Y/m,REF-Z/m,R/m,T/m,N/m,"
                   "B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
        else
            out << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,"
                   "B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,GDOP,HDOP,VDOP,TDOP,SigmaP,SigmaV,SatCount\n";
    }
    static void csvRowLeo(std::ostream &out, const GnssTask::EpochData &r,
                          const GnssTask::SolveTask &task) {
        const auto &res = r.sppResult;
        out << std::setprecision(4) << res.xyz[0] << ',' << res.xyz[1] << ',' << res.xyz[2];
        if (task.hasRefOrbit) {
            out << ',' << r.refECEF[0] << ',' << r.refECEF[1] << ',' << r.refECEF[2];
            out << ',' << r.rtnR << ',' << r.rtnT << ',' << r.rtnN;
        }
        csvRowTail(out, r);
    }

    // --- RTK：固定/浮动基线(ECEF + ENU) + 解状态 + LAMBDA ratio ---
    static void csvHeaderRtk(std::ostream &out, const GnssTask::SolveTask &) {
        out << "Wk,SOW,STAT,Fix-X/m,Fix-Y/m,Fix-Z/m,Fix-E/m,Fix-N/m,Fix-U/m,"
               "Float-X/m,Float-Y/m,Float-Z/m,Float-E/m,Float-N/m,Float-U/m,Ratio,NSats\n";
    }
    static void csvRowRtk(std::ostream &out, const GnssTask::EpochData &r,
                          const GnssTask::SolveTask &task) {
        // 固定基线 = sppResult.xyz − refECEF（求解线程已令 refECEF = 基准站坐标）
        const Vector3d ref(task.refECEF.X(), task.refECEF.Y(), task.refECEF.Z());
        const Vector3d blX = r.rtkBaseFloat;
        const ENU enuX = XYZtoENU(XYZ(ref.x() + blX.x(), ref.y() + blX.y(), ref.z() + blX.z()), task.refECEF);
        out << std::setprecision(4) << r.rtkStat << ',';
        if (r.rtkStat == 1) {
            const Vector3d blF = r.sppResult.xyz - ref;
            const ENU enuF = XYZtoENU(XYZ(ref.x() + blF.x(), ref.y() + blF.y(), ref.z() + blF.z()), task.refECEF);
            out << blF.x() << ',' << blF.y() << ',' << blF.z() << ','
                << enuF.E() << ',' << enuF.N() << ',' << enuF.U() << ',';
        } else {
            // 浮动历元没有整数解：Fix 列留空（否则会把浮点估值误画成"固定跳变"尖峰）
            out << ",,,,,,";
        }
        out << blX.x() << ',' << blX.y() << ',' << blX.z() << ','
            << enuX.E() << ',' << enuX.N() << ',' << enuX.U() << ','
            << r.rtkRatio << ',' << r.numSatsResult;
    }

    static void statusSuffixNone(std::string &, ImVec4 &, const GnssTask::EpochData &) {}
    static void statusSuffixRtk(std::string &text, ImVec4 &color, const GnssTask::EpochData &cur) {
        const char *st = cur.rtkStat == 1 ? "固定(Fixed)" : cur.rtkStat == 2 ? "浮动(Float)" : "无效";
        char buf2[200];
        snprintf(buf2, sizeof buf2, "  | RTK:%s ratio=%.2f 卫星=%d", st, cur.rtkRatio, cur.numSatsResult);
        text += buf2;
        color = cur.rtkStat == 1 ? ImVec4(0.3f, 1, 0.3f, 1) : ImVec4(1, 0.8f, 0.2f, 1);
    }

    static const SolverUi &solverUi(const GnssTask::SolverMode m) {
        static const SolverUi generic{&csvHeaderGeneric, &csvRowGeneric, &statusSuffixNone};
        static const SolverUi leo{&csvHeaderLeo, &csvRowLeo, &statusSuffixNone};
        static const SolverUi rtk{&csvHeaderRtk, &csvRowRtk, &statusSuffixRtk};
        switch (m) {
            case GnssTask::SolverMode::Leo: return leo;
            case GnssTask::SolverMode::Rtk: return rtk;
            default: return generic;   // SPP 与 PPP 的结果列完全同构
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

    static std::vector<std::pair<SatID, SkyPoint> > BuildCurrentSkyPoints(
        const std::shared_ptr<GnssTask::SolveTask> &task, const int sel) {
        std::vector<std::pair<SatID, SkyPoint> > curPts;
        std::lock_guard lk(task->mutex);
        if (sel < 0 || sel >= static_cast<int>(task->epochs.size())) return curPts;
        auto &ep = task->epochs[sel];
        return GuiCharts::buildCurSkyPoints(sel, ep.satIds.size(),
                                            [&](const size_t i) {
                                                GuiCharts::SkyView v;
                                                v.sat = ep.satIds[i];
                                                v.azimRad = ep.azimuths[i];
                                                v.elevRad = ep.elevations[i];
                                                v.used = ep.solved && !ep.rejected[i];
                                                return v;
                                            }, 0.0);
    }

    void RenderTaskImpl(const std::shared_ptr<GnssTask::SolveTask> &task);

    void RenderTask(const std::shared_ptr<GnssTask::SolverTaskBase> &job) {
        const auto core = job->solveResult();
        if (job->state == GnssTask::SolveTask::State::Config) {
            job->renderConfigPanel();
            return;
        }
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
        auto [epochCount, selectedIdx] = GuiHelpers::readEpochView(task);
        int selectedEpochLocal = selectedIdx;

        if (!isRealtime && busy) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f),
                               "正在处理 [%s]: 已解算 %d 历元",
                               task->fileName.c_str(), task->solvedCount.load());
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem("定位解算")) {
                GuiCharts::PosTabView v;
                v.epochCount = epochCount;
                v.selectedEpoch = &selectedEpochLocal;
                v.selectedSatIdx = &task->selectedSatIdx;
                v.refECEF = &task->refECEF;
                // 读 epochs/refECEF + 写 plotData：与 SolverThread 写方口径一致，
                // 先 mutex（保护 epochs/refECEF）再 plotMutex（保护 plotData），顺序相同故不会死锁。
                v.onRefChanged = [&] {
                    std::lock_guard lk(task->mutex);
                    std::lock_guard plk(task->plotMutex);
                    task->plotData.refreshENU(task->epochs, task->refECEF);
                };

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
                // epochCount==0 且仍在干活时，空态提示走"稍候"文案而不是"无解算历元"
                v.busy = busy;

                // 天顶图轨迹增量构建（独立于下方锁，内部自行持锁）
                if (epochCount > 0) EnsureSkyTracksBuilt(task);

                if (epochCount > 0 && selectedIdx >= 0) {
                    std::lock_guard lk(task->mutex);
                    auto &cur = task->epochs[selectedIdx]; //NOLINT

                    v.satRows = GuiCharts::buildSatRows(cur.satIds.size(),
                                                        [&](const size_t i) {
                                                            GuiCharts::SatRowView _v;
                                                            _v.sat = cur.satIds[i];
                                                            _v.used = !cur.rejected[i] && cur.solved;
                                                            _v.elevRad = cur.elevations[i];
                                                            _v.azimRad = cur.azimuths[i];
                                                            const bool haveXYZ = cur.satPVTs[i].p.squaredNorm() > 1.0;
                                                            _v.hasXYZ = haveXYZ;
                                                            if (haveXYZ) {
                                                                _v.x = cur.satPVTs[i].p[0];
                                                                _v.y = cur.satPVTs[i].p[1];
                                                                _v.z = cur.satPVTs[i].p[2];
                                                            }
                                                            return _v;
                                                        },
                                                        [&](GuiCharts::SatRow &sr, const size_t i) {
                                                            const auto it = cur.sppResult.postRes.find(cur.satIds[i]);
                                                            sr.extra.emplace_back(
                                                                "伪距残差(m)", it != cur.sppResult.postRes.end()
                                                                               ? fmt4(it->second)
                                                                               : std::string("-"));
                                                        });
                    v.hasSatRows = true;

                    if (task->selectedSatIdx >= 0 && task->selectedSatIdx < static_cast<int>(cur.satIds.size())) {
                        int si = task->selectedSatIdx;
                        v.showObsDetail = true;
                        v.detailSystem = cur.satIds[si].system;
                        v.detailObs = cur.allObs[si];
                    }

                    if (cur.solved) {
                        const auto &r = cur.sppResult;
                        v.solved = true;
                        v.xyz = r.xyz;
                        v.enu = XYZtoENU(r.xyz, task->refECEF);
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
                        v.newed = pp.newed;
                        v.fixedY = true; // SPP 收敛到米级，固定 Y 轴 ±1m 便于观察微小漂移
                        v.resT = pp.satResTimes;
                        v.resV = pp.satResVals;
                        if (isDone && !pp.resRangeReady) {
                            auto [lo, hi] = GuiHelpers::computeRobustResRange(pp.satResVals);
                            pp.resYlo = lo;
                            pp.resYhi = hi;
                            pp.resRangeReady = true;
                        }
                        v.robustRes = pp.resRangeReady;
                        v.resYlo = pp.resYlo;
                        v.resYhi = pp.resYhi;
                        v.sigmaPs = pp.sigmaPs;
                        v.sigmaVs = pp.sigmaVs;
                        sigmaVNonZero = std::any_of(pp.sigmaVs.begin(), pp.sigmaVs.end(),
                                                    [](const double x) { return x > 1e-6; });
                        v.pdops = pp.pdops;
                    }
                    {
                        std::lock_guard lk(task->mutex);
                        v.skyTracks = task->skyTracks;
                        // 仅当实际估计了速度（>5% 历元有非零速度）时才显示 SigmaV；
                        // PPP / 静态 SPP 速度为 0，避免画出单个尖峰。
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
                        v.leoTraj = task->leoTraj;
                        v.leoRefTraj = task->leoRefTraj;
                        v.gnssVis = task->gnssVis;
                        v.gnssTraj = task->gnssTraj;
                        v.rtnTimes = task->rtnTimes;
                        v.rtnR = task->rtnR;
                        v.rtnT = task->rtnT;
                        v.rtnN = task->rtnN;
                        v.rtnD3 = task->rtnD3;
                    }
                }

                // RTK 等解算系统在状态行追加自己的状态串（固定/浮动 + ratio）；
                // 各系统的差异由 solverUi 描述符承载，这里不再出现按 mode 的分支。
                if (epochCount > 0 && selectedIdx >= 0) {
                    std::lock_guard lk(task->mutex);
                    const SolverUi &ui = solverUi(task->mode);
                    ui.statusSuffix(v.statusText, v.statusColor, task->epochs[selectedIdx]);
                }

                v.isDone = isDone || isRealtime;
                v.onExportCsv = [&] {
                    ExportCsv(task);
                };

                try {
                    GuiCharts::RenderPositioningTab(v);
                } catch (const std::exception &e) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "渲染异常: %s", e.what());
                } catch (...) {
                    ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "渲染发生未知异常");
                }
                // 把局部选中的历元写回共享状态（锁内），保证下一帧 readEpochView 读取的是一致值。
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
            // 附加质量分析页（RTK：基准站 QC）。每个附加页自带 qcInput/qcReport，复用同一套渲染。
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
        std::string dn = GuiHelpers::baseName(task->fileName);
        if (const auto dp = dn.rfind('.'); dp != std::string::npos) dn = dn.substr(0, dp);
        dn += "_" + std::string(modeLabel(task->mode)) + "_result.csv";
        if (dn.size() >= MAX_PATH) dn = std::string(modeLabel(task->mode)) + "_result.csv";
        std::wstring path;
        if (!GuiHelpers::saveCSVDialog(dn, path)) return;

        std::lock_guard lk(task->mutex);
        std::ofstream out(std::filesystem::path(path), std::ios::out);
        if (!out.is_open()) return;
        const SolverUi &ui = solverUi(task->mode);
        ui.csvHeader(out, *task);
        for (auto &r: task->epochs) {
            out << r.week << ',' << std::fixed << std::setprecision(3) << r.sow << ',';
            if (r.solved) ui.csvRow(out, r, *task);
            out << '\n';
        }
    }
} // namespace GuiSolverPanel
