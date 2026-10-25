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
#include <iterator>
#include <set>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>
#include "GuiCharts.h"
#include "Csv.h"           // 通用 CSV 轮子（读写）；GNSS 导出内容在 SolverTask.h，由 task->exportCsv 调用
#include "StringUtils.h"
#include "Log.h"
#include "widgets/tools/GuiInsPanel.h"

namespace Solvers::SolverPanel {
    // 渲染只需可视化，不需要逐历元全量点：LEO 一天 86400 历元 ×8 星 ≈ 69 万条(数十 MB)，
    // 若每帧在锁内整份拷贝，60fps 下就是 GB/s 级拷贝并长期占锁饿死解算线程
    // （实测 GUI 比 harness 慢约 30×）。故按最大点数等间隔抽样后再拷。
    // 用模板以便 rtnTimes 与 rtnR/T/N 用**同一套采样索引**（长度须保持一致，否则绘图错位）。
    //
    // 步长按 2 的幂量化（只增不减、无状态）：旧写法 step=size/maxPts 逐帧重算，size 每涨 1
    // 全体采样索引就漂移一次，已绘的点每帧换一拨 → 解算途中曲线"游动"（视觉抖动/闪烁）。
    // 量化后 src 在同一个 2 倍区间内增长时索引集合不变；跨 2 的幂边界才整体重采样一次
    // （86400 点全天最多 log2(86400/3000)≈5 次，且旧点集是新点集的子集，无跳变感）。
    template <typename T>
    static void copySampled(const std::vector<T> &src, std::vector<T> &dst,
                            const size_t maxPts = 3000) {
        if (src.size() <= maxPts) { dst = src; return; }
        size_t step = 1;
        while (step * maxPts < src.size()) step <<= 1; // ≥ ceil(size/maxPts) 的最小 2 的幂
        dst.clear();
        dst.reserve(src.size() / step + 2);
        for (size_t i = 0; i < src.size(); i += step)
            dst.push_back(src[i]);
        if ((src.size() - 1) % step != 0)
            dst.push_back(src.back()); // 保留末点，保证画到最新历元
    }

    // 逐星序列（残差时序 / 天顶图轨迹）：每颗星内部独立等距抽样。
    // 同一颗星的 t 与 v 序列等长，故用同一 maxPts 得到同一套索引，绘图不会错位。
    template <typename T>
    static void copySampledMap(const std::map<SatID, std::vector<T> > &src,
                               std::map<SatID, std::vector<T> > &dst, const size_t maxPts) {
        dst.clear();
        for (const auto &[sat, v]: src)
            copySampled(v, dst[sat], maxPts);
    }

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

    // ------------------------------------------------------------------
    // 各解算系统的"结果呈现"差异只有一处：状态行后缀（CSV 列定义已归到 SolveTask::exportCsv，
    // 由解算任务域按 mode 负责，见 SolverTask.h）。主流程只剩通用骨架。
    // ------------------------------------------------------------------
    struct SolverUi {
        // 构建右侧面板「解状态」行（固定/浮点 + ratio）；SPP 用 statusNone 留空（不显示该块）
        void (*solveStatusLine)(std::string &, ImVec4 &, const GnssTask::EpochData &);
    };

    // CSV 写出委托给 SolveTask::exportCsv（业务 schema 在 SolverTask.h，CSV 轮子在 Csv.h），
    // GUI 与 CLI 共用同一份定义，不再在此处硬编码。

    static const char *solveStatLabel(unsigned char stat) {
        return stat == 1 ? "固定(Fixed)" : "浮点(Float)";
    }
    static ImVec4 solveStatColor(unsigned char stat) {
        return stat == 1 ? ImVec4(0.3f, 1, 0.3f, 1) : ImVec4(1, 0.8f, 0.2f, 1);
    }

    // 不显示解状态块（SPP 无模糊度固定概念）
    static void statusNone(std::string &, ImVec4 &, const GnssTask::EpochData &) {}

    // RTK：用 RTK 专用字段（与 sppResult 同源，rtkStat/rtkRatio 由 SolveDriver::feedRtk 复制而来）
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
        static const SolverUi generic{&statusNone};     // SPP
        static const SolverUi ppp{&statusAmbig};
        static const SolverUi leo{&statusAmbig};
        static const SolverUi rtk{&statusRtk};
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
        return Solvers::Charts::buildCurSkyPoints(sel, ep.satIds.size(),
                                            [&](const size_t i) {
                                                Solvers::Charts::SkyView v;
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
        auto [epochCount, selectedIdx] = Solvers::Helpers::readEpochView(task);
        int selectedEpochLocal = selectedIdx;

        if (!isRealtime && busy) {
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f),
                               "正在处理 [%s]: 已解算 %d 历元",
                               task->fileName.c_str(), task->solvedCount.load());
        }

        if (ImGui::BeginTabBar("##qa_tabs")) {
            if (ImGui::BeginTabItem("定位解算")) {
                Solvers::Charts::PosTabView v;
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

                    // 按启用星座过滤卫星列表：筛选后只显示勾选系统的卫星（其余不出现在列表里）
                    std::vector<int> visIdx;
                    visIdx.reserve(static_cast<int>(cur.satIds.size()));
                    for (int i = 0; i < static_cast<int>(cur.satIds.size()); ++i)
                        if (task->enabledSystems.count(cur.satIds[i].system)) visIdx.push_back(i);

                    v.satRows = Solvers::Charts::buildSatRows(visIdx.size(),
                                                        [&](const size_t i) {
                                                            const int oi = static_cast<int>(visIdx[i]);
                                                            Solvers::Charts::SatRowView _v;
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
                                                        [&](Solvers::Charts::SatRow &sr, const size_t i) {
                                                            const int oi = static_cast<int>(visIdx[i]);
                                                            const auto it = cur.sppResult.postRes.find(cur.satIds[oi]);
                                                            sr.extra.emplace_back(
                                                                "伪距残差(m)", it != cur.sppResult.postRes.end()
                                                                               ? fmt4(it->second)
                                                                               : std::string("-"));
                                                        });
                    v.hasSatRows = true;

                    // 选中卫星按过滤后的行号映射回原始索引（selectedSatIdx 存的是过滤行号）
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

                        // ---- 三表（坐标表 / 参考+ENU(RTN) 表 / DOP+状态表）数据 ----
                        const bool isRtkMode = (task->mode == GnssTask::SolverMode::Rtk);
                        const bool isLeoMode = (task->mode == GnssTask::SolverMode::Leo);
                        const bool canFixed  = isRtkMode || isLeoMode ||
                                               (task->mode == GnssTask::SolverMode::Ppp);
                        const bool isFixed = isRtkMode ? (cur.rtkStat == 1) : (r.stat == 1);
                        v.hasFixedDistinction = canFixed;
                        v.hasFixed = isFixed;

                        // 浮点/固定 ECEF 显示值（RTK 浮点=基站+基线浮点；PPP/LEO 用 xyzFloat/xyzFixed）
                        Eigen::Vector3d xyzFloat = r.xyzFloat;
                        Eigen::Vector3d xyzFixed = (r.xyzFixed.squaredNorm() > 1.0) ? r.xyzFixed : r.xyz;
                        if (isRtkMode) {
                            xyzFloat = cur.rtkFloatXyz;
                            xyzFixed = (cur.rtkStat == 1) ? r.xyz : cur.rtkFloatXyz;
                        }
                        if (xyzFloat.squaredNorm() < 1.0) xyzFloat = r.xyz;   // 兜底：无浮点解时用当前解
                        v.xyzFloatDisp = xyzFloat;
                        v.xyzFixedDisp = xyzFixed;
                        v.sigmaFloatDisp = (r.sigmaPFloat > 0) ? r.sigmaPFloat : r.sigmaP;
                        v.sigmaFixedDisp = (r.sigmaPFixed > 0) ? r.sigmaPFixed : r.sigmaP;

                        // 基线（RTK 专用）
                        if (isRtkMode) {
                            v.hasBaseline = true;
                            v.baseFloatDisp = cur.rtkBaseFloat;
                            if (cur.rtkStat == 1) v.baseFixedDisp = r.xyz - task->refECEF;
                        }

                        // 速度
                        if (r.vel.squaredNorm() > 1e-4) {
                            v.hasVelDisp = true;
                            v.velDisp = r.vel;
                            v.sigmaVDisp = r.sigmaV;
                        }

                        // 参考 + ENU/RTN 表（仅区分固定/浮点的模式；LEO 无参考轨道不显示）
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
                        // 等距抽样拷贝：24h 数据整份拷约 35MB/帧（含 4 张逐星残差 map），
                        // 60fps 下是 GB/s 级 memcpy 且长期占锁饿死解算线程。抽样后降到 ~1MB/帧。
                        // 所有与 times 共用 X 轴的序列必须用**同一 maxPts**（索引一致才不错位）；
                        // 短数据（≤maxPts）仍是全量拷贝，无精度损失。
                        constexpr size_t kPlotPts = 3000;
                        copySampled(pp.times, v.times, kPlotPts);
                        copySampled(pp.enu_e, v.enu_e, kPlotPts);
                        copySampled(pp.enu_n, v.enu_n, kPlotPts);
                        copySampled(pp.enu_u, v.enu_u, kPlotPts);
                        v.newed = pp.newed;
                        // 一次性语义：取走即清。GuiCharts 的渲染函数本意是通过引用清 newed
                        // （见 GuiCharts.cpp:185/248/340），但 PosTabView 以 const 传入、调用处
                        // 传的是局部拷贝，pp.newed 永远无人复位 → 永真 → ENU/Ratio/DOP/σ 四张图
                        // 每帧都执行 SetupAxisLimits(X1,0,last,ImPlotCond_Always)：用户一缩放/平移，
                        // 下一帧就被拽回，观感就是"图一直在闪/跳"。在此清零后，X 轴跟随最新历元
                        // 每个新数据批次只生效一次，交互权交还用户。
                        pp.newed = false;
                        v.fixedY = true; // SPP 收敛到米级，固定 Y 轴 ±1m 便于观察微小漂移
                        // 逐星残差：每星独立抽到 1500 点（残差图自带 X 轴，不与其他序列共用）
                        constexpr size_t kResPts = 1500;
                        copySampledMap(pp.satResTimes, v.resT, kResPts);
                        copySampledMap(pp.satResVals, v.resV, kResPts);
                        copySampledMap(pp.satResPhaseTimes, v.resPhaseT, kResPts);
                        copySampledMap(pp.satResPhaseVals, v.resPhaseV, kResPts);
                        if (isDone && !pp.resRangeReady) {
                            auto [lo, hi] = Solvers::Helpers::computeRobustResRange(pp.satResVals);
                            pp.resYlo = lo;
                            pp.resYhi = hi;
                            pp.resRangeReady = true;
                        }
                        v.robustRes = pp.resRangeReady;
                        v.resYlo = pp.resYlo;
                        v.resYhi = pp.resYhi;
                        copySampled(pp.sigmaPs, v.sigmaPs, kPlotPts);
                        copySampled(pp.sigmaPFloatVals, v.sigmaPFloatVals, kPlotPts);
                        copySampled(pp.sigmaVs, v.sigmaVs, kPlotPts);
                        sigmaVNonZero = std::any_of(pp.sigmaVs.begin(), pp.sigmaVs.end(),
                                                    [](const double x) { return x > 1e-6; });
                        copySampled(pp.pdops, v.pdops, kPlotPts);
                        copySampled(pp.gdops, v.gdops, kPlotPts);
                        copySampled(pp.hdops, v.hdops, kPlotPts);
                        copySampled(pp.vdops, v.vdops, kPlotPts);
                        copySampled(pp.tdops, v.tdops, kPlotPts);
                        copySampled(pp.ratioVals, v.ratioVals, kPlotPts);
                        copySampled(pp.float_e, v.float_e, kPlotPts);
                        copySampled(pp.float_n, v.float_n, kPlotPts);
                        copySampled(pp.float_u, v.float_u, kPlotPts);
                        v.showRatio = (task->mode == GnssTask::SolverMode::Rtk ||
                                       task->mode == GnssTask::SolverMode::Ppp ||
                                       task->mode == GnssTask::SolverMode::Leo);
                        v.hasFloatSeries = v.showRatio;
                        // RTK：参考坐标即基站坐标，解算页只读展示（不提供编辑）
                        v.showBaseXyz = (task->mode == GnssTask::SolverMode::Rtk);
                        v.isRtk = (task->mode == GnssTask::SolverMode::Rtk);
                        v.baseXyz = &task->refECEF;
                    }
                    {
                    std::lock_guard lk(task->mutex);
                    // 天顶图轨迹：24h×8 星约 11MB/帧，抽到每星 1500 点（断点判据已改自适应，
                    // 见 Solvers::Charts::RenderSkyplot，否则抽样后的间隔会被误判成断点）
                    copySampledMap(task->skyTracks, v.skyTracks, 1500);
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

                    // 固定率（PPP/RTK/LEO）：实时统计固定解占已解算历元的比例。
                    // 固定判据与状态行一致：RTK 用 rtkStat==1；PPP/LEO 用 sppResult.stat==1（0 视为浮点）。
                    if (v.showRatio && epochCount > 0) {
                        int fixedCnt = 0, totalCnt = 0;
                        {
                            std::lock_guard lk(task->mutex);
                            for (const auto &ep : task->epochs) {
                                if (!ep.solved) continue;
                                ++totalCnt;
                                const bool isFixed = (task->mode == GnssTask::SolverMode::Rtk)
                                        ? (ep.rtkStat == 1)
                                        : (ep.sppResult.stat == 1);
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
                        // 抽样拷贝：整份拷 86400 条会把解算线程饿死（见 copySampled 注释）
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

                // 右侧面板「解状态」块（固定/浮点 + ratio）：PPP/LEO/RTK 显示，SPP 不显示。
                // 原 RTK 状态行里的 " | RTK:固定/浮点 ratio=..." 已移至此，状态行统一只显示"共 N 个历元"。
                if (epochCount > 0 && selectedIdx >= 0 && v.solved) {
                    std::lock_guard lk(task->mutex);
                    const SolverUi &ui = solverUi(task->mode);
                    std::string line; ImVec4 col;
                    ui.solveStatusLine(line, col, task->epochs[selectedIdx]);
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
        std::string dn = Solvers::Helpers::baseName(task->fileName);
        if (const auto dp = dn.rfind('.'); dp != std::string::npos) dn = dn.substr(0, dp);
        dn += "_" + std::string(modeLabel(task->mode)) + "_result.csv";
        if (dn.size() >= MAX_PATH) dn = std::string(modeLabel(task->mode)) + "_result.csv";
        std::wstring path;
        if (!Solvers::Helpers::saveCSVDialog(dn, path)) return;
        LOG_INFO << "[导出] CSV (" << modeLabel(task->mode) << ") → "
                 << std::filesystem::path(path).string();

        // 在锁内组装（读取 epochs / refECEF），写出文件放到锁外。GNSS 列布局由 SolveTask 负责。
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
} // namespace Solvers::SolverPanel
