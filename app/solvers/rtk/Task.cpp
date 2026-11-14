#include "Task.h"
#include "core/equ/Precision.h"
#include "core/geom/CoordConvert.h"
#include "FileUtils.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ostream>
#include <string>

#include "Decimator.h"
#include "Log.h"
#include "Observer.h"
#include "StringUtils.h"
#include "solvers/common/Config.h"
#include "solvers/rtk/Pipeline.h"
#include "solvers/rtk/Realtime.h"
#include "util/Clock.h"

namespace Solvers::RTK {
    void Task::applyConfig(const SolverOptions &cfg) {
        mode = Mode::RTK;
        opts = cfg;
        setupTruth(cfg);
        applyObsPath(cfg.roverPath);
        // 双站任务展示两个文件名，覆写 applyObsPath 的单文件派生。
        fileName = File::baseName(cfg.roverPath) + " / " + File::baseName(cfg.basePath);
        baseObsPathBuf = cfg.basePath;
        navPathBuf = cfg.navPath;
        // ini 中的基准坐标优先于 RINEX 表头概略坐标。
        const Eigen::Vector3d cfgBase(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ);
        useBaseCoord = cfgBase.norm() > 1e3;
        baseXyz = useBaseCoord ? cfgBase : Eigen::Vector3d::Zero();
        rtEndpoints.resize(2);
        rtEndpoints[0].ip = cfg.baseIp;
        rtEndpoints[0].port = std::to_string(cfg.basePort);
        rtEndpoints[1].ip = cfg.roverIp;
        rtEndpoints[1].port = std::to_string(cfg.roverPort);
    }

    bool Task::solvePair(const ObsData &base, const ObsData &rover, EphemerisTable &eph) {
        bool ok = false;
        try {
            ok = rtk.processEpoch(base, rover, eph) && rtk.result.numSats > 0;
        } catch (const std::exception &e) {
            LOG_ERROR << "[RTK] 历元异常: " << e.what();
        } catch (...) {
            LOG_ERROR << "[RTK] 历元未知异常";
        }
        Epoch data;
        data.getFromObs(rover);
        if (ok) data.getFromRTK(rtk);
        // 失败也提交空解：倒序趟要按下标回填，epochs 必须与配对序列一一对应。
        const Eigen::Vector3d b = rtk.mBaseXyz;
        commitEpoch(std::move(data), [&](int, Epoch &) {
            if (!initializedRefECEF && b.squaredNorm() > 1e12) {
                refECEF = XYZ(b.x(), b.y(), b.z());
                initializedRefECEF = true;
            }
        });
        return ok;
    }

    void Task::onReset() {
        {
            std::lock_guard lk(pairMtx);
            std::queue<RTKPair> empty;
            std::swap(pairQueue, empty);
        }
        rtk = ::RTK{};
        navTable = EphemerisTable{};
        replay.clear();
    }

    void Task::wakeQueues() {
        Solvers::Task::wakeQueues();
        /// 与配对队列的等待侧共用一把锁后再通知。
        std::lock_guard lk(pairMtx);
        pairCv.notify_all();
        pairSpaceCv.notify_all();
    }

    void Task::readBody() {
        try {
            view.qcTabLabel = "质量分析[流动站]";
            auto baseTab = std::make_shared<QcTab>();
            baseTab->label = "质量分析[基准站]";
            baseTab->fileName = File::baseName(baseObsPathBuf);
            baseTab->view.clearQCInput(opts.sampleInterval);

            const Observer::Decimator dec(opts.sampleInterval);
            std::string pipeErr;
            const int nPair = pairFiles(
                baseObsPathBuf, view.obsPathBuf, navPathBuf,
                opts.sampleInterval, navTable,
                [&] { return shouldStop(); },
                [&](const ObsData &base, const ObsData &rover,
                    const std::shared_ptr<EphemerisTable> &eph, EphemerisTable *) {
                    view.pushQCInput(rover, rover.weekSecond.sow);
                    baseTab->view.pushQCInput(base, base.weekSecond.sow);
                    view.markQcDirty();
                    baseTab->view.markQcDirty();
                    if (!dec.accept(rover.weekSecond.sow)) return;
                    {
                        std::unique_lock lk(pairMtx);
                        pairSpaceCv.wait(lk, [&] {
                            return pairQueue.size() < queueDepth || stop.load() || solvingDone.load();
                        });
                        if (stop.load() || solvingDone.load()) return;
                        pairQueue.push({base, rover, eph});
                    }
                    pairCv.notify_one();
                }, pipeErr);
            if (nPair < 0) {
                failRead(pipeErr);
                loading = false;
                return;
            }
            {
                std::lock_guard lk(pairMtx);
                readDone = true;
            }
            wakeQueues();
            view.endQCInput();
            baseTab->view.endQCInput();
            {
                std::lock_guard lk(mutex);
                view.extraQcTabs.push_back(std::move(baseTab));
            }
        } catch (const std::exception &e) {
            failRead("RTK 读取错误: " + std::string(e.what()));
        } catch (...) {
            failRead("RTK 读取未知异常");
        }
        loading = false;
    }

    void Task::solveBody() {
        const Clock::Stopwatch total;

        // 快照一份再灌给解算器：解算期间 GUI 理论上还能改 opts，一趟解算的口径必须钉死。
        const SolverOptions o = opts;
        filterMode = o.filter;
        rtk.applyOptions(o);
        rtk.mBaseXyzUser = useBaseCoord ? baseXyz : Eigen::Vector3d::Zero();
        LOG_INFO << "[RTK] 开始: 截止高度角 " << o.cutoffDeg << "°, 滤波方向 " << filterLabel(o.filter);

        replay.clear();
        try {
            runForward();
            if (filterHasBackward(filterMode)) runBackward();
        } catch (const std::exception &e) {
            setError(std::string("RTK 解算错误: ") + e.what());
        } catch (...) {
            setError("RTK 解算未知异常");
        }
        finishSolving();

        std::size_t nSolved = 0;
        {
            std::lock_guard lk(mutex);
            nSolved = epochs.size();
        }
        LOG_INFO << "[RTK] 结束: " << nSolved << " 历元, 耗时 " << total.ms() << " ms";
    }

    void Task::runForward() {
        const bool solve = filterSolvesForward(filterMode);
        const bool keep = filterHasBackward(filterMode);
        solvePhase = solve ? SolvePhase::Forward : SolvePhase::Scan;
        while (true) {
            RTKPair item;
            {
                std::unique_lock lk(pairMtx);
                pairCv.wait(lk, [&] {
                    return !pairQueue.empty() || readDone.load() || stop.load();
                });
                if (pairQueue.empty() && (readDone.load() || stop.load())) break;
                if (pairQueue.empty()) continue;
                item = std::move(pairQueue.front());
                pairQueue.pop();
            }
            pairSpaceCv.notify_one();
            if (stop || epochLimitReached()) break;

            EphemerisTable *eph = item.eph ? item.eph.get() : &navTable;
            if (solve) {
                solvePair(item.base, item.rover, *eph);
            } else {
                // 纯后向档正序只扫不解，仍要占位，倒序趟才按下标对得上
                Epoch data;
                data.getFromObs(item.rover);
                commitEpoch(std::move(data));
            }
            if (keep) replay.push_back(std::move(item));
        }
    }

    void Task::runBackward() {
        std::size_t n = 0;
        {
            std::lock_guard lk(mutex);
            n = (std::min)(replay.size(), epochs.size());
        }
        if (n == 0) return;

        const Clock::Stopwatch backwardCost;
        LOG_INFO << "[RTK] 倒序滤波 " << n << " 历元 (" << filterLabel(filterMode) << ")";
        solvePhase = SolvePhase::Backward;
        solveEpoch = 0;
        // 纯后向档正序没解过，参考点由倒序趟自己重建
        if (!filterSolvesForward(filterMode)) initializedRefECEF = false;

        // 与 SPP 的倒序趟同理：新解算器不继承前向状态，从末历元重新收敛
        ::RTK bwd;
        bwd.applyOptions(opts);
        bwd.mBaseXyzUser = useBaseCoord ? baseXyz : Eigen::Vector3d::Zero();

        const auto publish = [this] {
            std::lock_guard lk(mutex);
            std::lock_guard plk(view.plotMutex);
            view.plotData.replaceAll(epochs, refECEF, mode);
            view.skyTracks.clear();
            view.skyTracksBuilt = 0;
        };
        const std::size_t step = (std::max)(std::size_t(1), n / 40);

        for (std::size_t i = n; i-- > 0;) {
            if (stop.load()) break;
            solveEpoch = static_cast<int>(n - i);
            EphemerisTable *eph = replay[i].eph ? replay[i].eph.get() : &navTable;
            bool ok = false;
            try {
                ok = bwd.processEpoch(replay[i].base, replay[i].rover, *eph) && bwd.result.numSats > 0;
            } catch (const std::exception &e) {
                LOG_ERROR << "[RTK] 倒序历元异常: " << e.what();
            } catch (...) {
                LOG_ERROR << "[RTK] 倒序历元未知异常";
            }
            Epoch data;
            data.getFromObs(replay[i].rover);
            if (ok) data.getFromRTK(bwd);
            {
                std::lock_guard lk(mutex);
                if (i < epochs.size()) mergeEpoch(epochs[i], data);
                if (!initializedRefECEF && bwd.mBaseXyz.squaredNorm() > 1e12) {
                    refECEF = XYZ(bwd.mBaseXyz.x(), bwd.mBaseXyz.y(), bwd.mBaseXyz.z());
                    initializedRefECEF = true;
                }
            }
            if ((n - i) % step == 0) publish();
        }
        publish();
        {
            std::lock_guard lk(mutex);
            for (auto &ep: epochs) {
                ep.sppResult.postRes.release();
                ep.sppResult.postResPhase.release();
            }
        }
        LOG_INFO << "[RTK] 倒序完成，耗时 " << backwardCost.ms() << " ms";
        replay.clear();
        replay.shrink_to_fit();
    }

    /// 两趟合并：整周解优先（固定解是整周约束的产物，掺浮点只会变差），两趟都浮点才按位置 σ 融合。
    void Task::mergeEpoch(Epoch &dst, const Epoch &bwd) {
        if (!bwd.solved) return;
        if (!dst.solved) {
            dst = bwd;
            return;
        }
        if (dst.rtkStat == 1) return;
        if (bwd.rtkStat == 1) {
            dst = bwd;
            return;
        }

        // 只存了对角 σ，逐轴加权即可（完整协方差在解算器里，没进 Epoch）
        const Eigen::Vector3d sf = dst.sppResult.sigmaXYZ;
        const Eigen::Vector3d sb = bwd.sppResult.sigmaXYZ;
        Eigen::Vector3d fused = dst.sppResult.xyz;
        Eigen::Vector3d sigma = sf;
        bool any = false;
        for (int k = 0; k < 3; ++k) {
            const double wf = sf[k] > 1e-6 ? 1.0 / (sf[k] * sf[k]) : 0.0;
            const double wb = sb[k] > 1e-6 ? 1.0 / (sb[k] * sb[k]) : 0.0;
            if (wf + wb <= 0.0) continue;
            fused[k] = (wf * dst.sppResult.xyz[k] + wb * bwd.sppResult.xyz[k]) / (wf + wb);
            sigma[k] = std::sqrt(1.0 / (wf + wb));
            any = true;
        }
        if (!any) return;

        // 基线是相对基准站的：位置挪了，基线跟着挪，否则导出时基准站坐标会漂
        Eigen::Vector3d basePos = Eigen::Vector3d::Zero();
        if (dst.rtkFloatXyz.squaredNorm() > 1e12) basePos = dst.rtkFloatXyz - dst.rtkBaseFloat;
        dst.sppResult.xyz = fused;
        dst.sppResult.xyzFloat = fused;
        dst.sppResult.xyzFixed = fused;
        dst.sppResult.blh = Coord::XYZToBLH(fused, Frame::WGS84);
        dst.sppResult.sigmaXYZ = sigma;
        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
        for (int k = 0; k < 3; ++k) Q(k, k) = sigma[k] * sigma[k];
        dst.sppResult.sigmaP = positionSigma(Q, 1.0);
        dst.sppResult.sigmaPFloat = dst.sppResult.sigmaP;
        dst.rtkFloatXyz = fused;
        if (basePos.squaredNorm() > 1e12) dst.rtkBaseFloat = fused - basePos;
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::RTKSession>(
            std::static_pointer_cast<Task>(shared_from_this()),
            rtEndpoints[0], rtEndpoints[1]);
    }

    /// RTK 的浮点解不在 res.xyzFloat 里：基线解 + 基准站坐标才是它的位置。
    void Task::exportRow(const std::size_t i, Export::Row &row) const {
        Solvers::Task::exportRow(i, row);
        using F = Export::Field;
        if (i >= epochs.size() || !epochs[i].solved) return;
        const Epoch &ep = epochs[i];
        const Eigen::Vector3d ref = exportRef(ep);
        const Eigen::Vector3d xyz = ep.rtkFloatXyz.squaredNorm() > 1.0 ? ep.rtkFloatXyz : ref + ep.rtkBaseFloat;
        row.set(F::Xf, xyz.x());
        row.set(F::Yf, xyz.y());
        row.set(F::Zf, xyz.z());
        if (ref.squaredNorm() > 1.0) {
            const ENU enu = Coord::XYZToENU(xyz, ref);
            row.set(F::Ef, enu.E());
            row.set(F::Nf, enu.N());
            row.set(F::Uf, enu.U());
            row.set(F::Rms3dF, std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U()));
        }
        // 基线列始终相对基准站，与真值无关（REF/偏差/3dRMS 列随真值走，无真值时整体剔除）。
        row.set(F::BaseXf, ep.rtkBaseFloat.x());
        row.set(F::BaseYf, ep.rtkBaseFloat.y());
        row.set(F::BaseZf, ep.rtkBaseFloat.z());
        row.set(F::BaseLenF, ep.rtkBaseFloat.norm());
        const Eigen::Vector3d basePos = xyz - ep.rtkBaseFloat; // 基准站坐标：浮点位置 - 浮点基线
        if (basePos.squaredNorm() > 1.0e12) {
            // 矢量换站心系只转不挪：给锚点加上矢量再求相对锚点的 ENU。
            const ENU blfEnu = Coord::XYZToENU(xyz, basePos);
            row.set(F::BaseEf, blfEnu.E());
            row.set(F::BaseNf, blfEnu.N());
            row.set(F::BaseUf, blfEnu.U());
        }
        if (ep.rtkStat == 1) {
            const Eigen::Vector3d fixedBl = ep.rtkFixedBaseline.squaredNorm() > 1e-8
                                                ? ep.rtkFixedBaseline
                                                : ep.sppResult.xyz - basePos;
            row.set(F::BaseXk, fixedBl.x());
            row.set(F::BaseYk, fixedBl.y());
            row.set(F::BaseZk, fixedBl.z());
            row.set(F::BaseLenK, fixedBl.norm());
            if (basePos.squaredNorm() > 1.0e12) {
                const ENU blkEnu = Coord::XYZToENU(XYZ(basePos + fixedBl), basePos);
                row.set(F::BaseEk, blkEnu.E());
                row.set(F::BaseNk, blkEnu.N());
                row.set(F::BaseUk, blkEnu.U());
            }
        }
    }

    std::vector<Export::Col> Task::exportFields(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return {}; // 根数只有低轨轨道有
        const std::vector<Export::Col> c = Export::floatFixColumns(false, true);
        // 基线列不受影响；REF/偏差/3dRMS 这些「相对参考」的列没挂真值就剔除。
        return exportRefReady() ? c : Export::withoutRefColumns(c);
    }

    std::string Task::exportStem() const {
        const std::string rover = File::stripExt(File::baseName(view.obsPathBuf));
        const std::string base = File::stripExt(File::baseName(baseObsPathBuf));
        if (rover.empty() || base.empty()) return Solvers::Task::exportStem(); // 实时流没有文件名
        return rover + "-" + base;
    }

    void Task::writeCsvParams(CSV::Meta &m) const {
        m.flag("使用 AR 固定", opts.rtk.useAR);
        m.flag("完整协方差", opts.rtk.fullCovariance);
        m.num("Ratio 阈值", opts.rtk.arThreshold);
        m.flag("使用周跳探测", opts.rtk.useCycleSlip);
        m.flag("使用 GF 周跳", opts.rtk.useGfCycleSlip);
        m.num("GF 周跳阈值倍数", opts.rtk.gfSlipMult);
        m.num("相位高度角加权下限 (sinE)", opts.rtk.phaseWeight.minSinElev);
        m.num("伪距:相位 σ 比", opts.rtk.codeOverPhaseSigma);
        m.integer("双差最少卫星数", opts.rtk.minSats);
        m.num("相位高度角模型 a", opts.rtk.phaseWeight.a);
        m.num("相位高度角模型 b", opts.rtk.phaseWeight.b);
        m.flag("排除北斗 GEO 参考星", opts.rtk.excludeGeoRef);
        m.xyz("基准站坐标 XYZ", opts.baseCoordX, opts.baseCoordY, opts.baseCoordZ);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
