#include "Task.h"
#include "FileUtils.h"

#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "core/geom/CoordConvert.h"
#include "core/geom/CoordStruct.h"
#include "Decimator.h"
#include "Log.h"
#include "Observer.h"
#include "SPP.h"
#include "StringUtils.h"
#include "core/atmo/Troposphere.h"
#include "io/ObsSource.h"
#include "io/ProductLoad.h"
#include "solvers/common/Core.h"
#include "solvers/spp/Realtime.h"
#include "util/Clock.h"

namespace Solvers::SPP {
    namespace {
        /// 把解算线程的逐帧取数包装成惰性序列：纯前向档只留当前帧，倒序档逐帧留存供按索引重放。
        class ObsFeed {
        public:
            using Pull = std::function<bool(Observer::Item &)>;

            ObsFeed(Pull pull, double interval, int limit, bool keepAll)
                : mPull(std::move(pull)), mDec(interval), mLimit(limit), mKeepAll(keepAll) {}

            bool ensure(const std::size_t index) {
                while (index >= mReady && !mDone)
                    if (!pullAccepted()) mDone = true;
                return !mDone;
            }

            [[nodiscard]] std::size_t count() const { return mReady; }

            /// 单槽模式下只允许取当前帧；两趟的 prepare/obs/回填都在同一帧内完成。
            [[nodiscard]] Observer::Item &at(const std::size_t index) {
                return mKeepAll ? mItems[index] : mCurrent;
            }

        private:
            bool pullAccepted() {
                while (mPull(mScratch)) {
                    if (mLimit > 0 && static_cast<int>(mReady) >= mLimit) return false;
                    if (!mDec.accept(mScratch.obs.weekSecond.sow)) continue;
                    if (mKeepAll) mItems.push_back(std::move(mScratch));
                    else mCurrent = std::move(mScratch);
                    ++mReady;
                    return true;
                }
                return false;
            }

            Pull mPull;
            Observer::Decimator mDec;
            int mLimit;
            bool mKeepAll;
            bool mDone = false;
            std::size_t mReady = 0;
            Observer::Item mScratch;
            Observer::Item mCurrent;
            std::vector<Observer::Item> mItems;
        };

        using F = Export::Field;

        std::vector<Export::Col> sppFields() {
            return {
                {F::Week, "Wk"}, {F::Tow, "SOW"}, {F::Stat, "Status"},
                {F::X, "ECEF-X/m", "ECEF-XYZ"},
                {F::Y, "ECEF-Y/m", "ECEF-XYZ"},
                {F::Z, "ECEF-Z/m", "ECEF-XYZ"},
                {F::RefX, "REF-X/m", "REF-XYZ"},
                {F::RefY, "REF-Y/m", "REF-XYZ"},
                {F::RefZ, "REF-Z/m", "REF-XYZ"},
                {F::E, "EAST/m", "ENU"},
                {F::N, "NORTH/m", "ENU"},
                {F::U, "UP/m", "ENU"},
                {F::Lat, "B/deg", "BLH"},
                {F::Lon, "L/deg", "BLH"},
                {F::H, "H/m", "BLH"},
                {F::Vx, "VX/m", "Vel-XYZ"},
                {F::Vy, "VY/m", "Vel-XYZ"},
                {F::Vz, "VZ/m", "Vel-XYZ"},
                {F::Pdop, "PDOP"}, {F::Gdop, "GDOP"}, {F::Hdop, "HDOP"},
                {F::Vdop, "VDOP"}, {F::Tdop, "TDOP"},
                {F::SigmaP, "SigmaP"}, {F::SigmaV, "SigmaV"},
                {F::Sats, "SatCount"}, {F::Rms3dF, "3dRMS/m"},
            };
        }

        std::vector<Export::Col> sppCSV() {
            std::vector<Export::Col> c;
            for (const Export::Col &col: sppFields())
                if (col.field != F::Stat) c.push_back(col);
            return c;
        }
    }

    std::string Task::navPath() const {
        return navPathBuf.empty() ? Solvers::Task::navPath() : navPathBuf;
    }

    void Task::applyConfig(const SolverOptions &cfg) {
        opts = cfg;
        setupTruth(cfg);
        applyObsPath(cfg.obsPath);
        navPathBuf = cfg.navPath;
        rtEndpoints.resize(1);
        rtEndpoints[0].ip = cfg.ip;
        rtEndpoints[0].port = std::to_string(cfg.port);
    }

    bool Task::initSolver(::SPP &solver) {
        std::vector<std::string> navs;
        if (!navPathBuf.empty()) navs.push_back(navPathBuf);
        else navs = ObsSource::scanNavFiles(filePath);
        for (const auto &nav: navs) {
            try {
                solver.loadBRDC(nav);
                hasNav = true;
                LOG_INFO << "SPP 读取广播星历: " << nav;
            } catch (const std::exception &e) {
                LOG_ERROR << "SPP 读取广播星历失败 " << nav << ": " << e.what();
            }
        }
        const auto products = ProductLoad::resolve(opts, modeKey(Mode::SPP));
        ProductLoad::loadSPPGrids(solver, products.vmf, products.gpt3);
        ProductLoad::warnTropoFallback("SPP", opts.spp.tropoMap, solver.tropoGridReady());
        return true;
    }

    void Task::beforeSolveEpoch(::SPP &solver, Observer::Item &item) {
        solver.ephTable.mergeEmbedded(item.eph, item.obs.epoch);
        if (item.eph) hasNav = true;
        // 只有 SPP 吃广播星历；PPP/LEO 靠精密产品，缺 BRDC 不算致命
        if (mode == Mode::SPP && !Observer::carriesEph(obsFormat) && !hasNav) noEphSolve = true;
    }

    bool Task::afterSolveEpoch(::SPP &, Epoch &data, Observer::Item &, int) {
        return data.solved;
    }

    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<::SPP>();
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::SPPSession>(
            std::static_pointer_cast<Task>(shared_from_this()), rtEndpoints[0]);
    }

    Task::Task() { mode = Mode::SPP; }

    Task::~Task() = default;

    void Task::solveBody() {
        fwdSolver = makeSolver();
        if (!fwdSolver) {
            setError("解算器未就绪");
            finishSolving();
            return;
        }
        const Clock::Stopwatch solveCost;
        try {
            if (!waitNavReady()) {
                LOG_WARN << "[解算] 任务被取消(stop 已置位)：解算线程未启动，0 历元";
                finishSolving();
                return;
            }

            std::string sys;
            for (const char c: opts.systems) sys += c;
            LOG_INFO << "[解算] 开始: 截止高度角 " << opts.cutoffDeg << "°, 系统="
                    << (sys.empty() ? std::string("全部") : sys)
                    << (opts.sampleInterval > 0.0
                            ? ", 采样间隔 " + std::to_string(opts.sampleInterval) + "s"
                            : ", 采样间隔按文件")
                    << ", 滤波方向 " << filterLabel(opts.filter);
            const SolverOptions o = opts;
            fwdSolver->applyOptions(o);
            if (!initSolver(*fwdSolver)) {
                LOG_ERROR << "[解算] 解算器初始化失败（产品加载返回 false）";
                finishSolving();
                return;
            }
            warnFrameMismatch();

            runForward();
            if (filter.hasBackward()) runBackward();
        } catch (const std::exception &e) {
            LOG_ERROR << "[解算] 解算线程异常: " << e.what();
            setError(e.what());
        } catch (...) {
            LOG_ERROR << "[解算] 解算线程未知异常";
            setError("解算线程未知异常");
        }
        std::size_t total = 0;
        {
            std::lock_guard lk(mutex);
            total = epochs.size();
        }
        LOG_INFO << "[解算] 结束: " << total << " 历元, 耗时 " << solveCost.ms() << " ms";
        finishSolving();
    }

    void Task::runForward() {
        filter.mode = opts.filter;
        solvePhase = filter.solvesForward() ? SolvePhase::Forward : SolvePhase::Scan;

        // 惰性历元序列挂在会话的回调上：倒序趟要靠它按索引重放同一批历元。
        const auto source = std::make_shared<ObsFeed>(
            [this](Observer::Item &out) { return nextObs(out); },
            opts.sampleInterval, maxEpochs, filter.hasBackward());

        fwdSession = {};
        fwdSession.solver = fwdSolver.get();
        fwdSession.feed = {
            [source](const std::size_t i) { return source->ensure(i); },
            [source] { return source->count(); },
            [source](const std::size_t i) -> const ObsData & { return source->at(i).obs; },
            [source](const std::size_t i) -> const CommonTime & { return source->at(i).obs.epoch; }
        };
        fwdSession.prepare = [this, source](const std::size_t i, ::SPP &s) {
            solveEpoch = static_cast<int>(i) + 1;
            beforeSolveEpoch(s, source->at(i));
        };
        fwdSession.onEpoch = [this, source](const std::size_t index, ::SPP &s, Filter::Out &out) {
            Observer::Item &item = source->at(index);
            Epoch data;
            try {
                data.getFromObs(item.obs);
                if (out.act == Filter::Out::Act::UseSolver) data.getFromSPP(s);
                else data.solved = false;
                if (out.act != Filter::Out::Act::Skip &&
                    !afterSolveEpoch(s, data, item, static_cast<int>(index)))
                    data.solved = false;
            } catch (const std::exception &e) {
                LOG_ERROR << "[解算] 历元异常 (sod=" << item.obs.epoch.m_sod << "): " << e.what();
                data.solved = false;
            } catch (...) {
                LOG_ERROR << "[解算] 历元未知异常 (sod=" << item.obs.epoch.m_sod << ")";
                data.solved = false;
            }
            out.solved = data.solved;
            commitEpoch(std::move(data));
            return !stop.load();
        };

        filter.runForward(fwdSession);
    }

    void Task::runBackward() {
        if (stop.load()) return;
        std::size_t epochCount = 0;
        {
            std::lock_guard lk(mutex);
            epochCount = epochs.size();
        }
        const std::size_t m = std::min(fwdSession.feed.count(), epochCount);
        if (m == 0) return;

        LOG_INFO << "[滤波] 扫描完成 " << m << " 历元，从末尾倒序滤波 (" << filterLabel(filter.mode) << ")";
        const Clock::Stopwatch backwardCost;
        solvePhase = SolvePhase::Backward;
        solveEpoch = 0;
        // 纯后向档正序一趟只扫不解，参考点没定，倒序趟要自己重建
        if (!filter.solvesForward()) initializedRefECEF = false;

        const std::size_t refreshStep = std::max<std::size_t>(1, m / 40);
        const auto publish = [this] {
            std::lock_guard plk(view.plotMutex);
            // 卫星残差序列只在正序趟画过，倒序刷新整表时原样保留
            std::map<SatID, std::vector<double> > resT = std::move(view.plotData.satResTimes);
            std::map<SatID, std::vector<double> > resV = std::move(view.plotData.satResVals);
            std::map<SatID, std::vector<double> > resPT = std::move(view.plotData.satResPhaseTimes);
            std::map<SatID, std::vector<double> > resPV = std::move(view.plotData.satResPhaseVals);
            view.plotData.replaceAll(epochs, refECEF, mode);
            view.plotData.satResTimes = std::move(resT);
            view.plotData.satResVals = std::move(resV);
            view.plotData.satResPhaseTimes = std::move(resPT);
            view.plotData.satResPhaseVals = std::move(resPV);
        };

        const auto onEpoch = [this, m, refreshStep, &publish](const std::size_t i, ::SPP &bwd,
                                                              Filter::Out &out) {
            solveEpoch = static_cast<int>(i) + 1;
            std::lock_guard lk(mutex);
            if (view.selectedEpoch < 0 || view.selectedEpoch == view.lastSolveCursor)
                view.selectedEpoch = static_cast<int>(i);
            view.lastSolveCursor = static_cast<int>(i);
            if (i < epochs.size()) {
                Epoch &ep = epochs[i];
                switch (out.act) {
                    case Filter::Out::Act::Fuse: {
                        Result &res = ep.sppResult;
                        res.xyz = out.xyz;
                        res.xyzFloat = out.xyz;
                        if (res.xyzFixed.squaredNorm() > 1.0) res.xyzFixed = out.xyz;
                        res.sigmaP = out.sigma;
                        res.sigmaPFloat = out.sigma;
                        if (res.sigmaPFixed > 0.0) res.sigmaPFixed = out.sigma;
                        res.blh = Coord::XYZToBLH(out.xyz, Frame::WGS84);
                        ep.solved = true;
                        break;
                    }
                    case Filter::Out::Act::UseSolver:
                        ep.getFromSPP(bwd);
                        ep.sppResult.vel.setZero(); // 倒序趟的速度符号与时间轴相反，没有物理意义
                        ep.solved = true;
                        break;
                    case Filter::Out::Act::Fail:
                        ep.solved = false;
                        ep.sppResult.reset();
                        break;
                    case Filter::Out::Act::Keep:
                    case Filter::Out::Act::Skip:
                        break;
                }
                if (ep.solved) {
                    // 逐历元参考真值随解随填，不必等整趟结束后再补。
                    applyTruthAtEpoch(ep, static_cast<int>(i));
                    refreshTruthAtEpoch(ep, fwdSession.feed.epoch(i));
                }
            }
            if (i % refreshStep == 0 || i + 1 == m) publish();
            return !stop.load();
        };
        const auto makeBackward = [this]() -> std::unique_ptr<::SPP> {
            std::unique_ptr<::SPP> s = makeSolver();
            if (!s) return nullptr;
            s->applyOptions(opts);
            return s;
        };
        Filter::Session back = fwdSession;
        back.make = makeBackward;
        back.prepare = {}; // 内嵌星历已在正序趟注入，倒序解算器靠 adoptProducts 接管
        back.onEpoch = onEpoch;
        if (!filter.runBackward(back)) return;
        LOG_INFO << "[滤波] 后向完成，耗时 " << backwardCost.ms() << " ms";

        {
            std::lock_guard lk(mutex);
            publish();
            std::lock_guard plk(view.plotMutex);
            if (modeUsesRTN(mode)) view.rebuildTruthTrails(epochs);
            view.skyTracks.clear();
            view.skyTracksBuilt = 0;
        }
        for (std::size_t i = 0; i < m && i < epochs.size(); ++i) {
            epochs[i].sppResult.postRes.release();
            epochs[i].sppResult.postResPhase.release();
        }
    }

    void Task::onReset() {
        filter.clear();
        fwdSession = {};
        fwdSolver.reset();
    }


    std::vector<Export::Col> Task::exportFields(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return {}; // 根数只有低轨轨道有
        const std::vector<Export::Col> c = sppFields();
        return exportRefReady() ? c : Export::withoutRefColumns(c);
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        if (fmt != Export::Format::CSV) return {};
        const std::vector<Export::Col> c = sppCSV();
        return exportRefReady() ? c : Export::withoutRefColumns(c);
    }

    void Task::writeCsvParams(CSV::Meta &m) const {
        m.num("码 σ (m)", opts.spp.codeSigma);
        m.num("高度角加权下限 (sinE)", opts.spp.minSinElev);
        m.text("映射函数", Troposphere::keyOf(opts.spp.tropoMap));
        m.num("ISB 过程 σ", opts.spp.isbProcSigma);
        m.num("ISB 初始 σ", opts.spp.isbInitSigma);
        m.flag("剔除电文标记不健康的卫星", opts.spp.useHealth);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
