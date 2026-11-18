#include "Task.h"

#include <filesystem>
#include <memory>
#include <ostream>
#include <queue>
#include <string>

#include "Log.h"
#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "FileUtils.h"
#include "io/Observer.h"
#include "rinex/RINEXNavStore.h"
#include "product/sp3/Reader.h"
#include "io/ProductPaths.h"
#include "io/ProductReader.h"
#include "solvers/common/Realtime.h"
#include "util/CSV.h"
#include "i18n/I18n.h"

namespace Solvers {
    namespace {
        std::atomic<int> cfgSeq{0};
    }

    int Task::nextCfgIndex() {
        return cfgSeq.fetch_add(1) + 1;
    }

    int Task::failRead(const std::string &msg) {
        {
            std::lock_guard lk(queueMutex);
            if (readDone.exchange(true)) return -1;
        }
        LOG_ERROR << msg;
        setError(msg);
        wakeQueues();
        return -1;
    }

    void Task::markNavReady() {
        {
            std::lock_guard lk(queueMutex);
            navReady = true;
        }
        queueCv.notify_all();
    }

    void Task::finishRead() {
        {
            std::lock_guard lk(queueMutex);
            readDone = true;
        }
        queueCv.notify_all();
        queueSpaceCv.notify_all();
    }

    int Task::feedToQueue(Observer::Source &src) {
        int ok = 0;
        while (!stop.load() && !epochLimitReached()) {
            Observer::Item item;
            if (!src.next(item)) break;
            view.pushQCInput(item.obs, item.obs.weekSecond.sow);
            view.markQcDirty();
            {
                std::unique_lock lk(queueMutex);
                queueSpaceCv.wait(lk, [this] {
                    return obsQueue.size() < queueDepth || stop.load() || solvingDone.load();
                });
                if (stop.load() || solvingDone.load()) break;
                obsQueue.push(std::move(item));
            }
            queueCv.notify_one();
            ++ok;
        }
        return ok;
    }

    std::string Task::navPath() const {
        return AppConfig::ProductPaths::of(modeKey(mode)).resolved(opts).nav;
    }

    ProductReader::Loaded Task::productsFor(const AppConfig::ProductPaths &d, const std::string &tag,
                                            const std::set<SatSystem> &systems,
                                            const std::vector<ProductType> &wanted) {
        const auto &pa = d;
        std::string key = tag + '|' + filePath;
        for (const std::string *s: {&pa.nav, &pa.sp3, &pa.clk, &pa.osb, &pa.att, &pa.erp, &pa.atx,
                                    &pa.tro, &pa.ion, &pa.vmf, &pa.gpt3}) {
            key += '|';
            key += *s;
        }
        for (const SatSystem sys: systems) key += charFrom(sys);
        key += '|';
        for (const auto id: wanted) key += std::to_string(static_cast<int>(id)) + ',';
        if (!prodCache || key != prodCacheKey) {
            const auto loaded = ProductReader::load(d, tag, systems, wanted);
            prodCache = loaded.prod;
            prodCacheReadOk = loaded.readOk;
            prodCacheKey = key;
        }
        return {prodCache, prodCacheReadOk};
    }

    bool Task::requireOrbit(const std::string &sp3, const std::string &brdc) {
        if (!sp3.empty() || !brdc.empty()) return true;
        setError(I18n::tr("pl.err.no_orbit"));
        return false;
    }

    int Task::readObsToQueue() {
        Observer::ObsStream src;
        std::string err;
        // 观测头的 GLONASS 槽位常常只列首历元那几颗，缺的星按广播星历的频率号补，
        // 否则频率落到 k=0，GLONASS 相位与 IF 组合会错到公里级。
        std::map<std::string, int> gloSlots;
        if (const std::string navFile = navPath(); !navFile.empty())
            RINEXNavStore::readGlonassSlots(navFile, gloSlots);
        if (!src.open(filePath, &err, &gloSlots)) return failRead(err + ": " + filePath);
        obsFormat = src.format();
        markNavReady();

        int ok = 0;
        try {
            ok = feedToQueue(src);
        } catch (const std::exception &e) {
            LOG_ERROR << src.formatName() << " parse err (" << filePath << "): " << e.what();
            return failRead(std::string(src.formatName()) + " parse err: " + e.what());
        }
        if (src.hitEpochLimit())
            LOG_WARN << src.formatName() << I18n::tr("task.log.epoch_limit", filePath);
        if (ok == 0)
            setError(I18n::tr("task.err.no_epoch", src.formatName(), filePath));
        // 只置位不清零：BRDC 可能已由 initSolver 装进求解器，本文件 0 历元不代表没有星历
        if (src.carriesEph() && ok > 0) hasNav = true;
        finishRead();
        return ok;
    }

    void Task::wakeQueues() {
        queueCv.notify_all();
        queueSpaceCv.notify_all();
    }

    void Task::cancel() {
        {
            std::lock_guard lk(queueMutex);
            stop = true;
        }
        // 真值获取也一并收手：任务都要停了，取回的坐标没有页面能套用。
        refFetchCancel = true;
        wakeQueues();
    }

    void Task::finishSolving() {
        {
            std::lock_guard lk(queueMutex);
            solvingDone = true;
        }
        wakeQueues();
    }

    bool Task::epochLimitReached() {
        if (maxEpochs <= 0) return false;
        std::lock_guard lk(mutex);
        return static_cast<int>(epochs.size()) >= maxEpochs;
    }

    bool Task::shouldStop() {
        if (stop.load() || solvingDone.load()) return true;
        if (realtimeSec > 0 && Clock::over(tStart, Clock::seconds(realtimeSec))) return true;
        return epochLimitReached();
    }

    void Task::waitDone() {
        if (worker.joinable()) worker.join();
    }

    void Task::readBody() {
        try {
            LOG_INFO << modeLabel(mode) << I18n::tr("task.log.reading", filePath);
            if (readObsToQueue() >= 0) view.endQCInput();
        } catch (const std::exception &e) {
            failRead(e.what());
        } catch (...) {
            failRead(I18n::trc("task.err.read_unknown"));
        }
        loading = false;
    }

    void Task::solveBody() {
        setError(I18n::trc("task.err.not_implemented"));
        finishSolving();
    }

    void Task::resetForSolve() {
        if (worker.joinable()) worker.join();

        stop = false;
        done = false;
        loading = true;
        state = State::Running;
        tStart = Clock::now();
        clearError();

        readDone = false;
        solvingDone = false;
        solveEpoch = 0;
        solvePhase = SolvePhase::Forward;
        navReady = false;
        hasNav = false;
        noEphSolve = false;
        initializedRefECEF = false;
        {
            std::lock_guard lk(queueMutex);
            std::queue<Observer::Item> empty;
            std::swap(obsQueue, empty);
        }
        {
            std::lock_guard lk(mutex);
            epochs.clear();
        }

        {
            std::lock_guard lk(view.plotMutex);
            view.plotData.clear();
            view.plotSeries = PlotSeries{};
            view.plotSeries.revision = view.plotData.revision; // 复位后不触发「有新数据」
        }
        view.clearQCInput(opts.sampleInterval);
        view.qcReport.reset();
        view.qcReady = false;
        view.qcDirty = false;
        view.extraQcTabs.clear();
        view.skyTracks.clear();
        view.skyTracksBuilt = 0;
        view.leoTraj.clear();
        view.leoRefTraj.clear();
        view.rtnTimes.clear();
        view.rtnR.clear();
        view.rtnT.clear();
        view.rtnN.clear();
        view.rtnD3.clear();
        view.hasLeo = false;
        view.leoPos3d.setZero();
        view.leoRef3d.setZero();
        view.selectedEpoch = -1;
        view.lastSolveCursor = -1;
        view.selectedSatIdx = -1;

        onReset();
    }

    void Task::startFile() {
        resetForSolve();
        worker = std::thread([self = shared_from_this()] {
            self->loadTruth();
            std::thread thRead([self] { self->readBody(); });
            std::thread thSolve([self] { self->solveBody(); });
            thRead.join();
            thSolve.join();
            self->state = State::Done;
            self->done = true;
            self->loading = false;
        });
    }

    void Task::startRealtime() {
        resetForSolve();
        worker = std::thread([self = shared_from_this()] {
            if (const auto session = self->makeSession()) session->run();
            self->view.endQCInput();
        });
    }

    void Task::exportTo(std::ostream &out, const Export::Spec &spec) const {
        Export::write(out, *this, spec);
    }


    std::size_t Task::exportCount() const { return epochs.size(); }

    Export::Stamp Task::exportStamp(const std::size_t i) const {
        if (i >= epochs.size()) return {};
        return {epochs[i].week, epochs[i].sow};
    }

    std::string Task::exportStem() const {
        return File::stripExt(File::baseName(fileName));
    }

    void Task::exportRow(const std::size_t i, Export::Row &row) const {
        row.reset();
        if (i >= epochs.size()) return;
        using F = Export::Field;
        const Epoch &ep = epochs[i];
        row.stamp = {ep.week, ep.sow};
        row.solved = ep.solved;
        row.stat = ep.solved ? (ep.isFixed(mode) ? 1 : 2) : 0;
        row.set(F::Week, ep.week);
        row.set(F::Tow, ep.sow);
        row.set(F::Stat, row.stat);
        if (!ep.solved) return;

        const Result &res = ep.sppResult;
        const Eigen::Vector3d ref = ep.refECEF;
        const bool hasRef = ref.squaredNorm() > 1.0;
        const Eigen::Vector3d xyz = res.xyz;
        const Eigen::Vector3d xyzFloat = res.xyzFloat.squaredNorm() > 1.0e12 ? res.xyzFloat : xyz;
        row.set(F::X, xyz.x());
        row.set(F::Y, xyz.y());
        row.set(F::Z, xyz.z());
        if (hasRef) {
            row.set(F::RefX, ref.x());
            row.set(F::RefY, ref.y());
            row.set(F::RefZ, ref.z());
            const ENU enu = Coord::XYZToENU(xyz, ref);
            row.set(F::E, enu.E());
            row.set(F::N, enu.N());
            row.set(F::U, enu.U());
            row.set(F::Rms3dF, std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U()));
        }
        const Eigen::Vector3d blh = res.blh.squaredNorm() > 0.0 ? res.blh : Coord::XYZToBLH(xyz, Frame::WGS84);
        row.set(F::Lat, blh[0] * RAD_TO_DEG);
        row.set(F::Lon, blh[1] * RAD_TO_DEG);
        row.set(F::H, blh[2]);
        // 经纬高按解拆分：POS/KML 等格式仍吃主解（Lat/Lon/H），CSV 的两组随浮点/固定走。
        {
            const Eigen::Vector3d fblh = Coord::XYZToBLH(xyzFloat, Frame::WGS84);
            row.set(F::LatF, fblh[0] * RAD_TO_DEG);
            row.set(F::LonF, fblh[1] * RAD_TO_DEG);
            row.set(F::Hf, fblh[2]);
        }
        row.set(F::Vx, res.vel[0]);
        row.set(F::Vy, res.vel[1]);
        row.set(F::Vz, res.vel[2]);
        {
            const Eigen::Vector3d vEnu = DCM::fromECEFToENU(blh[0], blh[1]) * res.vel;
            row.set(F::Ve, vEnu[0]);
            row.set(F::Vn, vEnu[1]);
            row.set(F::Vd, -vEnu[2]);
        }
        row.set(F::Xf, xyzFloat.x());
        row.set(F::Yf, xyzFloat.y());
        row.set(F::Zf, xyzFloat.z());
        if (hasRef) {
            const ENU enu = Coord::XYZToENU(xyzFloat, ref);
            row.set(F::Ef, enu.E());
            row.set(F::Nf, enu.N());
            row.set(F::Uf, enu.U());
        }
        row.set(F::SigmaPf, res.sigmaPFloat >= 0.0 ? res.sigmaPFloat : res.sigmaP);
        if (ep.isFixed(mode) && res.xyzFixed.squaredNorm() > 1.0e12) {
            row.set(F::Xk, res.xyzFixed.x());
            row.set(F::Yk, res.xyzFixed.y());
            row.set(F::Zk, res.xyzFixed.z());
            if (hasRef) {
                const ENU enu = Coord::XYZToENU(res.xyzFixed, ref);
                row.set(F::Ek, enu.E());
                row.set(F::Nk, enu.N());
                row.set(F::Uk, enu.U());
                row.set(F::Rms3dK, std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U()));
            }
            row.set(F::SigmaPk, res.sigmaPFixed);
            const Eigen::Vector3d kblh = res.blhFixed.squaredNorm() > 0.0
                                             ? res.blhFixed
                                             : Coord::XYZToBLH(res.xyzFixed, Frame::WGS84);
            row.set(F::LatK, kblh[0] * RAD_TO_DEG);
            row.set(F::LonK, kblh[1] * RAD_TO_DEG);
            row.set(F::Hk, kblh[2]);
        }
        row.set(F::Pdop, res.pdop);
        row.set(F::Gdop, res.gdop);
        row.set(F::Hdop, res.hdop);
        row.set(F::Vdop, res.vdop);
        row.set(F::Tdop, res.tdop);
        row.set(F::SigmaP, res.sigmaP);
        row.set(F::SigmaV, res.sigmaV);
        row.set(F::Sats, ep.numSatsResult > 0 ? ep.numSatsResult : res.numSats);
        row.set(F::Ratio, ep.rtkRatio > 0.0 ? ep.rtkRatio : res.ratio);
        {
            const double fallback = res.sigmaP > 0.0 ? res.sigmaP / std::sqrt(3.0) : 0.0;
            double se = 0.0, sn = 0.0, su = 0.0;
            if (res.sigmaXYZ.allFinite() && res.sigmaXYZ.squaredNorm() > 0.0) {
                const DCM R = DCM::fromECEFToENU(blh[0], blh[1]);
                // 方差传播只取平方项：Σ_j R²(i,j)·σ²(j)，直接 R·σ² 会正负相消。
                const Eigen::Vector3d varEnu =
                        R.matrix().cwiseProduct(R.matrix()) * res.sigmaXYZ.cwiseProduct(res.sigmaXYZ);
                se = std::sqrt(std::max(0.0, varEnu[0]));
                sn = std::sqrt(std::max(0.0, varEnu[1]));
                su = std::sqrt(std::max(0.0, varEnu[2]));
            }
            row.set(F::SigmaE, se > 0.0 ? se : fallback);
            row.set(F::SigmaN, sn > 0.0 ? sn : fallback);
            row.set(F::SigmaU, su > 0.0 ? su : fallback);
        }
        if (hasRef) {
            row.set(F::R, ep.rtnR);
            row.set(F::T, ep.rtnT);
            row.set(F::Nrtn, ep.rtnN);
        }
    }


    std::vector<Export::Col> Task::columnsForTruth(std::vector<Export::Col> cols) const {
        return truthReady() ? cols : Export::withoutRefColumns(cols);
    }

    std::vector<Export::Col> Task::exportFields(const Export::Format) const {
        return {};
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        return fmt == Export::Format::CSV ? exportFields(fmt) : std::vector<Export::Col>{};
    }

    void Task::writeCommonParams(CSV::Meta &m) const {
        m.num(I18n::trc("param.code_sigma"), opts.spp.codeSigma);
        m.num(I18n::trc("param.elev_weight_floor"), opts.spp.minSinElev);
        m.num(I18n::trc("param.isb_proc_sigma"), opts.spp.isbProcSigma);
        m.num(I18n::trc("param.isb_init_sigma"), opts.spp.isbInitSigma);
        m.flag(I18n::trc("param.drop_unhealthy"), opts.spp.useHealth);
    }

    void Task::writeCSVMeta(CSV::Writer &w) const {
        CSV::Meta m{w, I18n::trc("export.meta.sheet"), I18n::trc("export.meta.flag.on"),
                    I18n::trc("export.meta.flag.off")};

        w.row(I18n::trc("dl.ui.col.category"), I18n::trc("ins.verify.col.item"), I18n::trc("common.value"));
        const char *solve = I18n::trc("common.solution");
        const char *files = I18n::trc("export.meta.cat.files");
        m.item(solve, I18n::trc("common.mode"), modeLabel(mode));
        m.item(solve, I18n::trc("export.meta.frame"),
               hasOrbitFrame ? Frame::refName(orbitFrame) : I18n::trc("export.meta.val.frame_default"));
        m.item(solve, I18n::trc("export.meta.result_epochs"), std::to_string(exportCount()));
        if (modeHasFilter(mode)) m.item(solve, I18n::trc("param.filter_dir"), filterKey(opts.filter));

        const auto p = AppConfig::ProductPaths::of(modeKey(mode)).resolved(opts);
        m.item(files, I18n::trc("ObservationFile"), filePath);
        m.item(files, I18n::trc("rtk.base_obs_file"), opts.basePath);
        m.item(files, I18n::trc("rtk.rover_obs_file"), opts.roverPath);
        for (const auto &it: ProductReader::describe(p)) m.item(files, typeLabel(it.type), *it.path);
        m.item(files, I18n::trc("export.meta.file.truth_sp3"), truth.sp3Path);
        m.item(files, I18n::trc("export.meta.file.aux"), opts.aux);
        m.item(files, I18n::trc("export.meta.file.imu"), opts.ins.imuPath);

        m.num(I18n::trc("param.sample_interval"), opts.sampleInterval);
        m.text(I18n::trc("param.constellations"), sysSetToString(opts.systems).c_str());
        m.num(I18n::trc("param.elev_cutoff"), opts.cutoffDeg);
        m.integer(I18n::trc("export.meta.max_epochs"), maxEpochs);
        m.text(I18n::trc("export.meta.ref_form"), truth.form == RefForm::Fixed ? I18n::trc("export.meta.val.fixed")
                                                   : truth.form == RefForm::File ? I18n::trc("export.meta.val.file")
                                                                                 : I18n::trc("common.none"));
        if (truth.fixed.squaredNorm() > 1.0)
            m.xyz(I18n::trc("export.meta.ref_xyz"), truth.fixed.x(), truth.fixed.y(), truth.fixed.z());
        if (truth.form == RefForm::File) {
            std::lock_guard lk(truthFileMtx);
            if (truth.fileReady)
                m.integer(I18n::trc("export.meta.ref_prn"), truth.sat.id);
        }

        writeCSVParams(m);
        w.blank();
    }

    std::size_t Task::resultEpochCount() const {
        return epochs.size();
    }
}
