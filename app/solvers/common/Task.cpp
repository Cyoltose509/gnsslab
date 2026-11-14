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
#include "Velocity.h"
#include "io/Observer.h"
#include "rinex/RINEXNavStore.h"
#include "product/sp3/Reader.h"
#include "io/ProductLoad.h"
#include "solvers/common/Realtime.h"
#include "util/CSV.h"

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
        return ProductLoad::resolve(opts, modeKey(mode)).paths.brdc;
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
            LOG_WARN << src.formatName() << " 历元数触及上限，后续历元未读取: " << filePath;
        if (ok == 0)
            setError(std::string(src.formatName()) + " 无历元: " + filePath +
                     "（文件是否被取消/解压失败/选错文件？）");
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
        fillVelocityByDifference();
        {
            std::lock_guard lk(queueMutex);
            solvingDone = true;
        }
        wakeQueues();
    }

    void Task::fillVelocityByDifference() {
        std::lock_guard lk(mutex);
        if (epochs.empty()) return;

        std::vector<double> t;
        std::vector<Eigen::Vector3d> pos;
        t.reserve(epochs.size());
        pos.reserve(epochs.size());
        for (const Epoch &ep: epochs) {
            t.push_back(static_cast<double>(ep.week) * 604800.0 + ep.sow);
            pos.push_back(ep.solved ? ep.sppResult.xyz : Eigen::Vector3d::Zero());
        }
        std::vector<Eigen::Vector3d> diff;
        Velocity::byDifference(t, pos, diff);

        int velEpochs = 0;
        for (std::size_t i = 0; i < epochs.size(); ++i) {
            Epoch &ep = epochs[i];
            // 多普勒已经解出来的不动：那是一次真的测量，差分只是没观测时的兜底。
            if (ep.solved && ep.sppResult.vel.squaredNorm() == 0.0 &&
                diff[i].squaredNorm() > 0.0 && diff[i].allFinite())
                ep.sppResult.vel = diff[i];
            if (ep.sppResult.vel.squaredNorm() > 1e-4) ++velEpochs;
        }
        std::lock_guard plk(view.plotMutex); // plotData 归界面线程读，写入与 commitEpoch 同口径
        view.plotData.velEpochs = velEpochs;
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
            LOG_INFO << modeLabel(mode) << " 读取文件: " << filePath;
            if (readObsToQueue() >= 0) view.endQCInput();
        } catch (const std::exception &e) {
            failRead(e.what());
        } catch (...) {
            failRead("读取发生未知异常");
        }
        loading = false;
    }

    void Task::solveBody() {
        setError("任务未实现文件解算");
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
        const Eigen::Vector3d ref = exportRef(ep);
        const Eigen::Vector3d xyz = res.xyz;
        const Eigen::Vector3d xyzFloat = res.xyzFloat.squaredNorm() > 1.0e12 ? res.xyzFloat : xyz;
        row.set(F::X, xyz.x());
        row.set(F::Y, xyz.y());
        row.set(F::Z, xyz.z());
        if (ref.squaredNorm() > 1.0) {
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
        if (ref.squaredNorm() > 1.0) {
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
            if (ref.squaredNorm() > 1.0) {
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
        // 逐轴标准差（滤波协方差的 ECEF 三轴转到站心）：任何模式的 CSV 列表都不列它，
        // 只作为内部通道喂 .pos 的 sdn/sde/sdu；缺轴用 sigmaP 均分兜底。
        {
            const double fallback = res.sigmaP > 0.0 ? res.sigmaP / std::sqrt(3.0) : 0.0;
            double se = 0.0, sn = 0.0, su = 0.0;
            if (res.sigmaXYZ.allFinite() && res.sigmaXYZ.squaredNorm() > 0.0) {
                const DCM R = DCM::fromECEFToENU(blh[0], blh[1]);
                const Eigen::Vector3d varEnu = R * res.sigmaXYZ.cwiseProduct(res.sigmaXYZ);
                se = std::sqrt(std::max(0.0, varEnu[0]));
                sn = std::sqrt(std::max(0.0, varEnu[1]));
                su = std::sqrt(std::max(0.0, varEnu[2]));
            }
            row.set(F::SigmaE, se > 0.0 ? se : fallback);
            row.set(F::SigmaN, sn > 0.0 ? sn : fallback);
            row.set(F::SigmaU, su > 0.0 ? su : fallback);
        }
        if (ep.refECEF.squaredNorm() > 1.0) {
            row.set(F::R, ep.rtnR);
            row.set(F::T, ep.rtnT);
            row.set(F::Nrtn, ep.rtnN);
        }
    }

    Eigen::Vector3d Task::exportRef(const Epoch &ep) const {
        // 只有挂真值时 applyTruthAtEpoch 才会写 Epoch::refECEF；成员 refECEF 在无真值时
        // 是「首历元自己的位置」或用户拖的显示基准，拿它算 ENU 会得到伪误差。
        return ep.refECEF;
    }

    bool Task::exportRefReady() const { return truthReady(); }

    std::vector<Export::Col> Task::exportFields(const Export::Format) const {
        return {};
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        // 只有 CSV 按列写出，其余格式（POS/SP3/ORB/KML…）的列集对输出没有意义。
        return fmt == Export::Format::CSV ? exportFields(fmt) : std::vector<Export::Col>{};
    }

    void Task::writeCSVMeta(CSV::Writer &w) const {
        CSV::Meta m{w, "参数配置", "开", "关"};

        w.row("类别", "项目", "值");
        m.item("解算", "模式", modeLabel(mode));
        m.item("解算", "参考框架", hasOrbitFrame ? Frame::refName(orbitFrame) : "未声明(导出按 IGS14)");
        m.item("解算", "结果历元数", std::to_string(exportCount()));
        if (modeHasFilter(mode)) m.item("解算", "滤波方向", filterLabel(opts.filter));

        const auto p = ProductLoad::resolve(opts, modeKey(mode));
        m.item("参与文件", "观测文件", filePath);
        m.item("参与文件", "基准站观测", opts.basePath);
        m.item("参与文件", "流动站观测", opts.roverPath);
        m.item("参与文件", "广播星历 BRDC", p.paths.brdc);
        m.item("参与文件", "精密轨道 SP3", p.paths.sp3);
        m.item("参与文件", "精密钟差 CLK", p.paths.clk);
        m.item("参与文件", "码偏差 OSB", p.paths.osb);
        m.item("参与文件", "卫星姿态 ATT", p.paths.att);
        m.item("参与文件", "地球自转 ERP", p.paths.erp);
        m.item("参与文件", "天线文件 ATX", p.paths.atx);
        m.item("参与文件", "对流层 TRO", p.tro);
        m.item("参与文件", "电离层 IONEX", p.ionex);
        m.item("参与文件", "映射函数格网 VMF3", p.vmf);
        m.item("参与文件", "对流层格网 GPT3", p.gpt3);
        m.item("参与文件", "参考真值 SP3", truth.sp3Path);
        m.item("参与文件", "辅助文件", opts.aux);
        m.item("参与文件", "IMU 文件", opts.ins.imuPath);

        m.num("采样间隔 (s，0=按文件)", opts.sampleInterval);
        m.text("参与星座", sysSetToString(opts.systems).c_str());
        m.num("截止高度角 (°)", opts.cutoffDeg);
        m.integer("最大历元数 (0=不限)", maxEpochs);
        m.text("参考真值形式", truth.form == RefForm::Fixed ? "恒值 XYZ"
                              : truth.form == RefForm::File ? "逐历元 SP3" : "无");
        if (truth.fixed.squaredNorm() > 1.0)
            m.xyz("参考真值 XYZ", truth.fixed.x(), truth.fixed.y(), truth.fixed.z());
        if (truth.form == RefForm::File && truth.fileReady)
            m.integer("参考真值 SP3 星号", truth.sat.id);

        writeCsvParams(m);
        w.blank();
    }

    std::size_t Task::resultEpochCount() const {
        return epochs.size();
    }
}
