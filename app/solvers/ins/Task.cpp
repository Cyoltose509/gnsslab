#include "solvers/ins/Task.h"
#include "FileUtils.h"

#include <algorithm>
#include <exception>
#include <iterator>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "Exception.h"
#include "solver/ins/Aligner.h"
#include "solver/ins/INS.h"
#include "Log.h"
#include "Observer.h"
#include "i18n/I18n.h"

namespace Solvers::INS {
    namespace {
        /// 无型号日志要够开头的三个反推窗口才定得出当量
        constexpr std::size_t kScaleHeadSamples = 12000;
        /// 攒给粗对准与零偏标定的首批头段时长
        constexpr double kAlignHeadSec = 120.0;
        /// 段后连续运动够这么长，才算静置结束
        constexpr double kRunoffSec = 30.0;
        /// 判出静置段还没到头就再读这么多秒重判
        constexpr double kAlignExtendSec = 60.0;
        /// 回放时的读取上限，只防超长日志把整份数据塞进内存
        constexpr double kAlignMaxSec = 3600.0;
        /// 每多少个历元把解算结果发布一批给界面
        constexpr std::size_t kFlushEpochs = 200;

        /// IMU 样本的 t 是「周×604800 + 周内秒」的绝对秒
        double sowOf(const NavState &s, const int week) {
            return week > 0 ? s.t - static_cast<double>(week) * static_cast<double>(FULL_WEEK) : s.t;
        }

        Eigen::Vector3d initialLLH(const INSConfig &c) {
            if (!(c.latDeg >= -90.0 && c.latDeg <= 90.0 && c.lonDeg >= -180.0 && c.lonDeg <= 180.0 &&
                  c.hM > -1000.0 && c.hM < 100000.0))
                throw GeometryException(I18n::tr("ins.err.blh_invalid"));
            return {c.latDeg * DEG_TO_RAD, c.lonDeg * DEG_TO_RAD, c.hM};
        }

        using F = Export::Field;

        /// 惯导 CSV 的默认表头：导航解 + 零速修正后的对照解。
        std::vector<Export::Col> insCSV() {
            return {
                {F::Tow, "GPSTow"},
                {F::Lat, "Lat_deg", "BLH"},
                {F::Lon, "Lon_deg", "BLH"},
                {F::H, "Height_m", "BLH"},
                {F::Vn, "VN_mps", "Vel-NED"},
                {F::Ve, "VE_mps", "Vel-NED"},
                {F::Vd, "VD_mps", "Vel-NED"},
                {F::Roll, "Roll_deg", "RPH"},
                {F::Pitch, "Pitch_deg", "RPH"},
                {F::Heading, "Heading_deg", "RPH"},
                {F::E, "E_m", "EN"}, {F::N, "N_m", "EN"},
                {F::ZuptLat, "ZuptLat_deg", "Zupt-BLH"},
                {F::ZuptLon, "ZuptLon_deg", "Zupt-BLH"},
                {F::ZuptH, "ZuptHeight_m", "Zupt-BLH"},
                {F::ZuptVn, "ZuptVN_mps", "Zupt-NED"},
                {F::ZuptVe, "ZuptVE_mps", "Zupt-NED"},
                {F::ZuptVd, "ZuptVD_mps", "Zupt-NED"},
                {F::ZuptRoll, "ZuptRoll_deg", "Zupt-RPH"},
                {F::ZuptPitch, "ZuptPitch_deg", "Zupt-RPH"},
                {F::ZuptHeading, "ZuptHeading_deg", "Zupt-RPH"},
                {F::ZuptE, "ZuptE_m", "Zupt-EN"}, {F::ZuptN, "ZuptN_m", "Zupt-EN"},
                {F::ZuptFlag, "ZuptApplied"},
            };
        }

        /// 候选列：默认列原序，中间插比力与角速度（默认不导，但也是真值表里有的量）。
        std::vector<Export::Col> insFields() {
            std::vector<Export::Col> c = insCSV();
            const std::size_t at = c.size() - 12; // 第一列 Zupt* 之前
            const Export::Col raw[] = {
                {F::Axb, "Axb/(m/s2)", "Accel"},
                {F::Ayb, "Ayb/(m/s2)", "Accel"},
                {F::Azb, "Azb/(m/s2)", "Accel"},
                {F::Wxb, "Wxb/(rad/s)", "Gyro"},
                {F::Wyb, "Wyb/(rad/s)", "Gyro"},
                {F::Wzb, "Wzb/(rad/s)", "Gyro"},
            };
            c.insert(c.begin() + static_cast<std::ptrdiff_t>(at), std::begin(raw), std::end(raw));
            return c;
        }
    }

    std::shared_ptr<const Result> Task::snapshot() const {
        std::lock_guard lk(resMutex);
        return result;
    }

    void Task::onReset() {
        {
            std::lock_guard lk(resMutex);
            result.reset();
        }
        {
            std::lock_guard lk(queueMutex);
            imuQueue.clear();
        }
        headSamples = 0;
        aligning = false;
        curves = CurveCache{};
        navWeek = 0;
        freeIns = ::INS{};
        zuptIns = ::INS{};
        scale = ImuScale{};
        alignHead.clear();
        solveFrom = 0;
        // 初始位置与重力只跟配置有关，两个线程起来之前算一次，之后都只读。
        try {
            llh0 = initialLLH(opts.ins);
            gravity = getGravity(llh0[0], llh0[2]);
            initialError.clear();
        } catch (const std::exception &ex) {
            llh0.setZero();
            gravity = 0.0;
            initialError = I18n::tr("ins.err.blh_prefix", ex.what());
        }
    }

    bool Task::feedImu(const ImuSample &s) {
        {
            std::unique_lock lk(queueMutex);
            queueSpaceCv.wait(lk, [this] {
                return imuQueue.size() < queueDepth || stop.load() || solvingDone.load();
            });
            if (stop.load() || solvingDone.load()) return false;
            imuQueue.push_back(s);
        }
        queueCv.notify_one();
        return true;
    }

    bool Task::nextImu(ImuSample &out) {
        {
            std::unique_lock lk(queueMutex);
            queueCv.wait(lk, [this] {
                return !imuQueue.empty() || readDone.load() || stop.load();
            });
            if (imuQueue.empty()) return false;
            out = imuQueue.front();
            imuQueue.pop_front();
        }
        queueSpaceCv.notify_one();
        return !stop.load();
    }

    void Task::readBody() {
        if (!initialError.empty()) {
            failRead(initialError);
            return;
        }
        const INSConfig c = opts.ins;
        if (!c.manual.known()) {
            failRead(I18n::tr("ins.err.scale_missing"));
            return;
        }
        const auto reader = Observer::makeNovAtelReader(c.imuPath);
        if (!reader || !reader->open(c.imuPath)) {
            failRead(I18n::tr("ins.err.open_imu", c.imuPath));
            return;
        }

        // 采样率不在界面上填：拿开头一小段的时间戳间隔算，手填当量时同样按数据走。
        ImuLog head;
        ImuSample sample;
        try {
            while (head.size() < kScaleHeadSamples && reader->getNextImu(sample)) head.push_back(sample);
        } catch (const std::exception &ex) {
            failRead(I18n::tr("ins.err.decode", ex.what()));
            return;
        }
        if (head.empty()) {
            failRead(I18n::tr("ins.err.no_records"));
            return;
        }
        const ImuScale resolved = c.manual;
        {
            std::lock_guard lk(resMutex);
            if (!result) result = std::make_shared<Result>();
            result->scale = resolved;
            result->scaleNote = scaleText.empty() ? I18n::tr("ins.scale.manual_note") : scaleText;
        }
        LOG_INFO << "[INS] " << I18n::tr("ins.log.scale", resolved.accelMpsPerLsb, resolved.gyroRadPerLsb,
                                         resolved.rateHz);
        markNavReady();

        for (const auto &s: head)
            if (!feedImu(s)) {
                finishRead();
                return;
            }
        try {
            while (reader->getNextImu(sample))
                if (!feedImu(sample)) break;
        } catch (const std::exception &ex) {
            failRead(I18n::tr("ins.err.decode", ex.what()));
            return;
        }
        finishRead();
    }

    void Task::solveBody() {
        const INSConfig c = opts.ins;
        try {
            if (!initialError.empty()) {
                setError(initialError);
                finishSolving();
                return;
            }
            if (!waitNavReady()) {
                LOG_WARN << "[INS] " << I18n::tr("ins.log.cancelled");
                finishSolving();
                return;
            }
            {
                std::shared_ptr<Result> res;
                {
                    std::lock_guard lk(resMutex);
                    res = result;
                }
                if (!res || !res->scale.known()) {
                    if (errorMessage().empty()) setError(I18n::tr("ins.err.scale_not_ready"));
                    finishSolving();
                    return;
                }
                scale = res->scale;
            }
            if (!prepareAlignHead(c) || !prepareInitialState(c)) {
                finishSolving();
                return;
            }
            runSolveLoop();
        } catch (const std::exception &e) {
            LOG_ERROR << "[INS] " << I18n::tr("solver.log.thread_error", e.what());
            setError(e.what());
        } catch (...) {
            LOG_ERROR << "[INS] " << I18n::tr("solver.err.thread_unknown");
            setError(I18n::trc("solver.err.thread_unknown"));
        }
        finishSolving();
    }

    bool Task::prepareAlignHead(const INSConfig &c) {
        // 攒开头一段做粗对准与零偏标定。
        aligning = true;
        const bool replay = std::filesystem::exists(c.imuPath);
        const double capSec = c.specifyAttitude ? 2.0 : replay ? kAlignMaxSec : kAlignHeadSec;
        const auto capSamples = static_cast<std::size_t>(capSec * scale.rateHz);
        const auto headSamplesNeed = static_cast<std::size_t>(kAlignHeadSec * scale.rateHz);
        std::size_t alignNeed = std::min(capSamples, headSamplesNeed);
        while (true) {
            while (alignHead.size() < alignNeed) {
                ImuSample s;
                if (!nextImu(s)) break;
                alignHead.push_back(s);
                headSamples.store(static_cast<int>(alignHead.size()));
            }
            if (alignHead.size() < alignNeed || alignHead.size() >= capSamples || !replay) break;
            const StaticSpan span = Aligner::detectStatic(alignHead, scale);
            if (span.tailMove >= static_cast<std::size_t>(kRunoffSec * scale.rateHz)) break;
            alignNeed = std::min(capSamples, alignHead.size() + static_cast<std::size_t>(kAlignExtendSec * scale.rateHz));
        }
        aligning = false;

        if (alignHead.empty()) {
            setError(I18n::tr("ins.err.no_records"));
            return false;
        }
        navWeek = alignHead.front().gnssWeek;
        if (!c.specifyAttitude && alignHead.size() < static_cast<std::size_t>(20.0 * scale.rateHz)) {
            setError(I18n::tr("ins.err.short_static", alignHead.size()));
            return false;
        }
        return true;
    }

    bool Task::prepareInitialState(const INSConfig &c) {
        zuptIns.scale = scale; // 输入准备与判静参数都按采样率换算
        zuptIns.gravity = gravity;
        // ZUPT 是 INS 的开关：zupt 实例打开，free 实例保持自由解作对照。
        zuptIns.zuptEnabled = true;
        zuptIns.zuptAccelTol = c.zuptAccelTol;
        zuptIns.zuptGyroTol = c.zuptGyroTol;
        zuptIns.zuptHoldSec = c.zuptHoldSec;

        NavState x0;
        x0.llh = llh0;
        x0.vned = Eigen::Vector3d(c.vN, c.vE, c.vD);
        Euler rphDeg; // 配置层用度，状态层用四元数，转换只在这一处发生
        if (!c.specifyAttitude) {
            zuptIns.align(alignHead, llh0[0]);
            rphDeg.roll = zuptIns.alignAtt.roll * RAD_TO_DEG;
            rphDeg.pitch = zuptIns.alignAtt.pitch * RAD_TO_DEG;
            rphDeg.yaw = zuptIns.alignAtt.yaw * RAD_TO_DEG;
            {
                std::lock_guard lk(resMutex);
                Result &r = *result;
                r.align.used = true;
                r.align.spanSec = static_cast<double>(zuptIns.alignCount) / scale.rateHz;
                r.align.rollDeg = rphDeg.roll;
                r.align.pitchDeg = rphDeg.pitch;
                r.align.headingDeg = rphDeg.yaw;
                r.align.earthRateErrDeg = zuptIns.alignEarthRateErrDeg;
                r.align.gyroSigmaRad = zuptIns.alignGyroSigmaRad;
                r.align.yawSpreadDeg = zuptIns.alignYawSpreadDeg;
                r.align.headingReliable = zuptIns.alignHeadingReliable;
            }
            LOG_INFO << "[INS] " << I18n::tr("ins.log.coarse_align", zuptIns.alignCount / scale.rateHz,
                                             rphDeg.roll, rphDeg.pitch, rphDeg.yaw,
                                             zuptIns.alignYawSpreadDeg);
        } else {
            rphDeg.roll = c.rollDeg;
            rphDeg.pitch = c.pitchDeg;
            rphDeg.yaw = c.headingDeg;
        }

        // 导航从对准段末端起算：段内的上电沉降、搬运不是载体运动，喂进编排只会积出假速度。
        solveFrom = c.specifyAttitude ? 0 : zuptIns.alignBegin + zuptIns.alignCount;
        if (solveFrom >= alignHead.size()) {
            setError(I18n::tr("ins.err.short_static", alignHead.size()));
            return false;
        }
        x0.t = alignHead[solveFrom].tow;
        x0.qbn = ATTQuat::fromEuler({rphDeg.roll * DEG_TO_RAD, rphDeg.pitch * DEG_TO_RAD,
                                     rphDeg.yaw * DEG_TO_RAD});

        freeIns.init(x0);
        zuptIns.init(x0);
        // 静置段标定：加计零偏进转换、参考矢量进判据，全部落在 zupt 实例内。
        zuptIns.calibrate(alignHead);
        if (zuptIns.biasApplied) {
            {
                std::lock_guard lk(resMutex);
                result->accelBias = zuptIns.accelBias;
                result->accelBiasApplied = true;
            }
            std::ostringstream accelBias, gyroBias;
            accelBias << zuptIns.accelBias.transpose();
            gyroBias << zuptIns.gyroBiasDiag.transpose();
            LOG_INFO << "[INS] " << I18n::tr("ins.log.static_bias", accelBias.str(), gyroBias.str());
        } else if (zuptIns.alignCount > 1) {
            LOG_INFO << "[INS] " << I18n::tr("ins.log.bias_too_large", zuptIns.accelBias.norm());
        }
        return true;
    }

    void Task::runSolveLoop() {
        std::vector<NavState> batchFree, batchZupt;
        std::vector<unsigned char> batchFlag;
        std::vector<InsEpoch> batchImu;
        std::size_t batchZuptCount = 0;
        std::size_t processed = 0;

        const auto flush = [&] {
            if (batchFree.empty()) return;
            {
                std::lock_guard lk(resMutex);
                result->nav.insert(result->nav.end(), batchFree.begin(), batchFree.end());
                result->navZupt.insert(result->navZupt.end(), batchZupt.begin(), batchZupt.end());
                result->zuptApplied.insert(result->zuptApplied.end(), batchFlag.begin(), batchFlag.end());
                result->imu.insert(result->imu.end(), batchImu.begin(), batchImu.end());
                result->zuptCount += batchZuptCount;
            }
            batchFree.clear();
            batchZupt.clear();
            batchFlag.clear();
            batchImu.clear();
            batchZuptCount = 0;
        };

        // 起算样本就是初始时刻，其余回填队列，逐历元推进只留一条循环。
        {
            std::lock_guard lk(queueMutex);
            const auto from = static_cast<std::ptrdiff_t>(solveFrom + 1);
            imuQueue.insert(imuQueue.begin(), alignHead.begin() + from, alignHead.end());
        }
        zuptIns.prevRaw = alignHead[solveFrom]; // 转换接续点：首个被转换样本是 alignHead[solveFrom+1]
        zuptIns.hasPrevRaw = true;

        while (true) {
            ImuSample smp;
            if (!nextImu(smp)) break;
            if (stop.load()) break;
            if (maxEpochs > 0 && static_cast<int>(processed) >= maxEpochs) break;

            const InsEpoch e = zuptIns.toEpoch(smp);

            freeIns.update(e);
            zuptIns.update(e);
            batchFree.push_back(freeIns.s);
            batchZupt.push_back(zuptIns.s);
            batchFlag.push_back(zuptIns.zuptApplied ? 1 : 0);
            if (zuptIns.zuptApplied) ++batchZuptCount;
            batchImu.push_back(e);

            ++processed;
            if (batchFree.size() >= kFlushEpochs) flush();
        }
        flush();

        std::size_t total = 0, zuptCount = 0;
        {
            std::lock_guard lk(resMutex);
            total = result->nav.size();
            zuptCount = result->zuptCount;
        }
        LOG_INFO << "[INS] " << I18n::tr("ins.log.done", total, scale.rateHz);
        LOG_INFO << "[INS] " << I18n::tr("ins.log.zupt_stat", total == 0 ? 0.0 : 100.0 * zuptCount / total,
                                         zuptIns.zuptDiag.accDevMax, zuptIns.zuptDiag.accFail,
                                         zuptIns.zuptDiag.gyrDevMax, zuptIns.zuptDiag.gyrFail);
    }

    std::size_t Task::resultEpochCount() const {
        const auto snap = snapshot();
        return snap ? snap->nav.size() : 0;
    }

    std::size_t Task::exportCount() const {
        return resultEpochCount();
    }

    Export::Stamp Task::exportStamp(const std::size_t i) const {
        const auto snap = snapshot();
        if (!snap || i >= snap->nav.size()) return {};
        return {static_cast<unsigned>(navWeek), sowOf(snap->nav[i], navWeek)};
    }

    void Task::exportRow(const std::size_t i, Export::Row &row) const {
        row.reset();
        const auto snap = snapshot();
        if (!snap) return;
        std::lock_guard lk(resMutex);
        if (i >= snap->nav.size()) return;
        using F = Export::Field;
        // 主列（BLH/速度/姿态/EN）跟随结果页当前查看的解；Zupt* 附加列恒为零速修正解，便于对照。
        const bool useZupt = shownSolution == 1 && i < snap->navZupt.size();
        const NavState &s = useZupt ? snap->navZupt[i] : snap->nav[i];
        const double lat0 = snap->nav.front().llh[0];
        const double lon0 = snap->nav.front().llh[1];
        row.stamp = {static_cast<unsigned>(navWeek), sowOf(s, navWeek)};
        row.solved = true;
        row.stat = 2;
        row.set(F::Tow, row.stamp.tow);
        row.set(F::Lat, s.llh[0] * RAD_TO_DEG);
        row.set(F::Lon, s.llh[1] * RAD_TO_DEG);
        row.set(F::H, s.llh[2]);
        row.set(F::Vn, s.vned[0]);
        row.set(F::Ve, s.vned[1]);
        row.set(F::Vd, s.vned[2]);
        const auto [roll, pitch, yaw] = s.qbn.toEuler();
        row.set(F::Roll, roll * RAD_TO_DEG);
        row.set(F::Pitch, pitch * RAD_TO_DEG);
        row.set(F::Heading, yaw * RAD_TO_DEG);
        double n = 0.0, e = 0.0;
        Coord::LLHToLocalNE(s.llh[0], s.llh[1], lat0, lon0, n, e);
        row.set(F::E, e);
        row.set(F::N, n);
        if (i < snap->navZupt.size()) {
            const NavState &z = snap->navZupt[i];
            row.set(F::ZuptLat, z.llh[0] * RAD_TO_DEG);
            row.set(F::ZuptLon, z.llh[1] * RAD_TO_DEG);
            row.set(F::ZuptH, z.llh[2]);
            row.set(F::ZuptVn, z.vned[0]);
            row.set(F::ZuptVe, z.vned[1]);
            row.set(F::ZuptVd, z.vned[2]);
            const auto [roll, pitch, yaw] = z.qbn.toEuler();
            row.set(F::ZuptRoll, roll * RAD_TO_DEG);
            row.set(F::ZuptPitch, pitch * RAD_TO_DEG);
            row.set(F::ZuptHeading, yaw * RAD_TO_DEG);
            double zn = 0.0, ze = 0.0;
            Coord::LLHToLocalNE(z.llh[0], z.llh[1], lat0, lon0, zn, ze);
            row.set(F::ZuptE, ze);
            row.set(F::ZuptN, zn);
        }
        if (i < snap->zuptApplied.size()) row.set(F::ZuptFlag, snap->zuptApplied[i]);
        if (i < snap->imu.size() && snap->imu[i].h > 0.0) {
            // 一次采样的增量除以间隔就是机体系比力与角速度（已扣静置段加计零偏）。
            const InsEpoch &e2 = snap->imu[i];
            row.set(F::Axb, e2.dvel[0] / e2.h);
            row.set(F::Ayb, e2.dvel[1] / e2.h);
            row.set(F::Azb, e2.dvel[2] / e2.h);
            row.set(F::Wxb, e2.dtheta[0] / e2.h);
            row.set(F::Wyb, e2.dtheta[1] / e2.h);
            row.set(F::Wzb, e2.dtheta[2] / e2.h);
        }
    }


    std::vector<Export::Col> Task::exportFields(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return {}; // 根数只有低轨轨道有
        return insFields();
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        return fmt == Export::Format::CSV ? insCSV() : std::vector<Export::Col>{};
    }

    void Task::writeCSVParams(CSV::Meta &m) const {
        const auto &ins = opts.ins;
        m.xyz(I18n::trc("param.ins_init_pos"), ins.latDeg, ins.lonDeg, ins.hM);
        m.xyz(I18n::trc("param.ins_init_vel"), ins.vN, ins.vE, ins.vD);
        m.flag(I18n::trc("export.meta.par.specify_attitude"), ins.specifyAttitude);
        m.num(I18n::trc("param.ins_roll"), ins.rollDeg);
        m.num(I18n::trc("param.ins_pitch"), ins.pitchDeg);
        m.num(I18n::trc("param.ins_heading"), ins.headingDeg);
        m.num(I18n::trc("param.decode_accel"), ins.manual.accelMpsPerLsb);
        m.num(I18n::trc("param.decode_gyro"), ins.manual.gyroRadPerLsb);
        m.num(I18n::trc("param.decode_rate"), ins.manual.rateHz);
        m.num(I18n::trc("param.zupt_accel_tol"), ins.zuptAccelTol);
        m.num(I18n::trc("param.zupt_gyro_tol"), ins.zuptGyroTol);
        m.num(I18n::trc("param.zupt_hold"), ins.zuptHoldSec);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->opts = cfg;
        task->filePath = cfg.ins.imuPath;
        task->fileName = File::baseName(cfg.ins.imuPath);
        return task;
    }
}
