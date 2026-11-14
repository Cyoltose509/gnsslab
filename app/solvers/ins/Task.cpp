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
#include "solver/ins/INS.h"
#include "Log.h"
#include "Observer.h"

namespace Solvers::INS {
    namespace {
        /// 无型号日志要够开头的三个反推窗口才定得出当量
        constexpr std::size_t kScaleHeadSamples = 12000;
        /// 攒给粗对准与零偏标定的头段时长
        constexpr double kAlignHeadSec = 120.0;
        /// 每多少个历元把解算结果发布一批给界面
        constexpr std::size_t kFlushEpochs = 200;

        /// IMU 样本的 t 是「周×604800 + 周内秒」的绝对秒
        double sowOf(const NavState &s, const int week) {
            return week > 0 ? s.t - static_cast<double>(week) * static_cast<double>(FULL_WEEK) : s.t;
        }

        Eigen::Vector3d initialLLH(const INSConfig &c) {
            if (!(c.latDeg >= -90.0 && c.latDeg <= 90.0 && c.lonDeg >= -180.0 && c.lonDeg <= 180.0 &&
                  c.hM > -1000.0 && c.hM < 100000.0))
                throw GeometryException("初始位置 BLH 非法：纬度应在 ±90°、经度 ±180°、高程 -1000~100000 m");
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
        const INSConfig c = opts.ins;
        Eigen::Vector3d llh;
        try {
            llh = initialLLH(c);
        } catch (const std::exception &ex) {
            failRead(std::string("初始位置 BLH 非法：") + ex.what());
            return;
        }
        const double gravity = NormalGravity::at(llh[0], llh[2]);

        const auto reader = Observer::makeNovAtelReader(c.imuPath);
        if (!reader || !reader->open(c.imuPath)) {
            failRead("无法打开 IMU 日志：" + c.imuPath);
            return;
        }

        // 先攒开头一小段定解码当量：手填或型号自描述一两帧就够，否则要够反推用的静置样本。
        ImuLog head;
        ImuSample sample;
        const std::size_t cap = c.manual.known() ? 2 : kScaleHeadSamples;
        try {
            while (head.size() < cap && reader->getNextImu(sample)) {
                head.push_back(sample);
                if (c.manual.known() || reader->detectedImuModel()) break;
            }
        } catch (const std::exception &ex) {
            failRead(std::string("IMU 日志解码失败：") + ex.what());
            return;
        }
        if (head.empty()) {
            failRead("IMU 日志中没有可解码的六轴计数记录");
            return;
        }

        ::INS ins; // 只用输入准备：解当量
        ins.gravity = gravity;
        if (!ins.resolveScale(head, reader->detectedImuModel(), c.manual)) {
            failRead(ins.scaleError);
            return;
        }
        {
            std::lock_guard lk(resMutex);
            if (!result) result = std::make_shared<Result>();
            result->modelName = ins.scaleName;
            result->scale = ins.scale;
            result->scaleSource = ins.scaleSource;
            result->scaleNote = ins.scaleNote;
        }
        LOG_INFO << "[INS] 解码当量：" << ins.scaleNote << "，采样率 " << ins.scale.rateHz << " Hz";
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
            failRead(std::string("IMU 日志解码失败：") + ex.what());
            return;
        }
        finishRead();
    }

    void Task::solveBody() {
        const INSConfig c = opts.ins;
        const auto bail = [this](const std::string &msg) {
            setError(msg);
            finishSolving();
        };

        Eigen::Vector3d llh;
        try {
            llh = initialLLH(c);
        } catch (const std::exception &ex) {
            bail(std::string("初始位置 BLH 非法：") + ex.what());
            return;
        }
        const double gravity = NormalGravity::at(llh[0], llh[2]);

        if (!waitNavReady()) {
            LOG_WARN << "[INS] 任务被取消：解算线程未启动";
            finishSolving();
            return;
        }
        std::shared_ptr<Result> r;
        {
            std::lock_guard lk(resMutex);
            r = result;
        }
        if (!r || !r->scale.known()) {
            if (errorMessage().empty()) setError("IMU 解码当量未就绪");
            finishSolving();
            return;
        }
        const ImuScale scale = r->scale;

        // 攒开头一段做粗对准与零偏标定：静止段再长也只截这么长，之后就是逐样本推进。
        aligning = true;
        ImuLog head;
        const double headSec = c.specifyAttitude ? 2.0 : kAlignHeadSec;
        const std::size_t alignNeed = static_cast<std::size_t>(headSec * scale.rateHz);
        while (head.size() < alignNeed) {
            ImuSample s;
            if (!nextImu(s)) break;
            head.push_back(s);
            headSamples.store(static_cast<int>(head.size()));
        }
        aligning = false;

        if (head.empty()) {
            bail("IMU 日志中没有可解码的六轴计数记录");
            return;
        }
        navWeek = head.front().gnssWeek;
        if (!c.specifyAttitude && head.size() < static_cast<std::size_t>(20.0 * scale.rateHz)) {
            bail("IMU 有效样本不足 20 s（" + std::to_string(head.size()) +
                 " 个），无法自动解析粗对准；请勾选「指定初始姿态角」并手填");
            return;
        }

        ::INS free, zupt;
        zupt.scale = scale;         // 输入准备与判静参数都按采样率换算
        zupt.gravity = gravity;
        // ZUPT 是 INS 的开关：zupt 实例打开，free 实例保持自由解作对照。
        zupt.zuptEnabled = true;
        zupt.zuptAccelTol = c.zuptAccelTol;
        zupt.zuptGyroTol = c.zuptGyroTol;
        zupt.zuptHoldSec = c.zuptHoldSec;

        InsInit init;
        init.t0 = head.front().tow;
        init.llh = llh;
        init.vned = Eigen::Vector3d(c.vN, c.vE, c.vD);
        std::size_t staticEnd = 0;
        if (!c.specifyAttitude) {
            if (!zupt.align(head, llh[0])) {
                bail(zupt.alignError);
                return;
            }
            init.rollDeg = zupt.alignAtt.roll * RAD_TO_DEG;
            init.pitchDeg = zupt.alignAtt.pitch * RAD_TO_DEG;
            init.headingDeg = zupt.alignAtt.yaw * RAD_TO_DEG;
            staticEnd = zupt.alignCount;
            {
                std::lock_guard lk(resMutex);
                r->align.used = true;
                r->align.spanSec = static_cast<double>(zupt.alignCount) / scale.rateHz;
                r->align.rollDeg = init.rollDeg;
                r->align.pitchDeg = init.pitchDeg;
                r->align.headingDeg = init.headingDeg;
                r->align.earthRateErrDeg = zupt.alignEarthRateErrDeg;
                r->align.gyroSigmaRad = zupt.alignGyroSigmaRad;
                r->align.yawSpreadDeg = zupt.alignYawSpreadDeg;
                r->align.headingReliable = zupt.alignHeadingReliable;
            }
            LOG_INFO << "[INS] 静态粗对准: 开头 " << static_cast<double>(zupt.alignCount) / scale.rateHz
                    << "s, roll=" << init.rollDeg << " pitch=" << init.pitchDeg
                    << " yaw=" << init.headingDeg << " deg, 航向散布=" << zupt.alignYawSpreadDeg << " deg";
        } else {
            init.rollDeg = c.rollDeg;
            init.pitchDeg = c.pitchDeg;
            init.headingDeg = c.headingDeg;
        }

        free.init(init);
        zupt.init(init);
        // 静置段标定：加计零偏进转换、参考矢量进判据，全部落在 zupt 实例内。
        zupt.calibrate(head, staticEnd);
        if (zupt.biasApplied) {
            {
                std::lock_guard lk(resMutex);
                r->accelBias = zupt.accelBias;
                r->accelBiasApplied = true;
            }
            LOG_INFO << "[INS] 静置段加计零偏: " << zupt.accelBias.transpose() << " m/s^2（已在比力增量中扣除）"
                    << ", 陀螺偏离地球自转 " << zupt.gyroBiasDiag.transpose() << " rad/s（含航向误差，未扣）";
        } else if (staticEnd > 1) {
            LOG_INFO << "[INS] 静置段比力偏离重力投影 " << zupt.accelBias.norm() << " m/s^2，过大，未作零偏扣除";
        }

        std::vector<NavState> batchFree, batchZupt;
        std::vector<unsigned char> batchFlag;
        std::vector<InsEpoch> batchImu;
        std::size_t batchZuptCount = 0;
        std::size_t processed = 0;

        const auto flush = [&] {
            if (batchFree.empty()) return;
            {
                std::lock_guard lk(resMutex);
                r->nav.insert(r->nav.end(), batchFree.begin(), batchFree.end());
                r->navZupt.insert(r->navZupt.end(), batchZupt.begin(), batchZupt.end());
                r->zuptApplied.insert(r->zuptApplied.end(), batchFlag.begin(), batchFlag.end());
                r->imu.insert(r->imu.end(), batchImu.begin(), batchImu.end());
                r->zuptCount += batchZuptCount;
            }
            batchFree.clear();
            batchZupt.clear();
            batchFlag.clear();
            batchImu.clear();
            batchZuptCount = 0;
        };

        // 头段第一个样本就是初始时刻，其余回填队列，逐历元推进只留一条循环。
        {
            std::lock_guard lk(queueMutex);
            imuQueue.insert(imuQueue.begin(), head.begin() + 1, head.end());
        }
        zupt.prevRaw = head.front(); // 转换接续点：首个被转换样本是 head[1]
        zupt.hasPrevRaw = true;

        while (true) {
            ImuSample smp;
            if (!nextImu(smp)) break;
            if (stop.load()) break;
            if (maxEpochs > 0 && static_cast<int>(processed) >= maxEpochs) break;

            const InsEpoch e = zupt.toEpoch(smp);

            free.update(e);
            zupt.update(e);
            batchFree.push_back(free.s);
            batchZupt.push_back(zupt.s);
            batchFlag.push_back(zupt.zuptApplied ? 1 : 0);
            if (zupt.zuptApplied) ++batchZuptCount;
            batchImu.push_back(e);

            ++processed;
            if (batchFree.size() >= kFlushEpochs) flush();
        }
        flush();

        std::size_t total = 0, zuptCount = 0;
        {
            std::lock_guard lk(resMutex);
            total = r->nav.size();
            zuptCount = r->zuptCount;
        }
        LOG_INFO << "[INS] 解算完成: " << total << " 历元, 采样率 " << scale.rateHz << " Hz";
        LOG_INFO << "[INS] 零速判据: 生效 " << (total == 0 ? 0.0 : 100.0 * zuptCount / total)
                << "%, 比力偏差 max=" << zupt.zuptDiag.accDevMax << " m/s^2 (超限 " << zupt.zuptDiag.accFail
                << "), 角速度偏差 max=" << zupt.zuptDiag.gyrDevMax << " rad/s (超限 " << zupt.zuptDiag.gyrFail << ")";
        finishSolving();
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
        const Euler se = s.qbn.toEuler();
        row.set(F::Roll, se.roll * RAD_TO_DEG);
        row.set(F::Pitch, se.pitch * RAD_TO_DEG);
        row.set(F::Heading, se.yaw * RAD_TO_DEG);
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
            const Euler za = z.qbn.toEuler();
            row.set(F::ZuptRoll, za.roll * RAD_TO_DEG);
            row.set(F::ZuptPitch, za.pitch * RAD_TO_DEG);
            row.set(F::ZuptHeading, za.yaw * RAD_TO_DEG);
            double zn = 0.0, ze = 0.0;
            Coord::LLHToLocalNE(z.llh[0], z.llh[1], lat0, lon0, zn, ze);
            row.set(F::ZuptE, ze);
            row.set(F::ZuptN, zn);
        }
        if (i < snap->zuptApplied.size()) row.set(F::ZuptFlag, snap->zuptApplied[i]);
        if (i < snap->imu.size() && snap->imu[i].h > 0.0) {
            // 一次采样的增量除以间隔就是机体系比力与角速度（已扣静置段加计零偏）。
            const InsEpoch &e = snap->imu[i];
            row.set(F::Axb, e.dvel[0] / e.h);
            row.set(F::Ayb, e.dvel[1] / e.h);
            row.set(F::Azb, e.dvel[2] / e.h);
            row.set(F::Wxb, e.dtheta[0] / e.h);
            row.set(F::Wyb, e.dtheta[1] / e.h);
            row.set(F::Wzb, e.dtheta[2] / e.h);
        }
    }


    std::vector<Export::Col> Task::exportFields(const Export::Format fmt) const {
        if (fmt == Export::Format::ORB) return {}; // 根数只有低轨轨道有
        return insFields();
    }

    std::vector<Export::Col> Task::exportDefaults(const Export::Format fmt) const {
        return fmt == Export::Format::CSV ? insCSV() : std::vector<Export::Col>{};
    }

    void Task::writeCsvParams(CSV::Meta &m) const {
        const auto &ins = opts.ins;
        m.xyz("初始位置 BLH (°/°/m)", ins.latDeg, ins.lonDeg, ins.hM);
        m.xyz("初始速度 vNED (m/s)", ins.vN, ins.vE, ins.vD);
        m.flag("手填初始姿态角", ins.specifyAttitude);
        m.num("手填横滚 (°)", ins.rollDeg);
        m.num("手填俯仰 (°)", ins.pitchDeg);
        m.num("手填航向 (°)", ins.headingDeg);
        m.num("加计 (m/s)/LSB", ins.manual.accelMpsPerLsb);
        m.num("陀螺 rad/LSB", ins.manual.gyroRadPerLsb);
        m.num("采样率 (Hz)", ins.manual.rateHz);
        m.num("比力判静止容差 (m/s²)", ins.zuptAccelTol);
        m.num("角速度容差 (rad/s)", ins.zuptGyroTol);
        m.num("连续静止时长 (s)", ins.zuptHoldSec);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->opts = cfg;
        task->filePath = cfg.ins.imuPath;
        task->fileName = File::baseName(cfg.ins.imuPath);
        return task;
    }
}
