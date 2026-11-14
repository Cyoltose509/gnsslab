#include "InsVerify.h"

#include <algorithm>
#include <utility>

#include "core/Const.h"
#include "core/geom/Geodesy.h"

namespace {
    /// IMU.bin：t, Δθ(3), Δv(3)。
    constexpr std::size_t kImuStride = 7;
    /// PureINS.bin：t, φ, λ, h, vN, vE, vD, roll, pitch, heading。
    constexpr std::size_t kRefStride = 10;
    constexpr std::size_t kMaxCurvePts = 4000;

    double wrapDeg(double d) {
        while (d > 180.0) d -= 360.0;
        while (d <= -180.0) d += 360.0;
        return d;
    }

    Eigen::Vector3d triple(const double *rec, const std::size_t first) {
        return {rec[first], rec[first + 1], rec[first + 2]};
    }
}

Ins::Verifier::Verifier(const std::string &imuPath, const std::string &refPath, const VerifyOptions &opt) {
    report.tol = opt.tol;
    m_refRec.resize(kRefStride);
    if (!m_imu.open(imuPath, kImuStride) || m_imu.bad()) {
        m_error = m_imu.error().empty() ? "无法打开 IMU 增量文件：" + imuPath : m_imu.error();
        return;
    }
    if (!m_ref.open(refPath, kRefStride) || m_ref.bad()) {
        m_error = m_ref.error().empty() ? "无法打开参考轨迹文件：" + refPath : m_ref.error();
        return;
    }
    if (m_imu.empty()) {
        m_error = "IMU 增量文件里没有完整的 7 列记录（t, Δθ×3, Δv×3）：" + imuPath;
        return;
    }
    if (m_ref.empty()) {
        m_error = "参考轨迹文件里没有完整的 10 列记录（t, φ, λ, h, v×3, 姿态×3）：" + refPath;
        return;
    }
    report.refEpochs = m_ref.count();
    m_curveStride = std::max<std::size_t>(1, report.refEpochs / kMaxCurvePts);

    // 增量在 t 处覆盖 [t−dt, t]（后向约定）：初始状态落在起始时刻那条记录上，
    // 第一个能消费的增量是下一条。
    const double *rec = nullptr;
    bool found = false;
    while (m_imu.next(rec)) {
        if (rec[0] >= opt.startTime) {
            found = true;
            break;
        }
    }
    if (!found || m_imu.consumed() >= m_imu.count()) {
        m_error = "IMU 文件在起始时刻之后不足两条记录，无法推进机械编排。";
        return;
    }

    InsInit init = opt.init;
    init.t0 = rec[0];
    m_ins.init(init);
    m_t0 = m_tPrev = init.t0;

    while (m_ref.next(rec)) {
        if (rec[0] >= init.t0) {
            std::copy_n(rec, kRefStride, m_refRec.begin());
            m_refPending = true;
            break;
        }
    }
    if (!m_refPending) {
        m_error = "参考轨迹全部早于 IMU 起始时刻，没有可比对的数据。";
        return;
    }
    m_ready = true;
}

void Ins::Verifier::compare(const NavState &ns, const double *rec) {
    const Eigen::Vector3d refLlh(rec[1] * DEG_TO_RAD, rec[2] * DEG_TO_RAD, rec[3]);
    const Eigen::Vector3d refVel(rec[4], rec[5], rec[6]);
    const auto [roll, pitch, yaw] = ns.qbn.toEuler();

    const double dLat = ns.llh[0] - refLlh[0];
    const double dLon = ns.llh[1] - refLlh[1];
    const double rm = Geodesy::meridianRadius(refLlh[0], Frame::CGCS2000) + refLlh[2];
    const double rn = Geodesy::primeVerticalRadius(refLlh[0], Frame::CGCS2000) + refLlh[2];

    VerifySample s;
    s.t = rec[0] - m_t0;
    s.dN = dLat * rm;
    s.dE = dLon * rn * std::cos(refLlh[0]);
    s.dU = ns.llh[2] - refLlh[2];
    s.dVn = ns.vned[0] - refVel[0];
    s.dVe = ns.vned[1] - refVel[1];
    s.dVd = ns.vned[2] - refVel[2];
    s.dRoll = wrapDeg(roll * RAD_TO_DEG - rec[7]);
    s.dPitch = wrapDeg(pitch * RAD_TO_DEG - rec[8]);
    s.dHeading = wrapDeg(yaw * RAD_TO_DEG - rec[9]);

    report.maxLatDeg = std::max(report.maxLatDeg, std::fabs(dLat * RAD_TO_DEG));
    report.maxLonDeg = std::max(report.maxLonDeg, std::fabs(dLon * RAD_TO_DEG));
    report.maxHM = std::max(report.maxHM, std::fabs(s.dU));
    report.maxV = std::max({report.maxV, std::fabs(s.dVn), std::fabs(s.dVe), std::fabs(s.dVd)});
    report.maxRollDeg = std::max(report.maxRollDeg, std::fabs(s.dRoll));
    report.maxPitchDeg = std::max(report.maxPitchDeg, std::fabs(s.dPitch));
    report.maxHeadingDeg = std::max(report.maxHeadingDeg, std::fabs(s.dHeading));
    report.maxHorizontalM = std::max(report.maxHorizontalM, std::hypot(s.dN, s.dE));

    if (report.compared % m_curveStride == 0) report.series.push_back(s);
    if (report.compared == 0) report.t0 = rec[0];
    report.t1 = rec[0];
    m_last = s;
    m_hasLast = true;
    ++report.compared;
}

bool Ins::Verifier::step() {
    const double *rec = nullptr;
    if (!m_imu.next(rec)) {
        if (m_imu.bad()) m_error = m_imu.error();
        return false;
    }
    InsEpoch e;
    e.t = rec[0];
    e.dtheta = triple(rec, 1);
    e.dvel = triple(rec, 4);
    e.h = e.t - m_tPrev;
    ++m_imuEpochs;
    if (!validEpochGap(e.h)) {  // 时间戳断档：这一步不推进，也不与参考比
        m_tPrev = e.t;
        return true;
    }

    const NavState before = m_ins.s;
    m_ins.update(e);

    // 参考时标落在 (tPrev, e.t] 内：与前后两个状态里更近的那个比。
    while (m_refPending && m_refRec[0] <= e.t) {
        compare(m_refRec[0] - m_tPrev < e.t - m_refRec[0] ? before : m_ins.s, m_refRec.data());
        if (const double *q = nullptr; m_ref.next(q)) {
            std::copy_n(q, kRefStride, m_refRec.begin());
        } else {
            m_refPending = false;
            if (m_ref.bad()) m_error = m_ref.error();
        }
    }
    m_tPrev = e.t;
    return true;
}

void Ins::Verifier::finish() {
    if (m_hasLast && report.compared % m_curveStride != 0) report.series.push_back(m_last);
    report.imuEpochs = m_imuEpochs;
    report.pass = report.maxLatDeg <= report.tol.latLonDeg && report.maxLonDeg <= report.tol.latLonDeg &&
                  report.maxHM <= report.tol.hM && report.maxV <= report.tol.vMps &&
                  report.maxRollDeg <= report.tol.attDeg && report.maxPitchDeg <= report.tol.attDeg &&
                  report.maxHeadingDeg <= report.tol.attDeg;
}

bool Ins::verifySample(const std::string &imuPath, const std::string &refPath, const VerifyOptions &opt,
                       VerifyReport &out, std::string &error) {
    out = VerifyReport{};
    error.clear();
    Verifier v(imuPath, refPath, opt);
    if (!v.ready()) {
        error = v.error();
        return false;
    }
    while (v.step()) {}
    if (!v.error().empty()) {
        error = v.error();
        return false;
    }
    v.finish();
    if (v.report.compared == 0) {
        error = "参考轨迹与 IMU 时段没有交集，未产生任何比对点。";
        return false;
    }
    out = std::move(v.report);
    return true;
}
