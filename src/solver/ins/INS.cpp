#include "INS.h"

#include <algorithm>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "Aligner.h"
#include "core/Const.h"
#include "Exception.h"
#include "Observer.h"


namespace {
    double meridianRadius(const double lat, const double h) {
        const double s = std::sin(lat), w2 = 1.0 - Frame::CGCS2000.e2 * s * s;
        return Frame::CGCS2000.a * (1.0 - Frame::CGCS2000.e2) / (w2 * std::sqrt(w2)) + h;
    }

    double primeVerticalRadius(const double lat, const double h) {
        const double s = std::sin(lat);
        return Frame::CGCS2000.a / std::sqrt(1.0 - Frame::CGCS2000.e2 * s * s) + h;
    }

    // ω^n_ie (式 6.180i), NED
    Eigen::Vector3d earthRateNED(const double lat) {
        return {OMEGA_EARTH * std::cos(lat), 0.0, -OMEGA_EARTH * std::sin(lat)};
    }

    // ω^n_en (式 6.180j), NED
    Eigen::Vector3d transportRateNED(const double lat, const double vN, const double vE, const double h) {
        const double rm = meridianRadius(lat, h), rn = primeVerticalRadius(lat, h);
        return {vE / rn, -vN / rm, -vE * std::tan(lat) / rn};
    }

    // 圆周散布：一组航向角 (rad) 相对圆均值的离散度，deg
    double circularSpreadDeg(const std::vector<double> &angles) {
        if (angles.size() < 2) return 0.0;
        double sx = 0.0, sy = 0.0;
        for (const double a: angles) {
            sx += std::cos(a);
            sy += std::sin(a);
        }
        const double r = std::hypot(sx, sy) / static_cast<double>(angles.size());
        return std::sqrt(-2.0 * std::log(std::max(r, 1e-12))) * RAD_TO_DEG;
    }

}


void INS::align(const ImuLog &head, const double latRad) {
    if (!scale.known()) throw InvalidRequest("INS::align: IMU decode scale is not set");
    const double rate = scale.rateHz;
    SiteParams sp;
    sp.latDeg = latRad * RAD_TO_DEG;
    sp.g = gravity;
    const StaticSpan span = Aligner::detectStatic(head, scale);
    if (span.size() < static_cast<std::size_t>(Aligner::kMinStaticSec * rate))
        throw InvalidRequest("INS::align: static segment shorter than 20 s, automatic alignment unavailable");

    const AlignQuality q = Aligner::diagnose(head, span.begin, span.end, sp, scale);
    alignAtt = q.att;
    alignEarthRateErrDeg = q.earthRateErrDeg;
    std::vector<double> yaws;
    yaws.reserve(5);
    for (int k = 0; k < 5; ++k)
        yaws.push_back(Aligner::solve(head, span.begin + span.size() * k / 5,
                                      span.begin + span.size() * (k + 1) / 5, sp).yaw);
    alignYawSpreadDeg = circularSpreadDeg(yaws);

    Eigen::Vector3d sum = Eigen::Vector3d::Zero(), sq = Eigen::Vector3d::Zero();
    for (std::size_t i = span.begin; i < span.end; ++i) {
        const Eigen::Vector3d w(head[i].gx, head[i].gy, head[i].gz);
        sum += w;
        sq += w.cwiseProduct(w);
    }
    const auto n = static_cast<double>(span.size());
    const Eigen::Vector3d mean = sum / n;
    alignGyroSigmaRad = std::sqrt(std::max(0.0, (sq / n - mean.cwiseProduct(mean)).sum())) *
                        scale.gyroRadPerLsb * rate;
    alignBegin = span.begin;
    alignCount = span.size();
    alignHeadingReliable = alignEarthRateErrDeg < 1.0 && alignYawSpreadDeg < 0.5;
}


void INS::init(const NavState &x0) {
    s = x0;
    prevDtheta.setZero();
    prevDvel.setZero();
    prevPrevPos = x0.llh;
    prevPrevVel = x0.vned;
    hasPrev = false;
    gravity = getGravity(x0.llh[0], x0.llh[2]);
    gyroEst.setZero();
    winHist.clear();
    fSum.setZero();
    wSum.setZero();
    hSum = 0.0;
    quiet = 0;
    zuptApplied = false;
    zuptDiag = {};
    zuptHoldN = static_cast<std::size_t>(std::max(1, static_cast<int>(zuptHoldSec * scale.rateHz)));
    zuptWinN = static_cast<std::size_t>(std::max(1.0, scale.rateHz));
    zuptEstWarmN = static_cast<std::size_t>(std::max(1.0, kZuptEstWarmSec * scale.rateHz));
    prevRaw = {};
    hasPrevRaw = false;
}


void INS::calibrate(const ImuLog &head) {
    accelBias.setZero();
    biasApplied = false;
    gyroBiasDiag.setZero();
    zuptFRef.setZero();
    zuptWRef.setZero();
    zuptHasRef = false;
    const std::size_t end = alignBegin + alignCount;
    if (end <= alignBegin + 1 || end > head.size()) return;
    double hSumCalib = 0.0;
    for (std::size_t i = alignBegin; i < end; ++i) {
        const InsEpoch e = makeEpoch(head[i], i > 0 ? &head[i - 1] : nullptr);
        zuptFRef += e.dvel;
        zuptWRef += e.dtheta;
        hSumCalib += e.h;
    }
    zuptFRef /= hSumCalib;
    zuptWRef /= hSumCalib;
    // 静止时比力应为重力投影，偏离量即加计零偏；陀螺项同时含航向误差，静止下不可分，只作诊断。
    const Eigen::Vector3d fNominal = s.qbn.conjugated().rotate(Eigen::Vector3d(0.0, 0.0, -gravity));
    const Eigen::Vector3d wNominal = s.qbn.conjugated().rotate(
        Eigen::Vector3d(OMEGA_EARTH * std::cos(s.llh[0]), 0.0, -OMEGA_EARTH * std::sin(s.llh[0])));
    accelBias = zuptFRef - fNominal;
    gyroBiasDiag = zuptWRef - wNominal;
    biasApplied = accelBias.norm() < kAccBiasApplyTol;
    zuptHasRef = true;
}


InsEpoch INS::toEpoch(const ImuSample &sm) {
    InsEpoch out = makeEpoch(sm, hasPrevRaw ? &prevRaw : nullptr);
    prevRaw = sm;
    hasPrevRaw = true;
    return out;
}


void INS::update(const InsEpoch &e) {
    if (!zuptEnabled) {
        mechanize(e);
        return;
    }

    Eigen::Vector3d fMean = Eigen::Vector3d::Zero();
    const bool still = detectStatic(e, fMean);

    // 机械编排推进；增量先扣静止段折算出的陀螺零偏
    const Eigen::Vector3d prevLlh = s.llh;
    InsEpoch ei = e;
    ei.dtheta -= gyroEst * e.h;
    mechanize(ei);

    quiet = still ? quiet + 1 : 0;
    zuptApplied = quiet >= static_cast<int>(zuptHoldN);
    if (zuptApplied) holdStill(e, fMean, prevLlh);
}


bool INS::detectStatic(const InsEpoch &e, Eigen::Vector3d &fMean) {
    fSum += e.dvel;
    wSum += e.dtheta;
    hSum += e.h;
    winHist.push_back(e);
    while (winHist.size() > zuptWinN) {
        const InsEpoch &old = winHist.front();
        fSum -= old.dvel;
        wSum -= old.dtheta;
        hSum -= old.h;
        winHist.pop_front();
    }
    fMean.setZero();
    if (hSum <= 0.0) return false;
    fMean = fSum / hSum;
    const Eigen::Vector3d wMean = wSum / hSum;
    double fs = 0.0, fs2 = 0.0, ws = 0.0, ws2 = 0.0;
    for (const InsEpoch &q : winHist) {
        const double fm = q.dvel.norm() / q.h, wm = q.dtheta.norm() / q.h;
        fs += fm; fs2 += fm * fm; ws += wm; ws2 += wm * wm;
    }
    const double nwin = static_cast<double>(winHist.size());
    const double fBar = fs / nwin, fStd = std::sqrt(std::max(0.0, fs2 / nwin - fBar * fBar));
    const double wBar = ws / nwin, wStd = std::sqrt(std::max(0.0, ws2 / nwin - wBar * wBar));
    if (zuptHasRef) {
        const double accDev = (fMean - zuptFRef).norm();
        const double gyrDev = wMean.norm();
        zuptDiag.accDevMax = std::max(zuptDiag.accDevMax, accDev);
        zuptDiag.gyrDevMax = std::max(zuptDiag.gyrDevMax, gyrDev);
        if (accDev >= zuptAccelTol) ++zuptDiag.accFail;
        if (gyrDev >= zuptGyroTol) ++zuptDiag.gyrFail;
        return accDev < zuptAccelTol && gyrDev < zuptGyroTol &&
               fStd < kZuptAccStdTol && wStd < kZuptGyroStdTol;
    }
    return std::fabs(fBar - gravity) < zuptAccelTol && wBar < zuptGyroTol &&
           fStd < kZuptAccStdTol && wStd < kZuptGyroStdTol;
}


void INS::holdStill(const InsEpoch &e, const Eigen::Vector3d &fMean, const Eigen::Vector3d &prevLlh) {
    s.vned.setZero();
    s.llh = prevLlh;
    if (quiet < static_cast<int>(zuptEstWarmN)) return;
    const Eigen::Vector3d vn = s.qbn.rotate(fMean);
    const Eigen::Vector3d tgt(0.0, 0.0, -gravity);
    const Eigen::Vector3d axis = vn.cross(tgt);
    if (const double sn = axis.norm(); sn > 1e-12) {
        const Eigen::Vector3d dir = axis / sn; // 单位转轴
        const double ang = std::atan2(sn, vn.dot(tgt));
        const Eigen::Vector3d dphi = dir * ang; // n 系待补转角
        const double step = std::min(kZuptAttGain * ang, kZuptAttRateMax * e.h);
        const Eigen::Vector3d corr = dir * step;
        s.qbn = (ATTQuat::fromRotationVector(corr) * s.qbn).normalized();
        if (const Eigen::Vector3d implied = kZuptAttGain * s.qbn.conjugated().rotate(dphi) / e.h; implied.norm() <= kZuptGyroBiasCap) {
            const double a = std::min(1.0, e.h / kZuptGyroEstTau);
            gyroEst = (1.0 - a) * gyroEst - a * implied;
        }
        gyroEst = gyroEst.cwiseMax(-kZuptGyroBiasCap).cwiseMin(kZuptGyroBiasCap);
    }
}


void INS::mechanize(const InsEpoch &e) {
    const double dt = e.h;
    const ATTQuat qOld = s.qbn;
    const Eigen::Vector3d pOld = s.llh;
    const Eigen::Vector3d vOld = s.vned;

    //外推中间时刻位置/速度
    const Eigen::Vector3d pMid = 1.5 * pOld - 0.5 * prevPrevPos;
    const Eigen::Vector3d vMid = 1.5 * vOld - 0.5 * prevPrevVel;
    const double latMid = pMid[0], hMid = pMid[2];
    const double vNmid = vMid[0], vEmid = vMid[1];

    //中间时刻地理参数
    const double g = getGravity(latMid, hMid);
    const Eigen::Vector3d wEarth = earthRateNED(latMid);
    const Eigen::Vector3d wTrans = transportRateNED(latMid, vNmid, vEmid, hMid);
    const Eigen::Vector3d wIn = wEarth + wTrans;
    const Eigen::Vector3d zeta = wIn * dt;

    //b 系比力积分项: 旋转补偿 + 划桨补偿
    Eigen::Vector3d dvb = e.dvel;
    if (hasPrev)
        dvb += 0.5 * e.dtheta.cross(e.dvel) + 1.0 / 12.0 * (prevDtheta.cross(e.dvel) + prevDvel.cross(e.dtheta));

    // 投影到 n 系
    Eigen::Vector3d dvn = qOld.rotate(dvb);
    dvn -= 0.5 * zeta.cross(dvn);

    //重力/哥氏积分项
    const Eigen::Vector3d gVec(0.0, 0.0, g);
    const Eigen::Vector3d agc = gVec - (2.0 * wEarth + wTrans).cross(vMid);
    const Eigen::Vector3d dvgcor = agc * dt;

    //速度更新
    const Eigen::Vector3d vNew = vOld + dvn + dvgcor;

    //位置更新
    const Eigen::Vector3d vPosMid = 0.5 * (vOld + vNew);
    const double vN = vPosMid[0], vE = vPosMid[1], vD = vPosMid[2];

    const double hNew = s.llh[2] - vD * dt;
    const double hHalf = 0.5 * (s.llh[2] + hNew);

    const double latNew = s.llh[0] + vN * dt / meridianRadius(s.llh[0], hHalf);
    const double latHalf = 0.5 * (s.llh[0] + latNew);

    const double lonNew = s.llh[1] + vE * dt / (primeVerticalRadius(latHalf, hHalf) * std::cos(latHalf));

    const Eigen::Vector3d pNew = Eigen::Vector3d(latNew, lonNew, hNew);
    s.llh = pNew;
    s.vned = vNew;
    s.t = e.t;

    //姿态更新
    //姿态步独立重算中点
    const Eigen::Vector3d pMidA = 0.5 * (pOld + pNew);
    const Eigen::Vector3d vMidA = 0.5 * (vOld + vNew);
    const double latMA = pMidA[0], hMA = pMidA[2];
    const double vNma = vMidA[0], vEma = vMidA[1];
    const Eigen::Vector3d wEa = earthRateNED(latMA);
    const Eigen::Vector3d wTa = transportRateNED(latMA, vNma, vEma, hMA);
    const Eigen::Vector3d zetaA = (wEa + wTa) * dt;
    const ATTQuat qnn = ATTQuat::fromRotationVector(-zetaA);
    Eigen::Vector3d phi = e.dtheta;
    if (hasPrev) phi += 1.0 / 12.0 * prevDtheta.cross(e.dtheta); // 圆锥补偿
    const ATTQuat qbb = ATTQuat::fromRotationVector(phi);
    s.qbn = (qnn * qOld * qbb).normalized(); // 链乘

    // 历史 (移动窗口)
    prevPrevPos = pOld;
    prevPrevVel = vOld;
    prevDtheta = e.dtheta;
    prevDvel = e.dvel;
    hasPrev = true;
}


InsEpoch INS::makeEpoch(const ImuSample &sm, const ImuSample *prev) const {
    const double nominalH = 1.0 / scale.rateHz;
    InsEpoch out;
    out.t = sm.tow;
    out.h = prev ? sm.tow - prev->tow : nominalH;
    if (!validEpochGap(out.h)) out.h = nominalH;
    out.dtheta = Eigen::Vector3d(sm.gx, sm.gy, sm.gz) * scale.gyroRadPerLsb;
    out.dvel = Eigen::Vector3d(sm.ax, sm.ay, sm.az) * scale.accelMpsPerLsb -
               (biasApplied ? accelBias : Eigen::Vector3d::Zero()) * out.h;
    return out;
}
