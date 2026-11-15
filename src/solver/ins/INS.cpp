#include "INS.h"
#include "i18n/I18n.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "Aligner.h"
#include "core/Const.h"
#include "Observer.h"


namespace {
    constexpr double kE2 = Frame::CGCS2000.e2;
    // 零速修正内参
    constexpr double kZuptAttGain = 0.05;      // 静止姿态向重力方向的单历元修正比例
    constexpr double kZuptGyroEstTau = 10.0;   // 陀螺零偏一阶滤波时间常数 (s)
    constexpr double kZuptGyroBiasCap = 1e-4;  // 陀螺零偏估计钳位 (rad/s)
    // 静置段标定内参
    constexpr double kAccBiasApplyTol = 0.05;  // 加计零偏可信阈值 (m/s^2)，超过则不扣


    double meridianRadius(const double lat, const double h) {
        const double s = std::sin(lat), w2 = 1.0 - kE2 * s * s;
        return Frame::CGCS2000.a * (1.0 - kE2) / (w2 * std::sqrt(w2)) + h;
    }

    double primeVerticalRadius(const double lat, const double h) {
        const double s = std::sin(lat);
        return Frame::CGCS2000.a / std::sqrt(1.0 - kE2 * s * s) + h;
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

    // 无型号日志时按开头静置段的 |f| 量级反推并匹配内置型号表
    bool reverseScale(const ImuLog &log, const double gravity, INS &ins) {
        constexpr int kCandidates = 3;
        const std::size_t n = log.size();
        if (n < 200) {
            ins.scaleNote = I18n::tr("ins.scale.reverse_too_few");
            return false;
        }
        for (int k = 0; k < kCandidates; ++k) {
            constexpr std::size_t kStride = 4000;
            constexpr std::size_t kWindow = 4000;
            const std::size_t begin = static_cast<std::size_t>(k) * kStride;
            if (begin >= n) break;
            const std::size_t end = std::min(begin + kWindow, n);
            if (end - begin < 200) break;
            const ImuLog seg(log.begin() + static_cast<std::ptrdiff_t>(begin),
                             log.begin() + static_cast<std::ptrdiff_t>(end));
            const ImuScaleGuess gs = guessImuScale(seg, gravity);
            if (!gs.model) {
                if (ins.scaleNote.empty()) ins.scaleNote = gs.note;
                continue;
            }
            ins.scale = gs.model->scale;
            ins.scaleName = gs.model->name;
            ins.scaleSource = ImuScaleSource::Reverse;
            ins.scaleNote = gs.note.empty()
                                ? I18n::tr("ins.scale.reverse_ok")
                                : I18n::tr("ins.scale.reverse_note", gs.note);
            return true;
        }
        return false;
    }
}


bool INS::resolveScale(const ImuLog &head, const ImuModel *detected, const ImuScale &manual) {
    scaleName.clear();
    scaleNote.clear();
    scaleError.clear();
    if (head.empty()) {
        scaleError = I18n::tr("ins.err.no_records");
        return false;
    }
    if (manual.known()) {
        scale = manual;
        scaleSource = ImuScaleSource::Manual;
        scaleNote = I18n::tr("ins.scale.note_manual");
        return true;
    }
    if (detected) {
        scale = detected->scale;
        scaleName = detected->name;
        scaleSource = ImuScaleSource::Record;
        scaleNote = I18n::tr("ins.scale.from_record");
        return true;
    }
    if (!reverseScale(head, gravity, *this)) {
        scaleError = I18n::tr("ins.scale.unknown", scaleNote);
        return false;
    }
    return true;
}


bool INS::align(const ImuLog &log, const double latRad) {
    alignOk = false;
    alignError.clear();
    const double rate = scale.rateHz > 0.0 ? scale.rateHz : 200.0;
    SiteParams sp;
    sp.latDeg = latRad * RAD_TO_DEG;
    sp.g = gravity;
    std::size_t end = 0;
    try {
        end = Aligner::staticHead(log, scale, sp);
        if (end < static_cast<std::size_t>(20.0 * rate)) {
            alignError = I18n::tr("ins.align.head_too_short");
            return false;
        }
        alignAtt = Aligner::solve(log, 0, end, sp);
        alignEarthRateErrDeg = Aligner::diagnose(log, 0, end, sp, scale).earthRateErrDeg;
        std::vector<double> yaws;
        for (int k = 0; k < 5; ++k)
            yaws.push_back(Aligner::solve(log, end * k / 5, end * (k + 1) / 5, sp).yaw);
        alignYawSpreadDeg = circularSpreadDeg(yaws);
    } catch (const std::exception &ex) {
        alignError = I18n::tr("ins.align.failed", ex.what());
        return false;
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero(), sq = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < end; ++i) {
        const Eigen::Vector3d w(log[i].gx, log[i].gy, log[i].gz);
        sum += w;
        sq += w.cwiseProduct(w);
    }
    const double n = static_cast<double>(end);
    const Eigen::Vector3d mean = sum / n;
    alignGyroSigmaRad = std::sqrt(std::max(0.0, (sq / n - mean.cwiseProduct(mean)).sum())) *
                        scale.gyroRadPerLsb * rate;
    alignCount = end;
    alignHeadingReliable = alignEarthRateErrDeg < 1.0 && alignYawSpreadDeg < 0.5;
    alignOk = true;
    return true;
}


void INS::init(const InsInit &init) {
    s.t = init.t0;
    s.llh = init.llh;
    s.vned = init.vned;
    s.qbn = ATTQuat::fromEuler({
        init.rollDeg * DEG_TO_RAD, init.pitchDeg * DEG_TO_RAD,
        init.headingDeg * DEG_TO_RAD
    });
    prevDtheta.setZero();
    prevDvel.setZero();
    prevPrevPos = init.llh;
    prevPrevVel = init.vned;
    hasPrev = false;
    gravity = NormalGravity::at(init.llh[0], init.llh[2]);
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
    prevRaw = {};
    hasPrevRaw = false;
}


void INS::calibrate(const ImuLog &head, const std::size_t end) {
    accelBias.setZero();
    biasApplied = false;
    gyroBiasDiag.setZero();
    zuptFRef.setZero();
    zuptWRef.setZero();
    zuptHasRef = false;
    if (end <= 1) return;
    double hSumCalib = 0.0;
    for (std::size_t i = 0; i < end; ++i) {
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


InsEpoch INS::makeEpoch(const ImuSample &sm, const ImuSample *prev) const {
    const double nominalH = scale.rateHz > 0.0 ? 1.0 / scale.rateHz : 0.0;
    InsEpoch out;
    out.t = sm.tow;
    out.h = prev ? sm.tow - prev->tow : nominalH;
    if (!validEpochGap(out.h)) out.h = nominalH;
    out.dtheta = Eigen::Vector3d(sm.gx, sm.gy, sm.gz) * scale.gyroRadPerLsb;
    out.dvel = Eigen::Vector3d(sm.ax, sm.ay, sm.az) * scale.accelMpsPerLsb -
               (biasApplied ? accelBias : Eigen::Vector3d::Zero()) * out.h;
    return out;
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
    // 判静窗口：输入增量累计约 1 s 求均值，与静置段基准比较。
    // 用矢量窗口均值而不是 |f|≈g：后者对匀速与发散段同样成立，等于没有判据；
    // 重复时间戳的样本 h 全为 0，均值会变成 inf/NaN，此窗口直接判非静止。
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
    if (zuptHasRef) {
        const double accDev = (fMean - zuptFRef).norm();
        const double gyrDev = (wMean - zuptWRef).norm();
        zuptDiag.accDevMax = std::max(zuptDiag.accDevMax, accDev);
        zuptDiag.gyrDevMax = std::max(zuptDiag.gyrDevMax, gyrDev);
        if (accDev >= zuptAccelTol) ++zuptDiag.accFail;
        if (gyrDev >= zuptGyroTol) ++zuptDiag.gyrFail;
        return accDev < zuptAccelTol && gyrDev < zuptGyroTol;
    }
    // 无静置基准（手填姿态、没有对准段）：|f|≈g 且角速度小的幅值判据
    return std::fabs(e.dvel.norm() / e.h - gravity) < zuptAccelTol &&
           e.dtheta.norm() / e.h < zuptGyroTol;
}


void INS::holdStill(const InsEpoch &e, const Eigen::Vector3d &fMean, const Eigen::Vector3d &prevLlh) {
    // 静止：速度、位置都不应变，姿态以重力方向为准。
    s.vned.setZero();
    s.llh = prevLlh;
    const Eigen::Vector3d vn = s.qbn.rotate(fMean);
    const Eigen::Vector3d tgt(0.0, 0.0, -gravity);
    const Eigen::Vector3d axis = vn.cross(tgt);
    const double sn = axis.norm();
    if (sn > 1e-12) {
        const double ang = std::atan2(sn, vn.dot(tgt));
        const Eigen::Vector3d dphi = axis / sn * ang; // n 系待补转角
        s.qbn = (ATTQuat::fromRotationVector(kZuptAttGain * dphi) * s.qbn).normalized();
        // 待补转角持续同号即陀螺在推姿态：按一阶滤波折算成机体系零偏，供运动段扣除
        const double a = std::min(1.0, e.h / kZuptGyroEstTau);
        gyroEst = (1.0 - a) * gyroEst +
                  a * (s.qbn.conjugated().rotate(dphi) / e.h);
        gyroEst = gyroEst.cwiseMax(-kZuptGyroBiasCap).cwiseMin(kZuptGyroBiasCap);
    }
}


void INS::mechanize(const InsEpoch &e) {
    const double dt = e.h;
    const ATTQuat qOld = s.qbn; // q^{n(k-1)}_{b(k-1)}
    const Eigen::Vector3d curPos = s.llh; // p_{k-1}
    const Eigen::Vector3d curVel = s.vned; // v_{k-1}

    // (1) 外推中间时刻位置/速度 (式 6.206)
    const Eigen::Vector3d pMid = 1.5 * curPos - 0.5 * prevPrevPos;
    const Eigen::Vector3d vMid = 1.5 * curVel - 0.5 * prevPrevVel;
    const double latMid = pMid[0], hMid = pMid[2];
    const double vNmid = vMid[0], vEmid = vMid[1];

    // (2) 中间时刻地理参数
    const double g = NormalGravity::at(latMid, hMid);
    const Eigen::Vector3d wEarth = earthRateNED(latMid);
    const Eigen::Vector3d wTrans = transportRateNED(latMid, vNmid, vEmid, hMid);
    const Eigen::Vector3d wIn = wEarth + wTrans; // ω^n_in = ω^n_ie + ω^n_en
    const Eigen::Vector3d zeta = wIn * dt; // ζ = ∫ω^n_in dt ≈ ω^n_in(t_{k-1/2})·Δt (式 6.180h)

    // (3) b 系比力积分项: 旋转补偿 + 划桨补偿 (式 6.225)
    Eigen::Vector3d dvb = e.dvel;
    if (hasPrev)
        dvb += 0.5 * e.dtheta.cross(e.dvel)
                + (1.0 / 12.0) * (prevDtheta.cross(e.dvel) + prevDvel.cross(e.dtheta));

    // 投影到 n 系: Δv^n_f = [I - 1/2(ζ×)] C^{b(k-1)}_{n(k-1)} Δv^b_f (式 6.213)
    Eigen::Vector3d dvn = qOld.rotate(dvb);
    dvn -= 0.5 * zeta.cross(dvn);

    // (4) 重力/哥氏积分项: agc = g - (2ω_ie + ω_en)×v, 取中点 (式 6.204, 6.205)
    const Eigen::Vector3d gVec(0.0, 0.0, g);
    const Eigen::Vector3d agc = gVec - (2.0 * wEarth + wTrans).cross(vMid);
    const Eigen::Vector3d dvgcor = agc * dt;

    // (5) 速度更新 (式 6.203a)
    const Eigen::Vector3d vNew = curVel + dvn + dvgcor;

    // ---- 位置更新 (6.4) ----
    const Eigen::Vector3d vPosMid = 0.5 * (curVel + vNew); // v_{k-1/2} 用于位置积分
    const double vN = vPosMid[0], vE = vPosMid[1], vD = vPosMid[2];

    const double hOld = s.llh[2];
    const double hNew = hOld - vD * dt; // 式 6.234
    const double hHalf = 0.5 * (hOld + hNew);

    // 注: meridianRadius / primeVerticalRadius 内部已含 +h (即 R_M+R_N 在飞行高度上的曲率半径)
    const double latOld = s.llh[0];
    const double latNew = latOld + vN * dt / meridianRadius(latOld, hHalf); // 式 6.235
    const double latHalf = 0.5 * (latOld + latNew);

    const double lonOld = s.llh[1];
    const double lonNew = lonOld + vE * dt /
                          (primeVerticalRadius(latHalf, hHalf) * std::cos(latHalf)); // 式 6.237

    s.llh = Eigen::Vector3d(latNew, lonNew, hNew);
    s.vned = vNew;
    s.t = e.t; // 增量@t 覆盖 [t-dt, t] (后向约定), 更新后状态时间即 e.t

    // ---- 姿态更新 (链乘, §6.5 算法1 行 7452-7462) ----
    // 姿态步独立重算中点 (行 7440): p_{k-1/2}=1/2(p_{k-1}+p_k), v_{k-1/2}=1/2(v_{k-1}+v_k)
    const Eigen::Vector3d pMidA = 0.5 * (curPos + s.llh); // s.llh 此刻已是新位置 p_k
    const Eigen::Vector3d vMidA = 0.5 * (curVel + vNew); // curVel=v_{k-1}, vNew=v_k
    const double latMA = pMidA[0], hMA = pMidA[2];
    const double vNma = vMidA[0], vEma = vMidA[1];
    const Eigen::Vector3d wEa = earthRateNED(latMA);
    const Eigen::Vector3d wTa = transportRateNED(latMA, vNma, vEma, hMA);
    const Eigen::Vector3d zetaA = (wEa + wTa) * dt; // ζ_k=(ω^n_ie+ω^n_en)Δt (姿态步中点)
    const ATTQuat qnn = ATTQuat::fromRotationVector(-zetaA); // q^{n(k)}_{n(k-1)} (行 7446)
    Eigen::Vector3d phi = e.dtheta; // Δθ_k
    if (hasPrev) phi += (1.0 / 12.0) * prevDtheta.cross(e.dtheta); // 圆锥补偿 (式 6.179/6.180c)
    const ATTQuat qbb = ATTQuat::fromRotationVector(phi); // q^{b(k-1)}_{b(k)} (行 7450)
    s.qbn = (qnn * qOld * qbb).normalized(); // 链乘 (式 6.180a)

    // 历史 (移动窗口)
    prevPrevPos = curPos;
    prevPrevVel = curVel;
    prevDtheta = e.dtheta;
    prevDvel = e.dvel;
    hasPrev = true;
}
