#include "GnssSolver.h"

#include "Products.h"

#include <algorithm>
#include <vector>

#include <Eigen/Cholesky>

#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "Log.h"

GnssSolver::GnssSolver() : prod(std::make_shared<Products>()) {
}


void GnssSolver::estimatePosition() {
    if (mFilterInit) updateFilter();
    else initFilter();
    publishResult();
}


Eigen::Matrix3d GnssSolver::posCovariance() const {
    const auto &id = ekf.posIdx;
    if (id[0] < 0 || id[1] < 0 || id[2] < 0 || ekf.Q.rows() <= id[2])
        return Eigen::Matrix3d::Identity() * 1.0e12;
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            Q(a, b) = ekf.Q(id[a], id[b]);
    return Q;
}


void GnssSolver::solve() {
    result.reset();
    trackEpochGap();
    estimatePosition();
    // 冻结历元不发布速度，也不推进「上一成功历元」；结果清零与几何刷新归 publishResult。
    if (mFilterFrozen) return;
    mLastEpochTime = mCurObs->epoch;
    mHasLastEpoch = true;
    mGapThisEpoch = false;
    estimateVelocity();
}


void GnssSolver::trackEpochGap() {
    if (mHasLastObs) {
        const double obsGap = mDirection * (mCurObs->epoch - mLastObsTime);
        const double solveGap = mHasLastEpoch ? mDirection * (mCurObs->epoch - mLastEpochTime) : 0.0;
        // 门限按标称采样间隔缩放。
        if (const double solveGapLimit = std::max(kAmbSolveGapReset, 1.5 * mNominalInterval);
            (mNominalInterval > 0.0 && obsGap > 1.5 * mNominalInterval) || obsGap > kMaxMotionGapS ||
            (mNominalInterval > 0.0 && solveGap > solveGapLimit)) {
            if (!(mKeepAmbMaxGap > 0.0 && obsGap <= mKeepAmbMaxGap)) mForceAmbReset = true;
        }
    }
    mLastObsTime = mCurObs->epoch;
    mHasLastObs = true;
    if (mHasLastEpoch) {
        if (const double gap = std::fabs(mCurObs->epoch - mLastEpochTime);
            (mNominalInterval > 0.0 && gap > 1.5 * mNominalInterval) || gap > kMaxMotionGapS) {
            mGapThisEpoch = true;
        } else if (mNominalInterval < 0.0 && gap >= 1.0 && gap <= kMaxMotionGapS) mNominalInterval = gap;
    }
}


void GnssSolver::earthRotation(std::map<SatID, PVT> &dst, const Eigen::Vector3d &recv) {
    dst.clear();
    for (auto const &[sat, pvt]: satPVTTransTime) {
        const double ang = OMEGA_EARTH * (pvt.p - recv).norm() / C_MPS;
        const double c = std::cos(ang), s = std::sin(ang);
        Eigen::Matrix3d rot;
        rot <<  c, s, 0,
               -s, c, 0,
                0, 0, 1;
        PVT out = pvt;
        out.p = rot * pvt.p;
        out.v = rot * pvt.v;
        dst[sat] = out;
    }
}


void GnssSolver::computeElevAzim(const std::map<SatID, PVT> &src, const Eigen::Vector3d &recv,
                                 SatValueMap &elev, SatValueMap &azim) const {
    elev.clear();
    azim.clear();
    for (auto const &[sat, pvt]: src) {
        elev[sat] = Coord::elevation(recv, pvt.p, frame);
        azim[sat] = Coord::azimuth(recv, pvt.p, frame);
    }
}


bool GnssSolver::ephUnhealthy(const SatID &sat, const Ephemeris *eph, const bool enabled) {
    if (!enabled || !eph || eph->health == 0u) return false;
    if (mHealthWarned.insert(sat).second)
        LOG_WARN << "[HEALTH] dropped satellite " << sat.toString()
                << " flagged unhealthy by the broadcast ephemeris (health=" << eph->health << ")";
    return true;
}

void GnssSolver::readbackResidual(const EquSys &eq, const Eigen::VectorXd &residual) {
    int iobs = 0;
    for (const auto &[eid, ed]: eq.obsEquData) {
        if (iobs >= residual.size()) break;
        if (ed.isPhase)
            result.postResPhase.set(eid.sat, residual(iobs));
        else
            result.postRes.set(eid.sat, residual(iobs));
        ++iobs;
    }
}


GnssSolver::DopplerResult GnssSolver::estimateVelocity() {
    DopplerResult out;
    // 用 result.xyz 而非 xyz：RTK 固定解只写 result（基线固定解优于浮点解），SPP/PPP 两者同值。
    const Eigen::Vector3d &pos = result.xyz;
    if (!isSaneECEF(pos)) return out;

    const Weight w = dopplerWeight();
    std::vector<Eigen::Vector4d> rows;
    std::vector<double> obsVal, wts;
    rows.reserve(satPVTRecTime.size());
    for (const auto &[sat, pvt]: satPVTRecTime) {
        const auto it = mCurObs->satTypeValueData.find(sat);
        if (it == mCurObs->satTypeValueData.end() || !wantDopplerSat(sat, it->second)) continue;
        
        double doppler = 0.0;
        bool found = false;
        for (const auto &[k, v]: it->second) {
            if (k.size() >= 2 && k[0] == 'D' && k[1] == '1') {
                doppler = v;
                found = true;
                break;
            }
        }
        if (!found) continue;
        if (!pvt.p.allFinite() || !pvt.v.allFinite()) continue;

        const double rho = (pvt.p - pos).norm();
        if (rho < 1.0) continue;
        const double wgt = w.weight(satElevData.at(sat)) * kDopplerWeightScale;
        if (!(wgt > 0.0)) continue;
        const double freqHz = sat.system == SatSystem::R ? getGlonassFreq('1', sat.chan) : getFreq(sat.system, "L1");
        if (freqHz <= 0.0) continue;

        const Eigen::Vector3d u = (pos - pvt.p) / rho;
        Eigen::Vector4d row;
        row << u[0], u[1], u[2], 1.0;
        rows.push_back(row);
        obsVal.push_back(-(C_MPS / freqHz) * doppler + pvt.v.dot(u));
        wts.push_back(wgt);
    }

    out.used = static_cast<int>(rows.size());
    if (out.used < kMinDopplerSats) return out;

    Eigen::Matrix4d N = Eigen::Matrix4d::Zero();
    Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
    for (std::size_t i = 0; i < rows.size(); ++i) {
        N += wts[i] * rows[i] * rows[i].transpose();
        rhs += wts[i] * rows[i] * obsVal[i];
    }
    const Eigen::Vector4d x = N.ldlt().solve(rhs);
    if (!x.allFinite()) return out;
    out.vel = x.head<3>();
    out.clockDrift = x[3];

    // σ0 由后验残差估：正好 4 颗时无冗余，退回 1（此时 sigma 只是形式上的几何精度）。
    double sigma0Sq = 1.0;
    if (out.used > kMinDopplerSats) {
        double vwv = 0.0;
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const double e = obsVal[i] - rows[i].dot(x);
            vwv += wts[i] * e * e;
        }
        sigma0Sq = vwv / static_cast<double>(out.used - kMinDopplerSats);
        if (!std::isfinite(sigma0Sq) || sigma0Sq <= 0.0) sigma0Sq = 1.0;
    }
    const Eigen::Matrix4d Q = N.inverse();
    for (int k = 0; k < 3; ++k)
        out.sigma[k] = std::sqrt(std::max(Q(k, k), 0.0) * sigma0Sq);
    out.ok = true;

    // 少于 4 颗（!ok）：速度留 0，交给导出侧的空值逻辑
    result.vel = out.vel;
    result.sigmaVel = out.sigma;
    result.sigmaV = out.sigma.norm();
    return out;
}
