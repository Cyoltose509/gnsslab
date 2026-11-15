#include "Kinematic.h"
#include "core/Const.h"
#include "Exception.h"
#include "Log.h"
#include "core/geom/CoordStruct.h"
#include "SolverLSQ.h"
#include <Eigen/Eigen>
#include <map>
#include <set>

namespace {
    // 每历元重锚定的 Baarda 最大剔除轮数：BDS GEO 最多 4 颗，全部被剔也到此为止。
    constexpr int kMaxBaardaRounds = 4;
}

Eigen::Vector3d PPPKinematic::reanchorSPP(const Eigen::Vector3d &seedIn, int *nOut, double *rmsOut,
                                          double *pdopOut, char sysOnly) const {
    if (!mCurObs) return seedIn;
    const CommonTime epoch = mCurObs->epoch;
    const std::string &station = mCurObs->station;
    const Variable dx(station, Parameter::dX), dy(station, Parameter::dY), dz(station, Parameter::dZ);
    std::map<char, Variable> cdtVars;
    for (const auto &[sys, param] : mCdtParam) cdtVars.try_emplace(sys, station, param);

    std::set<SatID> rejected;
    Eigen::Vector3d xc = seedIn;
    int lastN = 0;
    double lastRms = 1.0e9;
    double pdop = 1.0e9;

    for (int round = 0; round <= kMaxBaardaRounds; ++round) {
        std::map<char, double> clk;
        for (const auto &[sys, _] : cdtVars) clk[sys] = 0.0;
        SolverLSQ solver;
        EquSys eq;
        for (int it = 0; it < mConfig.maxReanchorIter; ++it) {
            eq.reset();
            eq.station = station;
            eq.varSet.insert(dx); eq.varSet.insert(dy); eq.varSet.insert(dz);
            for (const auto &[sys, v] : cdtVars) eq.varSet.insert(v);

            TropoSite site;
            site.update(prod->tropo, mConfig.tropoMap, xc, epoch);
            const double ztdMean = mFilterInit ? mZtdEstimate : 0.0;
            const Eigen::Vector3d recvTide = mApplySolidTide ? computeTideDisplacement(xc, epoch) : Eigen::Vector3d::Zero();
            int nObs = 0;
            for (const auto &[sat, tv] : mCurObs->satTypeValueData) {
                if (sysOnly && sat.system != sysOnly) continue;
                if (rejected.count(sat)) continue;
                const FreqCombo *defp = freqComboFor(sat);
                if (!defp) continue;
                const FreqCombo &def = *defp;
                if (!def.available(tv, true)) continue;
                const auto itP = satPVTRecTime.find(sat);
                if (itP == satPVTRecTime.end()) continue;
                double elev = 0.0;
                if (const auto itE = satElevData.find(sat); itE != satElevData.end()) elev = itE->second;
                if (elev < mConfig.cutoffElevRad && !mSkipElevCutoff) continue;
                const PVT &pvt = itP->second;
                const Eigen::Vector3d recvEff = xc + recvTide + recvAntennaOffsetECEF(xc, epoch, sat);
                const double rho0 = (pvt.p - recvEff).norm();
                const Eigen::Vector3d los = -(pvt.p - recvEff) / rho0;
                const double dts = pvt.clockBias * C_MPS;
                const double rel = pvt.relativityCorrection * C_MPS;
                const double P_IF = def.combineCodeFromObs(tv) - prod->osb.codeBiasAligned(def, sat, station);
                const double obsMinusClk = P_IF + dts + rel - clk[sat.system];
                double weight = 1.0 / (mConfig.codeSigma * mConfig.codeSigma);
                if (elev < PI / 6) { const double s = std::sin(elev); weight *= s * s; }
                double mW = 0.0;
                double dry = 0.0;
                if (mHasTropo) dry = site.dryDelay(mConfig.tropoMap, elev, &mW);
                else mW = site.mapping(mConfig.tropoMap, elev); // 不估计 ZTD 时只要干映射值 mW
                const double tropoCorr = dry + TropoSite::wetDelay(ztdMean, mW);
                EquData ed;
                ed.prefit = obsMinusClk - tropoCorr - rho0;
                ed.varCoeffData[dx] = los[0]; ed.varCoeffData[dy] = los[1]; ed.varCoeffData[dz] = los[2];
                ed.varCoeffData[cdtVars[sat.system]] = 1.0;
                ed.weight = weight;
                eq.obsEquData[EquID(sat, "IF")] = ed;
                ++nObs;
            }
            if (nObs < 4) break;
            lastN = nObs;
            // 几何强度：(H^T H)^-1 的位置部分。4 星近共面时 PDOP 可达上千，此时 1 m 码噪声
            // 就放大成公里级位置误差，解算结果不能当锚点用。
            {
                Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nObs, 4);
                int row = 0;
                for (const auto &[eid, ed]: eq.obsEquData) {
                    for (const auto &[var, coeff]: ed.varCoeffData) {
                        const auto pt = var.par;
                        if (pt == Parameter::dX) H(row, 0) = coeff;
                        else if (pt == Parameter::dY) H(row, 1) = coeff;
                        else if (pt == Parameter::dZ) H(row, 2) = coeff;
                        else if (pt.isClock()) H(row, 3) = coeff;
                    }
                    ++row;
                }
                if (const Eigen::FullPivLU<Eigen::MatrixXd> lu(H.transpose() * H); lu.isInvertible()) {
                    const Eigen::MatrixXd inv = lu.inverse();
                    pdop = std::sqrt(std::max(inv(0, 0) + inv(1, 1) + inv(2, 2), 0.0));
                }
            }
            try { solver.solve(eq, eq.weightVector().asDiagonal().toDenseMatrix()); }
            catch (...) { break; }
            const Eigen::Vector3d dpos(solver.getSolution(Parameter::dX),
                                solver.getSolution(Parameter::dY),
                                solver.getSolution(Parameter::dZ));
            xc += dpos;
            for (const auto &[sys, v] : cdtVars) clk[sys] += solver.getSolution(v.par);
            if (dpos.norm() < mConfig.convEps) break;
        }
        if (lastN >= 4 && solver.v.size() > 0)
            lastRms = std::sqrt(solver.v.squaredNorm() / solver.v.size());

        // 末轮解的标准化残差逐星检验（与独立 SPP 同式：w = |v| / (σ0·√qvv)）。
        SatID worstSat{};
        double worstW = 0.0;
        if (lastN >= 7 && solver.sigma0 > 0.0 && solver.Q.rows() > 0) {
            const auto &Ninv = solver.Q;
            int i = 0;
            for (const auto &[eid, ed]: eq.obsEquData) {
                if (i >= solver.v.size()) break;
                const double w = ed.weight;
                const double invW = w > 0.0 ? 1.0 / w : 0.0;
                const auto hRow = solver.H.row(i);
                if (const double qvv = invW - (hRow * Ninv * hRow.transpose())(0, 0); qvv > 0.0) {
                    if (const double ws = std::abs(solver.v[i]) / (solver.sigma0 * std::sqrt(qvv)); ws > worstW) {
                        worstW = ws;
                        worstSat = eid.sat;
                    }
                }
                ++i;
            }
        }
        if (worstSat.system == 0 || worstW <= W_THRESHOLD) break;
        rejected.insert(worstSat);
    }

    if (nOut) *nOut = lastN;
    if (rmsOut) *rmsOut = lastRms;
    if (pdopOut) *pdopOut = pdop;
    return xc;
}

Eigen::Vector3d PPPKinematic::predictPosition(const double dt, const CommonTime &epoch) const {
    (void) dt;
    (void) epoch;
    if (!mFilterInit || mReanchorEveryEpoch) {
        const Eigen::Vector3d post = mFilterInit ? currentEstimatedPosition() : Eigen::Vector3d::Zero();
        const Eigen::Vector3d seed = isSaneECEF(post)
                                  ? post
                                  : isSaneECEF(mRefPos)
                                        ? mRefPos
                                        : mCurObs ? mCurObs->antennaPosition : Eigen::Vector3d::Zero();
        int n = 0;
        double rms = 1.0e9;
        Eigen::Vector3d xc = reanchorSPP(seed, &n, &rms);
        bool anchorOk = n >= 4 && rms <= mConfig.sppResidualThr;
        // 混合星座锚定可能被异质钟基准污染：BDS-3 精密钟以 B1I/B3I 组合为基准，而接收机
        // 跟踪的 B1I+B2b 配对没有 OSB/广播 TGD 可改正，逐星带 ±5-12 m 常偏。锚定只负责给
        // 滤波器一个干净的线性化点，GPS 单系统锚定的后验残差更小时就取 GPS 锚。
        if (mCdtParam.size() > 1 && mCdtParam.count('G')) {
            int nG = 0;
            double rmsG = 1.0e9;
            const Eigen::Vector3d xcG = reanchorSPP(seed, &nG, &rmsG, nullptr, 'G');
            if (nG >= 4 && rmsG < rms) {
                xc = xcG;
                rms = rmsG;
                anchorOk = true;
            }
        }
        return anchorOk ? xc : seed;
    }
    return currentEstimatedPosition();
}

bool PPPKinematic::processEpoch(ObsData &obs) {
    try {
        if (!PPPStatic::processEpoch(obs)) return false;
        bool diverged = false;
        if (mReanchorPos.squaredNorm() > 1e12 && result.xyz.squaredNorm() > 1e12) {
            if ((result.xyz - mReanchorPos).norm() > mDivergeJumpThresh) {
                // 重锚定的初值必须取本历元的可信锚点：已经发散的 result.xyz 作初值时
                // SPP 会收敛到另一个错解，再 setBootstrapPosition 就把位置状态永久写坏。
                const Eigen::Vector3d seed = isSaneECEF(mReanchorPos) ? mReanchorPos : result.xyz;
                const Eigen::Vector3d fresh = reanchorSPP(seed);
                result.xyz = fresh;
                result.xyzFloat = fresh; // 浮点解同源，否则 CSV 里两套位置打架
                ekf.setBootstrapPosition(fresh, 10.0 * 10.0); // 位置状态拉回重锚定点(σ=10 m)
                mForceAmbReset = true;
                ++mDivergeRecovers;
                diverged = true;
            }
        }

        // 无解历元（result 被置零）不能再进差分：0 与上一历元位置之差会差出几百万 m/s，
        // 下一历元按它外推就直接飞到几万公里外，整段再也回不来。
        if (result.xyz.squaredNorm() <= 1e12) return false;

        // 速度取相邻历元位置差分；跨中断的差分是弦速度而不是真实速度。
        // LEO 96 s 断档恢复首历元实测位置误差 316 m，直接差分出 316 m/s 的速度误差，
        // 之后每历元外推都带着它，整段按这个比例发散。中断后沿用中断前的速度。
        if (mHasPrev) {
            // dt 保持有符号：倒序滤波时 Δp 与 Δt 同号相反，差分出的仍是真实速度；
            // 物理外推 pred = pos + vel·dt 用同一符号自洽，只有间隔判据取 |dt|。
            const double dtv = obs.epoch - mPrevEpoch;
            if (dtv != 0.0 && (mNominalInterval <= 0.0 || std::fabs(dtv) <= 1.5 * mNominalInterval))
                mRecvVel = (result.xyz - mPrevEstPos) / dtv;
        }
        mPrevEstPos = result.xyz;
        mPrevEpoch = obs.epoch;
        mHasPrev = true;
        if (!diverged && result.numSats >= mConfig.minSatsForUpdate && mRecvVel.squaredNorm() > 1.0e6) {
            mMotionRefPos = result.xyz;
            mMotionRefVel = mRecvVel;
            mMotionRefEpoch = obs.epoch;
            mHasMotionRef = true;
        }
        if (mMotionThresh > 0.0 && mHasMotionRef && !mGapThisEpoch) {
            if (const double dt = obs.epoch - mMotionRefEpoch; mDirection * dt > 0.0 && mDirection * dt <= 5.0) {
                const Eigen::Vector3d pred = mMotionRefPos + mMotionRefVel * dt;
                if (const double thr = mMotionThresh + 0.5 * 9.0 * dt * dt; isSaneECEF(pred) && (result.xyz - pred).norm() > thr) {
                    result.xyz = Eigen::Vector3d::Zero();
                    return false;
                }
            }
        }
        return true;
    } catch (const SVNumException &) {
        return false;   // 本历元不可解（星数不足等），跳过；下一历元继续
    }
}
