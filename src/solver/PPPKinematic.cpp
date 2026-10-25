#include "PPPKinematic.h"
#include "Const.h"
#include "Exception.h"
#include "Log.h"
#include "Troposphere.h"
#include "CoordConvert.h"
#include "CoordStruct.h"
#include "TimeConvert.h"
#include "SolverLSQ.h"
#include <Eigen/Eigen>
#include <map>
#include <set>

Eigen::Vector3d PPPKinematic::reanchorSPP(const Eigen::Vector3d &seedIn, int *nOut, double *rmsOut) const {
    if (!mCurObs) return seedIn;
    const CommonTime epoch = mCurObs->epoch;
    const std::string &station = mCurObs->station;
    const Variable dx(station, Parameter::dX), dy(station, Parameter::dY), dz(station, Parameter::dZ);
    std::map<char, Variable> cdtVars;
    for (const auto &[sys, param] : sysCdtParam) cdtVars.try_emplace(sys, station, param);

    Eigen::Vector3d xc = seedIn;
    std::map<char, double> clk;
    for (const auto &[sys, _] : cdtVars) clk[sys] = 0.0;
    int lastN = 0;
    double lastRms = 1.0e9;

    SolverLSQ solver;
    for (int it = 0; it < mConfig.maxReanchorIter; ++it) {
        EquSys eq;
        eq.station = station;
        eq.varSet.insert(dx); eq.varSet.insert(dy); eq.varSet.insert(dz);
        for (const auto &[sys, v] : cdtVars) eq.varSet.insert(v);

        const auto blhEst = XYZToBLH(xc, Frame::WGS84);
        const double latDeg = blhEst.B() * RAD_TO_DEG;
        const double hgt = blhEst.H();
        const double doy = CommonTime2YDSTime(epoch).doy;
        const double zhd = mHasTropo ? Troposphere::tropoSaastamoinenZHD(blhEst.B(), blhEst.H()) : 0.0;
        const double ztdMean = mEkfInit ? mZtdEstimate : 0.0;
        const Eigen::Vector3d recvTide = mApplySolidTide ? computeTideDisplacement(xc, epoch) : Eigen::Vector3d::Zero();
        const Eigen::Vector3d recvPCOe = recvAntennaOffsetECEF(xc, epoch);
        int nObs = 0;
        for (const auto &[sat, tv] : mCurObs->satTypeValueData) {
            const FreqCombo *defp = freqComboFor(sat);
            if (!defp) continue;
            const FreqCombo &def = *defp;
            if (!def.available(tv, true)) continue;
            const auto itP = satPVTRecTime.find(sat);
            if (itP == satPVTRecTime.end()) continue;
            double elev = 0.0;
            if (const auto itE = satElevData.find(sat); itE != satElevData.end()) elev = itE->second;
            if (elev < mConfig.cutoffElevRad) continue;
            const PVT &pvt = itP->second;
            const Eigen::Vector3d recvEff = xc + recvTide + recvPCOe;
            const double rho0 = (pvt.p - recvEff).norm();
            const Eigen::Vector3d los = -(pvt.p - recvEff) / rho0;
            const double dts = pvt.clockBias * C_MPS;
            const double rel = pvt.relativityCorrection * C_MPS;
            // codeBiasAligned：观测码(C1C/C2W)相对钟差参考码(C1W/C2W)的差，避免与 COD .CLK 的 C1W/C2W datum 双计。
            const double P_IF = def.combineCodeFromObs(tv) - mOsb.codeBiasAligned(def, sat);
            const double obsMinusClk = P_IF + dts + rel - clk[sat.system];
            double weight = 1.0 / (mConfig.sigIFCode * mConfig.sigIFCode);
            if (elev < PI / 6) { const double s = std::sin(elev); weight *= s * s; }
            double mW = 0.0;
            const double mH = Troposphere::tropoNMF(elev, latDeg, hgt, doy, &mW);
            const double tropoCorr = ztdMean * mW + mH * zhd;
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
        try { solver.solve(eq); }
        catch (...) { break; }
        const Eigen::Vector3d dpos(solver.getSolution(Parameter::dX),
                            solver.getSolution(Parameter::dY),
                            solver.getSolution(Parameter::dZ));
        xc += dpos;
        for (const auto &[sys, v] : cdtVars) clk[sys] += solver.getSolution(v.getParaType());
        if (dpos.norm() < mConfig.convEps) break;
    }
    if (lastN >= 4 && solver.v.size() > 0)
        lastRms = std::sqrt(solver.v.squaredNorm() / solver.v.size());
    if (nOut) *nOut = lastN;
    if (rmsOut) *rmsOut = lastRms;
    return xc;
}

Eigen::Vector3d PPPKinematic::predictPosition(const double dt, const CommonTime &epoch) const {
    (void) dt;
    (void) epoch;
    // 运动学恒为重锚定：虚分派已决定走此实现，mReanchor 运行期开关已删除，无需再判（与 LEO 一致）
    if (!mEkfInit || mReanchorEveryEpoch) {
        const Eigen::Vector3d post = mEkfInit ? currentEstimatedPosition() : Eigen::Vector3d::Zero();
        const Eigen::Vector3d seed = isSaneECEF(post)
                                  ? post
                                  : (isSaneECEF(mRefPos)
                                         ? mRefPos
                                         : (mCurObs ? mCurObs->antennaPosition : Eigen::Vector3d::Zero()));
        int n = 0;
        double rms = 1.0e9;
        const Eigen::Vector3d xc = reanchorSPP(seed, &n, &rms);
        // 重锚定质量太差则退回 seed
        return n < 4 || rms > mConfig.sppResidualThr ? seed : xc;
    }
    return currentEstimatedPosition();
}

bool PPPKinematic::processEpoch(ObsData &obs) {
    try {
    const bool ok = PPPStatic::processEpoch(obs);
    if (!ok) return false;
    // 注：曾在此加「间隙后把 EKF 位置状态拉回 SPP 锚点」的 #008 修复，经核对为无效改动：
    // PPPStatic::processEpoch 第 831 行每历元都执行 ekf.setBootstrapPosition(xyzEst, 0.0)，
    // 位置先验均值本就被强制拉到本历元 SPP 线性化点，不存在"状态沿用间隙前漂移值"。
    // 间隙后 10–18m 瞬态的真因是 SPP 锚点自身精度（码观测+卫星少）+ 模糊度重置，
    // 只能靠 #309 RTS 反向平滑或更好的间隙预测解决。
    // 运动学恒为重锚定（虚分派已决定），mReanchor 运行期开关已删除，此块为 kinematic 专属逻辑
    bool diverged = false;
        if (mReanchorPos.squaredNorm() > 1e12 && result.xyz.squaredNorm() > 1e12) {
            if ((result.xyz - mReanchorPos).norm() > mDivergeJumpThresh) {
                const Eigen::Vector3d fresh = reanchorSPP(result.xyz);
                result.xyz = fresh;
                ekf.setBootstrapPosition(fresh, 100.0); // 位置状态拉回重锚定点(P=100 m^2)
                mForceAmbReset = true;
                ++mDivergeRecovers;
                diverged = true;
            }
        }
        if (mHasPrev && obs.epoch > mPrevEpoch)
            mRecvVel = (result.xyz - mPrevEstPos) / (obs.epoch - mPrevEpoch);
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
            if (const double dt = obs.epoch - mMotionRefEpoch; dt > 0.0 && dt <= 5.0) {
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
