#include "PPPKinematic.h"
#include "Const.h"
#include "Log.h"
#include "Troposphere.h"
#include "CoordConvert.h"
#include "CoordStruct.h"
#include "TimeConvert.h"
#include "SolverLSQ.h"
#include <Eigen/Eigen>
#include <map>
#include <set>

Vector3d PPPKinematic::reanchorSPP(const Vector3d &seedIn, int *nOut, double *rmsOut) const {
    if (!mCurObs) return seedIn;
    const CommonTime epoch = mCurObs->epoch;
    const std::string &station = mCurObs->station;
    const Variable dx(station, Parameter::dX), dy(station, Parameter::dY), dz(station, Parameter::dZ);
    std::map<char, Variable> cdtVars;
    for (const auto &[sys, param] : sysCdtParam) cdtVars.try_emplace(sys, station, param);

    Vector3d xc = seedIn;
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

        const auto blhEst = XYZtoBLH(xc, Frame::WGS84);
        const double latDeg = blhEst.B() * 180.0 / PI;
        const double hgt = blhEst.H();
        const double doy = CommonTime2YDSTime(epoch).doy;
        const double zhd = mHasTropo ? Troposphere::tropoSaastamoinenZHD(blhEst.B(), blhEst.H()) : 0.0;
        const double ztdMean = mEkfInit ? mZtdEstimate : 0.0;
        const Vector3d recvTide = mApplySolidTide ? computeTideDisplacement(xc, epoch) : Vector3d::Zero();
        const Vector3d recvPCOe = recvAntennaOffsetECEF(xc, epoch);
        int nObs = 0;
        for (const auto &[sat, tv] : mCurObs->satTypeValueData) {
            if (!ifCodeTypes.count(sat.system)) continue;
            const FreqCombo &def = ifCodeTypes.at(sat.system);
            if (!def.available(tv, true)) continue;
            const auto itP = satPVTRecTime.find(sat);
            if (itP == satPVTRecTime.end()) continue;
            double elev = 0.0;
            if (const auto itE = satElevData.find(sat); itE != satElevData.end()) elev = itE->second;
            if (elev < mConfig.cutoffElevRad) continue;
            const PVT &pvt = itP->second;
            const Vector3d recvEff = xc + recvTide + recvPCOe;
            const double rho0 = (pvt.p - recvEff).norm();
            const Vector3d los = -(pvt.p - recvEff) / rho0;
            const double dts = pvt.clockBias * C_MPS;
            const double rel = pvt.relativityCorrection * C_MPS;
            const double P_IF = def.combineCodeFromObs(tv) - mOsb.codeBias(def, sat);
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
        const Vector3d dpos(solver.getSolution(Parameter::dX),
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

Vector3d PPPKinematic::predictPosition(const double dt, const CommonTime &epoch) const {
    (void) dt;
    (void) epoch;
    if (mReanchor) {
        if (!mEkfInit || mReanchorEveryEpoch) {
            const Vector3d post = mEkfInit ? currentEstimatedPosition() : Vector3d::Zero();
            const Vector3d seed = isSaneEcef(post)
                                      ? post
                                      : (isSaneEcef(mRefPos)
                                             ? mRefPos
                                             : (mCurObs ? mCurObs->antennaPosition : Vector3d::Zero()));
            int n = 0;
            double rms = 1.0e9;
            const Vector3d xc = reanchorSPP(seed, &n, &rms);
            // 重锚定质量太差
            return n < 4 || rms > mConfig.sppResidualThr ? seed : xc;
        }
        return currentEstimatedPosition();
    }
    return currentEstimatedPosition();
}

bool PPPKinematic::processEpoch(ObsData &obs) {
    const bool ok = PPPStatic::processEpoch(obs);
    if (!ok) return false;
    if (mReanchor) {
        bool diverged = false;
        if (mReanchorPos.squaredNorm() > 1e12 && result.xyz.squaredNorm() > 1e12) {
            if ((result.xyz - mReanchorPos).norm() > mDivergeJumpThresh) {
                const Vector3d fresh = reanchorSPP(result.xyz);
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
                const Vector3d pred = mMotionRefPos + mMotionRefVel * dt;
                if (const double thr = mMotionThresh + 0.5 * 9.0 * dt * dt; isSaneEcef(pred) && (result.xyz - pred).norm() > thr) {
                    result.xyz = Vector3d::Zero();
                    return false;
                }
            }
        }
    }
    return true;
}
