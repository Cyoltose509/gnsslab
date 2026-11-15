#include "Static.h"
#include "core/equ/Precision.h"
#include "core/geom/CoordConvert.h"
#include <cmath>

void PPPStatic::readback() {
    try { result.xyz = currentEstimatedPosition(); } catch (...) {}
    result.xyzFloat = result.xyz;
    result.blh = Coord::XYZToBLH(result.xyz, Frame::WGS84);
    result.numSats = static_cast<int>(curEq.obsEquData.size()) / 2;

    for (const auto &[sys, par]: mCdtParam) {
        GnssEKF::tryGetSolution(par, ekf.currentUnkSet, ekf.solution, mClkEstimate[sys]);
    }
    for (char sys: mSysSeen) {
        if (!osbCoversSystem(sys)) {
            const Variable vcb = makeCodeBiasVar(mCurObs->station, SatID(sys, -1), "IF");
            mCbEstimate[sys] = ekf.getSolution(vcb);
        }
    }
    // 逐 GLONASS/BDS 星 IFCB 回读
    for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
        if (sat.system != 'R' && sat.system != 'C') continue;
        if (osbCovers(sat)) continue;
        if (const FreqCombo *defp = freqComboFor(sat); !defp || !defp->available(tv, true)) continue;
        const Variable vifb = makeIfbVar(mCurObs->station, sat, "IF");
        mIfcbEstimate[sat] = ekf.getSolution(vifb);
    }
    GnssEKF::tryGetSolution(Parameter::ztd, ekf.currentUnkSet, ekf.solution, mZtdEstimate);
    for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        if (!defp->available(tv, true)) continue;
        const Variable vamb = makeAmbiguityVar(mCurObs->station, sat, "IF");
        mAmbEstimate[sat] = ekf.getSolution(vamb); // 按 Variable 取：不在集中直接返回 0，不抛
    }

    if (const Eigen::VectorXd postfit = ekf.postfitResidual; postfit.size() > 0) {
        int iobs = 0;
        for (const auto &[eid, ed]: curEq.obsEquData) {
            if (iobs < postfit.size() && !eid.obsType.empty()) {
                if (eid.obsType[0] != 'L')
                    result.postRes.set(eid.sat, postfit(iobs)); // 码(伪距)残差
                else
                    result.postResPhase.set(eid.sat, postfit(iobs)); // 相位残差
            }
            ++iobs;
        }
    }

    geometryDOPs(result, curEq, result.blh);

    const Variable vdx(mCurObs->station, Parameter::dX);
    const Variable vdy(mCurObs->station, Parameter::dY);
    const Variable vdz(mCurObs->station, Parameter::dZ);
    const Variable vox(mCurObs->station, Parameter::dOX);
    const Variable voy(mCurObs->station, Parameter::dOY);
    const Variable voz(mCurObs->station, Parameter::dOZ);
    const Eigen::MatrixXd P = ekf.Q;
    auto axisVar = [&](const Variable &pos, const Variable &off) -> double {
        if (!ekf.currentIndexData.count(pos)) return 0.0;
        const int ip = ekf.currentIndexData.at(pos);
        double v = P(ip, ip);
        if (ekf.currentIndexData.count(off)) {
            const int io = ekf.currentIndexData.at(off);
            v += P(io, io) + 2.0 * P(ip, io);
        }
        return v;
    };
    const double sx = std::sqrt(std::max(axisVar(vdx, vox), 0.0));
    const double sy = std::sqrt(std::max(axisVar(vdy, voy), 0.0));
    const double sz = std::sqrt(std::max(axisVar(vdz, voz), 0.0));
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(0, 0) = sx * sx;
    Q(1, 1) = sy * sy;
    Q(2, 2) = sz * sz;
    sigma0 = posteriorSigma0(ekf.postfitResidual, curEq.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()));
    result.sigmaP = std::max(positionSigma(Q, sigma0), 1e-3);
    result.sigmaPFloat = result.sigmaP;
}

