#include "Static.h"
#include "core/equ/Precision.h"
#include "core/geom/CoordConvert.h"
#include <cmath>

void PPPStatic::readback(const ObsData &obsData) {
    try { result.xyz = currentEstimatedPosition(); } catch (...) {}
    result.xyzFloat = result.xyz;
    result.blh = Coord::XYZToBLH(result.xyz, Frame::WGS84);
    result.numSats = static_cast<int>(mEq.obsEquData.size()) / 2;

    for (const auto &[sys, par]: mCdtParam) {
        GnssEKF::tryGetSolution(par, ekf.currentUnkSet, ekf.solution, mClkEstimate[sys]);
    }
    for (char sys: mSysSeen) {
        if (!osbCoversSystem(sys)) {
            const Variable vcb = makeCodeBiasVar(obsData.station, SatID(sys, -1), "IF");
            mCbEstimate[sys] = ekf.getSolution(vcb);
        }
    }
    // 逐 GLONASS/BDS 星 IFCB 回读
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (sat.system != 'R' && sat.system != 'C') continue;
        if (osbCovers(sat)) continue;
        if (const FreqCombo *defp = freqComboFor(sat); !defp || !defp->available(tv, true)) continue;
        const Variable vifb = makeIfbVar(obsData.station, sat, "IF");
        mIfcbEstimate[sat] = ekf.getSolution(vifb);
    }
    GnssEKF::tryGetSolution(Parameter::ztd, ekf.currentUnkSet, ekf.solution, mZtdEstimate);
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        if (!defp->available(tv, true)) continue;
        const Variable vamb = makeAmbiguityVar(obsData.station, sat, "IF");
        mAmbEstimate[sat] = ekf.getSolution(vamb); // 按 Variable 取：不在集中直接返回 0，不抛
    }

    if (const Eigen::VectorXd postfit = ekf.postfitResidual; postfit.size() > 0) {
        int iobs = 0;
        for (const auto &[eid, ed]: mEq.obsEquData) {
            if (iobs < postfit.size() && !eid.obsType.empty()) {
                if (eid.obsType[0] != 'L')
                    result.postRes.set(eid.sat, postfit(iobs)); // 码(伪距)残差
                else
                    result.postResPhase.set(eid.sat, postfit(iobs)); // 相位残差
            }
            ++iobs;
        }
    }

    geometryDOPs(result, mEq, result.blh);

    const Variable vdx(obsData.station, Parameter::dX);
    const Variable vdy(obsData.station, Parameter::dY);
    const Variable vdz(obsData.station, Parameter::dZ);
    const Variable vox(obsData.station, Parameter::dOX);
    const Variable voy(obsData.station, Parameter::dOY);
    const Variable voz(obsData.station, Parameter::dOZ);
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
    sigma0 = posteriorSigma0(ekf.postfitResidual, mEq.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()));
    result.sigmaP = std::max(positionSigma(Q, sigma0), 1e-3);
    result.sigmaPFloat = result.sigmaP;
}

Eigen::Matrix3d PPPStatic::posCovariance() const {
    if (!mCurObs || !mFilterInit || ekf.Q.size() == 0)
        return Eigen::Matrix3d::Identity() * 1.0e12;
    const std::string &st = mCurObs->station;
    const auto idx = [this](const Variable &v) {
        const auto it = ekf.currentIndexData.find(v);
        return it == ekf.currentIndexData.end() ? -1 : it->second;
    };
    const int id[3] = {
        idx(Variable(st, Parameter::dX)),
        idx(Variable(st, Parameter::dY)),
        idx(Variable(st, Parameter::dZ))
    };
    return positionCovBlock(ekf.Q, id);
}
