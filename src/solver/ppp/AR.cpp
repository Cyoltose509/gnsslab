#include "Static.h"
#include "ARLambda.h"
#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include <cmath>

bool PPPStatic::tryFixAmbiguities() {
    mArRatio = 0.0;
    mArNumFixed = 0;
    result.xyzFixed = Eigen::Vector3d::Zero();
    ++mArAttempts;
    if (!mFilterInit) {
        return false;
    }

    const Eigen::VectorXd x = ekf.solution;
    const Eigen::MatrixXd P = ekf.Q;

    std::vector<SatID> sats;
    std::vector<int> idx;
    std::vector<double> Bf;
    int inState = 0;
    double sMin = 1e9, sMax = -1.0, sSum = 0.0;
    for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        if (!defp->available(tv, true)) continue;
        if (satPVTRecTime.count(sat) == 0) continue;
        const Variable vamb = Variable::makeAmbiguity(mCurObs->station, sat, "IF");
        if (!ekf.currentIndexData.count(vamb)) continue;
        const int i = ekf.currentIndexData.at(vamb);
        const double sig = std::sqrt(P(i, i));
        ++inState;
        if (sig < sMin) sMin = sig;
        if (sig > sMax) sMax = sig;
        sSum += sig;
        if (sig > mConfig.arGateSigma) continue;
        sats.push_back(sat);
        idx.push_back(i);
        Bf.push_back(x(i));
    }
    const int n = static_cast<int>(sats.size());
    if (n < 5) return false;

    Eigen::VectorXd a(2 * n);
    Eigen::MatrixXd Q(2 * n, 2 * n);
    Q.setZero();
    std::vector<double> lam_n(n), cFac(n); // cFac = f2/(f1−f2)
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const TypeValueMap &tv = mCurObs->satTypeValueData.at(sat);
        const FreqCombo &def = *freqComboFor(sat);
        const double f1 = def.f1, f2 = def.f2;
        lam_n[k] = def.lambdaN; // 窄巷波长(m)
        cFac[k] = f2 / (f1 - f2);
        const double lam1 = def.lambda1, lam2 = def.lambda2;
        double osbP1 = 0, osbP2 = 0, osbL1 = 0, osbL2 = 0;
        prod->osb.biases(def, sat, osbP1, osbP2, osbL1, osbL2, mCurObs->station);
        const double L1m = tv.at(def.phase1), L2m = tv.at(def.phase2);
        const double C1m = tv.at(def.code1), C2m = tv.at(def.code2);
        const double NwInst = def.MW_cycle(C1m, C2m, L1m, L2m, osbP1, osbP2, osbL1, osbL2);
        const double a1 = 1.0 / lam1, a2 = -1.0 / lam2;
        const double r = f1 / f2 * (f1 / f2);
        const double K = -(a1 + a2);
        const double Gc = a1 + r * a2;
        const double b2 = (Gc - K) / (r - 1.0);
        const double b1 = K - b2;
        const double sL = mConfig.phaseSigma, sP = mConfig.codeSigma;
        const double varNwInst = a1 * a1 * sL * sL + a2 * a2 * sL * sL + b1 * b1 * sP * sP + b2 * b2 * sP * sP;
        if (std::abs(Bf[k]) < 0.5) {
            mNwSmooth.erase(sat);
            mNwVar.erase(sat);
        }
        double NwVal, varNwVal;
        if (auto it = mNwSmooth.find(sat); it == mNwSmooth.end() || mNwVar.find(sat) == mNwVar.end()) {
            NwVal = NwInst;
            varNwVal = varNwInst;
            mNwSmooth[sat] = NwInst;
            mNwVar[sat] = varNwInst;
        } else {
            const double v0 = mNwVar[sat];
            const double Kg = v0 / (v0 + varNwInst);
            mNwSmooth[sat] += Kg * (NwInst - mNwSmooth[sat]);
            mNwVar[sat] = (1.0 - Kg) * v0;
            NwVal = mNwSmooth[sat];
            varNwVal = mNwVar[sat];
        }
        const double N1 = Bf[k] / lam_n[k] - cFac[k] * NwVal;
        a(2 * k) = NwVal;
        a(2 * k + 1) = N1;
        Q(2 * k, 2 * k) = varNwVal;
        Q(2 * k + 1, 2 * k + 1) = P(idx[k], idx[k]) / (lam_n[k] * lam_n[k]);
        for (int j = 0; j < n; ++j)
            if (j != k)
                Q(2 * k + 1, 2 * j + 1) = P(idx[k], idx[j]) / (lam_n[k] * lam_n[j]);
    }

    const auto [afix, ratio, fixed] = resolveWithRatio(a, Q, mConfig.arRatioThreshold);
    mArRatio = ratio;
    result.ratio = ratio;
    if (!fixed) return false;

    Eigen::VectorXd z(n);
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const FreqCombo &def = *freqComboFor(sat);
        const double f1 = def.f1, f2 = def.f2;
        const double N1f = afix(2 * k + 1), Nwf = afix(2 * k);
        const double Bfix = C_MPS * ((f1 - f2) * N1f + f2 * Nwf) / (f1 * f1 - f2 * f2);
        z(k) = Bfix;
    }

    const int Nst = static_cast<int>(x.size());
    Eigen::MatrixXd H(n, Nst);
    H.setZero();
    for (int k = 0; k < n; ++k) H(k, idx[k]) = 1.0;
    constexpr double varFix = 1.0e-10;
    const Eigen::MatrixXd Dv = varFix * Eigen::MatrixXd::Identity(n, n);
    const Eigen::MatrixXd S = H * P * H.transpose() + Dv;
    const Eigen::MatrixXd Km = P * H.transpose() * S.inverse();
    const Eigen::VectorXd xf = x + Km * (z - H * x);

    Eigen::Vector3d xyzFixed(
        xf(ekf.currentIndexData.at(Variable(mCurObs->station, Parameter::dX))),
        xf(ekf.currentIndexData.at(Variable(mCurObs->station, Parameter::dY))),
        xf(ekf.currentIndexData.at(Variable(mCurObs->station, Parameter::dZ))));
    result.xyzFixed = xyzFixed;
    result.blhFixed = Coord::XYZToBLH(xyzFixed, Frame::WGS84);
    {
        const int ix = ekf.currentIndexData.at(Variable(mCurObs->station, Parameter::dX));
        const int iy = ekf.currentIndexData.at(Variable(mCurObs->station, Parameter::dY));
        const int iz = ekf.currentIndexData.at(Variable(mCurObs->station, Parameter::dZ));
        const Eigen::MatrixXd Pf = (Eigen::MatrixXd::Identity(Nst, Nst) - Km * H) * P;
        // 固定解只取对角（与 RTK 的 Qfix 同口径），三分量与 sigmaP 共乘一个 sigma0
        result.sigmaXYZ = Eigen::Vector3d(std::sqrt(std::max(Pf(ix, ix), 0.0)),
                                          std::sqrt(std::max(Pf(iy, iy), 0.0)),
                                          std::sqrt(std::max(Pf(iz, iz), 0.0))) * sigma0;
        result.sigmaPFixed = result.sigmaXYZ.norm();
    }
    mArNumFixed = n;
    ++mArSuccess;
    return true;
}
