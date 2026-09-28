#include "LEO.h"
#include "Const.h"
#include "CoordStruct.h"
#include <Eigen/Eigen>
#include <fstream>
#include <sstream>
#include <vector>
#include "GnssStruct.h"


bool LEO::loadAuxFile(const std::string &path) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        if (line.find("r_CoG_ARP") != std::string::npos) {
            const size_t pos = line.find('>');
            if (pos == std::string::npos) continue;
            std::istringstream iss(line.substr(pos + 1));
            if (double x = 0, y = 0, z = 0; iss >> x >> y >> z) {
                mLeoAntOffset = Vector3d(x, y, z);
                mHasAux = true;
                return true;
            }
        }
    }
    return false;
}

bool LEO::loadLeoReference(const std::string &path, const SatID &sat) {
    if (!mRefRdr.read(path)) return false;
    mRefSat = sat;
    mHasRef = mRefRdr.contains(sat);
    return mHasRef;
}

PVT LEO::getReferencePVT(const CommonTime &epoch) const {
    if (!mHasRef) return PVT();
    return mRefRdr.getPVT(mRefSat, epoch);
}

Matrix3d LEO::BODY2ECEF(const Vector3d &leoPos, const Vector3d &leoVel) {
    const Vector3d ur = leoPos.normalized(); // 径向(向外)
    Vector3d ut;
    if (leoVel.squaredNorm() > 1.0) ut = leoVel.normalized(); // 沿迹
    else ut = Vector3d(0, 0, 1);
    Vector3d uc = ur.cross(ut); // 轨道法向(角动量方向)
    if (uc.squaredNorm() < 1e-6) uc = Vector3d(1, 0, 0);
    uc.normalize();
    ut = uc.cross(ur).normalized(); // 重新正交化：沿迹 = 法向 × 径向
    Matrix3d R;
    R.col(0) = -ut; // 沿迹 Xb：负号与星载 SRF 约定一致
    R.col(1) = uc;  // 轨道法向 Yb
    R.col(2) = ur;  // 径向 Zb
    return R;
}

Vector3d LEO::recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const {
    if (!mHasAux) return Vector3d::Zero();
    const Matrix3d R = BODY2ECEF(xyzEst, mRecvVel);
    return -R * mLeoAntOffset;
}

void LEO::bootstrapFirstEpoch(ObsData &obsData) {
    if (!mFirstEpoch) return;
    mRefPos = mRefPos.squaredNorm() > 1e12 ? mRefPos : obsData.antennaPosition;
    mFirstEpoch = false;
}

Vector3d LEO::predictPosition([[maybe_unused]] double dt, [[maybe_unused]] const CommonTime &epoch) const {
    const Vector3d fresh = isSaneEcef(mRefPos) ? mRefPos : mCurObs ? mCurObs->antennaPosition : Vector3d::Zero();
    const Vector3d post = mEkfInit ? currentEstimatedPosition() : Vector3d::Zero();
    Vector3d x = (mGapThisEpoch || !isSaneEcef(post)) ? fresh : post;
    if (!isSaneEcef(x)) x = mRefPos;

    if (!mCurObs) {
        mReanchorPos = x;
        return x;
    }

    if (mGapThisEpoch && isSaneEcef(fresh)) mLastGoodAnchor = fresh;
    else if (!isSaneEcef(mLastGoodAnchor)) mLastGoodAnchor = isSaneEcef(fresh) ? fresh : mRefPos;

    const double thr = mConfig.sppResidualThr; // SPP 后验残差 RMS 阈值(m)
    int n = 0;
    double q = 1.0e9;
    Vector3d xf = reanchorSPP(x, &n, &q);
    if (q > thr && isSaneEcef(mLastGoodAnchor)) {
        int na = 0;
        double qa = 1.0e9;
        if (const Vector3d xa = reanchorSPP(mLastGoodAnchor, &na, &qa); na >= 4 && qa < q) {
            xf = xa;
            n = na;
            q = qa;
        }
    }
    if (q > thr && isSaneEcef(fresh)) {
        int nb = 0;
        double qb = 1.0e9;
        if (const Vector3d xb = reanchorSPP(fresh, &nb, &qb); nb >= 4 && qb < q) {
            xf = xb;
            n = nb;
            q = qb;
        }
    }

    mSppGood = n >= 4 && q <= thr;

    if (mHasMotionRef && mSppVsPredThresh < 1.0e8) {
        if (const double ddt = mCurObs->epoch - mMotionRefEpoch; ddt > 0.0 && ddt <= 5.0) {
            if (const Vector3d pred = mMotionRefPos + mMotionRefVel * ddt;
                isSaneEcef(pred) && (xf - pred).norm() > mSppVsPredThresh) {
                mReanchorPos = pred;
                return pred;
            }
        }
    }

    if (mSppGood) {
        mLastGoodAnchor = xf; // 仅质量好时更新可信锚点
        mReanchorPos = xf;
        return xf;
    }
    const Vector3d fb = isSaneEcef(fresh) ? fresh : mLastGoodAnchor;
    mReanchorPos = fb;
    return fb;
}

bool LEO::processEpoch(ObsData &obs) {
    const bool ok = PPPKinematic::processEpoch(obs);
    if (ok && result.xyz.squaredNorm() > 1e12 && !mSppGood) {
        result.xyz = Vector3d::Zero();
        return false;
    }
    if (mSppGood) {
        xyz = mReanchorPos;
        computeElevAzim();
    }
    return ok;
}
