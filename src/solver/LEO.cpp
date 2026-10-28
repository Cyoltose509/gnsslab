#include "LEO.h"
#include "Const.h"
#include "CoordStruct.h"
#include "CoordConvert.h"
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
                mLeoAntOffset = Eigen::Vector3d(x, y, z);
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

Eigen::Vector3d LEO::recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const {
    if (!mHasAux) return Eigen::Vector3d::Zero();
    const Eigen::Matrix3d M = ECEFToLVLHDCM(xyzEst, mRecvVel);
    return -M.transpose() * mLeoAntOffset;
}

void LEO::bootstrapFirstEpoch(ObsData &obsData) {
    if (!mFirstEpoch) return;
    mRefPos = mRefPos.squaredNorm() > 1e12 ? mRefPos : obsData.antennaPosition;
    mFirstEpoch = false;
}

Eigen::Vector3d LEO::predictPosition([[maybe_unused]] double dt, [[maybe_unused]] const CommonTime &epoch) const {
    const Eigen::Vector3d fresh = isSaneECEF(mRefPos) ? mRefPos : mCurObs ? mCurObs->antennaPosition : Eigen::Vector3d::Zero();
    const Eigen::Vector3d post = mEkfInit ? currentEstimatedPosition() : Eigen::Vector3d::Zero();
    Eigen::Vector3d x = mGapThisEpoch || !isSaneECEF(post) ? fresh : post;
    if (!isSaneECEF(x)) x = mRefPos;

    if (!mCurObs) {
        mReanchorPos = x;
        return x;
    }

    if (mGapThisEpoch && isSaneECEF(fresh)) mLastGoodAnchor = fresh;
    else if (!isSaneECEF(mLastGoodAnchor)) mLastGoodAnchor = isSaneECEF(fresh) ? fresh : mRefPos;

    const double thr = mConfig.sppResidualThr; // SPP 后验残差 RMS 阈值(m)
    int n = 0;
    double q = 1.0e9;
    Eigen::Vector3d xf = reanchorSPP(x, &n, &q);
    if (q > thr && isSaneECEF(mLastGoodAnchor)) {
        int na = 0;
        double qa = 1.0e9;
        if (const Eigen::Vector3d xa = reanchorSPP(mLastGoodAnchor, &na, &qa); na >= 4 && qa < q) {
            xf = xa;
            n = na;
            q = qa;
        }
    }
    if (q > thr && isSaneECEF(fresh)) {
        int nb = 0;
        double qb = 1.0e9;
        if (const Eigen::Vector3d xb = reanchorSPP(fresh, &nb, &qb); nb >= 4 && qb < q) {
            xf = xb;
            n = nb;
            q = qb;
        }
    }

    mSppGood = n >= 4 && q <= thr;

    if (mHasMotionRef && mSppVsPredThresh < 1.0e8) {
        if (const double ddt = mCurObs->epoch - mMotionRefEpoch; ddt > 0.0 && ddt <= 5.0) {
            if (const Eigen::Vector3d pred = mMotionRefPos + mMotionRefVel * ddt;
                isSaneECEF(pred) && (xf - pred).norm() > mSppVsPredThresh) {
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
    const Eigen::Vector3d fb = isSaneECEF(fresh) ? fresh : mLastGoodAnchor;
    mReanchorPos = fb;
    return fb;
}

bool LEO::processEpoch(ObsData &obs) {
    const bool ok = PPPKinematic::processEpoch(obs);
    if (ok && result.xyz.squaredNorm() > 1e12 && !mSppGood) {
        result.xyz = Eigen::Vector3d::Zero();
        return false;
    }
    if (mSppGood) {
        xyz = mReanchorPos;
        computeElevAzim();
    }
    return ok;
}

void LEO::applyOptions(const SolverOptions &o) {
    // 仅调 SPP 基类注入 + leo_cli 实际设置的子集。
    // 切勿调 PPPStatic::applyOptions：否则会覆盖 LEO ctor 的专有值
    // （setTropo(false) / mApplySolidTide=false / mUseCycleSlip=false /
    //  mConfig.sigmaPhase=0.003 / ambNoise=1e-7 / firstEpochPosVar=1.0 / minSatsForUpdate=4 等）。
    SPP::applyOptions(o);                 // 截止角 / 系统 / ISB（与 leo_cli 一致）
    mApplySolidTide = o.solidTide != 0;
    setMeasNoise(o.sigIFCode, o.sigPhase);
    setPosProcNoise(o.posNoise);
    setKeepAmbMaxGap(o.keepAmbMaxGap);
    mMotionThresh = o.motionThresh;
    mSppVsPredThresh = o.sppVsPredThresh;
    // 注：setIsbProcNoise 已在 SPP::applyOptions 内设置；
    //     LEO ctor 的 setTropo(false) / mConfig.sigmaPhase=0.003 等保持，与 leo_cli 行为一致。
}
