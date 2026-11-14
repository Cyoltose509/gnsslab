#include "LEO.h"
#include "core/geom/CoordStruct.h"
#include "core/geom/Attitude.h"
#include "core/geom/CoordConvert.h"
#include "Log.h"
#include <Eigen/Eigen>
#include <iomanip>
#include <vector>
#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/SatID.h"

namespace {
    /// 4 星近共面时 PDOP 可上千，1 m 码噪声被放大成公里级，这样的 SPP 位置不能作线性化点
    constexpr double kPdopLimit = 50.0;
    /// PDOP 到这个量级位置在法向完全不可观：码解实测带 70–140 m 误差，本历元不给解
    constexpr double kPdopReject = 100.0;
    /// SPP 不可信时位置只剩相位撑着，偏离本历元锚点这么多就认为滤波已散
    constexpr double kSppFailJumpM = 300.0;
    /// 判定解算结果可信的最长外推跨度(s)：低轨断档一分钟以上很常见
    constexpr double kMotionTrustSpanS = 300.0;
    /// 加速度包络(m/s^2)：真实运动不可能超过它，用来给外推邻域定上界
    constexpr double kAccelEnvelope = 9.0;
    /// 相邻历元速度差分的加速度上限(m/s^2)：超过这个量级只可能是解算跳变，不是真实运动
    constexpr double kAccelLimit = 15.0;
    /// 位置状态离开可信锚点运动外推超过这么多，就认定状态被钟跳带走了(m)
    constexpr double kPosStateJumpM = 200.0;
    /// 断档恢复首历元的发布门限：PDOP × 码 σ 超过它就不发布该历元位置(m)
    constexpr double kSppPosGateM = 10.0;
    /// 发布位置至少要 5 颗星：4 星恰好解 XYZ+钟，残差恒为 0 没有冗余；
    /// 再少就是纯状态外推，位置和多普勒速度会一起沿轨道漂，实测能累积到几十公里。
    constexpr int kMinPublishSats = 5;
}


void LEO::adoptProducts(SPP &src) {
    auto *s = dynamic_cast<LEO *>(&src);
    if (!s || s == this) return;
    PPP::Core::adoptProducts(src);
    mAnt = s->mAnt;
    mAntSrc = s->mAntSrc;
}

void LEO::setAntennaOffset(const AntennaOffset &offset, const std::string &source) {
    mAnt = offset;
    mAntSrc = source;
    LOG_INFO << "[LEO] 天线改正 质心→ARP = " << mAnt.comToArp.transpose() << " m"
             << (mAnt.hasArp ? "" : "（未定，按 0）")
             << "，ARP→相位中心 = " << mAnt.pco.transpose() << " m"
             << (mAnt.hasPco ? "" : "（未定，按 0）")
             << "，合计 " << mAnt.total().transpose() << " m ← " << mAntSrc;
}

Eigen::Vector3d LEO::recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch,
                                            const SatID &sat) const {
    (void) epoch;
    (void) sat;
    if (!mAnt.any()) return Eigen::Vector3d::Zero();
    // 第一历元还没有速度估计，LVLH 的沿迹轴无从确定（退化兜底会拿 ECEF 的 Z 轴顶上），
    // 于是 0.3 m 的沿迹分量会被投到一个无关方向。宁可这一历元不做改正，也不要错方向。
    if (mRecvVel.squaredNorm() < 1.0e6) return Eigen::Vector3d::Zero();
    const DCM M = DCM::fromECEFToLVLH(xyzEst, mRecvVel);
    return M.transpose() * mAnt.total();
}

void LEO::bootstrapFirstEpoch(ObsData &obsData) {
    if (!mFirstEpoch) return;
    // 初值只许来自观测文件头或解算自身。参考轨道是真值，只用于 commitEpoch() 事后对比，
    // 拿它当起算点会让精度评估自证其说。星载 RINEX 缺 APPROX POSITION XYZ 时这里留 0，
    // 由 Core::solve() 用本历元码伪距粗定位补上；赤道海平面点兜底已撤销，不凭空造初值。
    if (!isSaneECEF(mRefPos)) mRefPos = obsData.antennaPosition;
    xyz = mRefPos;
    mFirstEpoch = false;
}

Eigen::Vector3d LEO::predictPosition(const double dt, const CommonTime &epoch) const {
    (void) epoch;
    // 线性化点用上一历元的位置、速度、加速度外推
    Eigen::Vector3d x = mFilterInit ? currentEstimatedPosition() : Eigen::Vector3d::Zero();
    mGeomWeak = false;
    const bool canExtrap = mRecvVel.squaredNorm() > 1.0e6 && std::fabs(dt) > 0.0 && std::fabs(dt) <= kMotionTrustSpanS;
    if (isSaneECEF(x) && canExtrap) x += mRecvVel * dt + 0.5 * mRecvAcc * dt * dt;
    if (!isSaneECEF(x)) x = mLastGoodAnchor;
    if (!isSaneECEF(x)) x = isSaneECEF(mRefPos) ? mRefPos : mCurObs ? mCurObs->antennaPosition : Eigen::Vector3d::Zero();

    if (!mCurObs) {
        mReanchorPos = x;
        return x;
    }

    // 可信锚点只由质量达标的 SPP 解更新；中断首历元更不能拿文件头坐标覆盖它。
    if (!isSaneECEF(mLastGoodAnchor)) mLastGoodAnchor = x;

    const double thr = mConfig.sppResidualThr; // SPP 后验残差 RMS 阈值(m)
    int n = 0;
    double q = 1.0e9;
    double pd = 1.0e9;
    Eigen::Vector3d xf = reanchorSPP(x, &n, &q, &pd);
    if (q > thr && isSaneECEF(mLastGoodAnchor)) {
        int na = 0;
        double qa = 1.0e9;
        if (const Eigen::Vector3d xa = reanchorSPP(mLastGoodAnchor, &na, &qa); na >= 4 && qa < q) {
            xf = xa;
            n = na;
            q = qa;
        }
    }

    mSppGood = n >= 4 && q <= thr && pd <= kPdopLimit;
    mGeomWeak = pd > kPdopLimit;
    mEpochPdop = pd;

    // 几何退化时 SPP 位置带着公里级误差，不把它当锚点，改用已有解作线性化点：
    // 相位观测噪声只有毫米级，放大后仍在十米量级，靠滤波器自己撑住比用错锚点强得多。
    if (!mSppGood && pd > kPdopLimit) {
        mReanchorPos = x;
        return x;
    }

    // SPP 与运动预测矛盾时以运动预测为准，但只限紧邻上一段可信解的情形（1–2 s 内）。
    // 断档几十秒后外推本身就带几百米误差，那不是观测的问题，不能拿它否决 SPP。
    if (mHasMotionRef && mSppVsPredThresh < 1.0e8) {
        if (const double g = mCurObs->epoch - mMotionRefEpoch; mDirection * g > 0.0 && mDirection * g <= 2.0) {
            if (const Eigen::Vector3d pred = mMotionRefPos + mMotionRefVel * g;
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
    const Eigen::Vector3d fb = isSaneECEF(x) ? x : mLastGoodAnchor;
    mReanchorPos = fb;
    return fb;
}
void LEO::buildEquSys(ObsData &obsData, const Eigen::Vector3d &xyzEst, VariableDataMap &csData, bool firstIter) {
    PPP::Core::buildEquSys(obsData, xyzEst, csData, firstIter);
    if (!mGeomWeak || mEpochPdop <= kPdopReject) return;
    // 4 颗星近共面（实测 PDOP 7.5e6）时位置在法向完全不可观：码观测会把位置状态直接
    // 拉到 3e8 m，之后整段解不回来；降到 PDOP 200 时码解也已带 70–140 m 误差。
    // 这类历元不给解比给错解好。
    mEq.obsEquData.clear();
}

bool LEO::processEpoch(ObsData &obs) {
    const bool prevSolved = mPrevEpochSolved;
    mPrevEpochSolved = false;
    const bool ok = PPP::Kinematic::processEpoch(obs);
    const bool hasSol = ok && result.xyz.squaredNorm() > 1e12;
    // 轨道高度物理包络 + 最少 5 星：跑到地球内部/上万公里外，或星数不足只剩状态外推，
    // 都会输出「标了 Float 却带假位置」的行，一律不发布。
    if (hasSol) {
        const double r = result.xyz.norm();
        if (r < 6.5e6 || r > 1.0e7 || result.numSats < kMinPublishSats) {
            result.xyz = Eigen::Vector3d::Zero();
            result.numSats = 0;
            return false;
        }
    }
    // 断档恢复首历元的位置只能拿码解撑着：4 颗星时 4 个观测正好解 4 个未知数（XYZ + 钟），
    // 残差恒为 0，滤波器没有任何冗余能发现码的偏差。实测 PDOP 12.66 那一历元码残差对参考
    // 只有 ±5 m，放大后位置偏 58 m；等第五颗星进来 PDOP 掉到 4.6，误差立刻降到 21 m。
    // 这种历元给解就是给错解，按 PDOP 估出的精度超限就不发布。
    if (hasSol && !prevSolved && mEpochPdop * mConfig.codeSigma > kSppPosGateM) {
        result.xyz = Eigen::Vector3d::Zero();
        result.numSats = 0;
        return false;
    }
    if (hasSol && mHasMotionRef) {
        // 断档恢复首历元的位置只能靠外推垫着：解算结果若离开「上一段运动 + 匀速外推」超过
        // 物理可能范围，说明它落进了另一个收敛域（实测断档 58 s 给出 466 km 的假解）。
        // 这时既不能输出，也不能让滤波器从这个点起步，否则下一历元按它差分出的速度会到
        // 光速量级，整段再也回不来。
        if (const double g = obs.epoch - mMotionRefEpoch; mDirection * g > 0.0 && mDirection * g <= kMotionTrustSpanS) {
            const Eigen::Vector3d pred = mMotionRefPos + mMotionRefVel * g;
            if (const double thr = 0.5 * kAccelEnvelope * g * g + kSppFailJumpM;
                isSaneECEF(pred) && (result.xyz - pred).norm() > thr) {
                result.xyz = Eigen::Vector3d::Zero();
                ekf.setBootstrapPosition(pred, 10.0 * 10.0); // σ=10 m
                return false;
            }
        }
    }
    if (hasSol && !mSppGood) {
        // SPP 不可信时位置只由相位撑着。几何退化历元（PDOP 超限）本来就没有定轨能力，
        // 解出来偏离外推几十米就说明它在跟着噪声走，这种值不给比给错好；
        // 几何尚可但 SPP 残差超阈值的历元，留着 300 m 的容差等相位把位置拉回来。
        if (const double jump = (result.xyz - mReanchorPos).norm(); jump > (mGeomWeak ? 50.0 : kSppFailJumpM)) {
            result.xyz = Eigen::Vector3d::Zero();
            return false;
        }
    }
    if (mSppGood) {
        xyz = mReanchorPos;
        computeElevAzim();
        // 加速度取相邻历元速度的差分。退化几何下速度本身就带百米级误差，差出来的不是加速度，
        // 只在 SPP 锚点可信的连续历元上更新，其余历元沿用上一段的有效值。
        if (ok && result.xyz.squaredNorm() > 1e12) {
            if (mHasPrevVel) {
                if (const double dtv = obs.epoch - mPrevVelTime; dtv != 0.0 && std::fabs(dtv) <= 1.5) {
                    const Eigen::Vector3d a = (mRecvVel - mPrevVel) / dtv;
                    if (a.norm() < kAccelLimit) mRecvAcc = a;
                }
            }
            mPrevVel = mRecvVel;
            mPrevVelTime = obs.epoch;
            mHasPrevVel = true;
        }
    }
    mPrevEpochSolved = result.xyz.squaredNorm() > 1e12;
    return ok;
}

void LEO::applyOptions(const SolverOptions &o) {
    PPP::Core::applyOptions(o);
    setTropo(false, o.spp.ztdInitValue, o.spp.ztdInitSigma, o.spp.ztdProcSigma);
    mMotionThresh = o.motionThresh;
    mSppVsPredThresh = o.sppVsPredThresh;
    mConfig.sppResidualThr = o.spp.sppResidualThr;
}
