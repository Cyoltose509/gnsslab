#include "LEO.h"
#include "core/geom/CoordStruct.h"
#include "core/geom/Attitude.h"
#include "core/geom/CoordConvert.h"
#include "Log.h"
#include <Eigen/Eigen>
#include <iomanip>
#include <sstream>
#include <vector>
#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/SatID.h"
#include <algorithm>


bool LEO::processEpoch(ObsData &obs) {
    prepareCode(obs);
    const bool prevSolved = mPrevEpochSolved;
    mPrevEpochSolved = false;
    const bool ok = PPPKinematic::processEpoch(obs);
    const bool hasSol = ok && isSaneECEF(result.xyz);
    // 轨道高度物理包络 + 最少 5 星 + σP 上限：跑到地球内部/上万公里外、星数不足只剩状态外推、
    // 或精度已彻底坏掉（位置碰巧过门但 σP 发散），都会输出「标了 Float 却是坏解」的行，一律不发布。
    if (hasSol) {
        const double r = result.xyz.norm();
        const int clockSys = static_cast<int>(std::count_if(curEq.varSet.begin(), curEq.varSet.end(),
                                                            [](const Variable &v) { return v.par.isClock(); }));
        // 每个系统各估一个钟，5 星跨两个系统时方程刚好等于未知数：残差和 σ 都可以很小，
        // 位置误差却由钟/ISB 吸收到几百米。重捕获且 SPP 几何偏弱时再多要 1 颗星：
        // SWARM 实测 5 星假解偏 23 m 而 σ 只有 1.8 m，靠残差门抓不到。
        const bool weakReacq = !prevSolved && mEpochPdop > kWeakReacqPdop;
        const int minSats = std::max(kMinPublishSats, clockSys + 3 + kMinRedundancy) + (weakReacq ? 1 : 0);
        if (r < 6.5e6 || r > 1.0e7 || result.numSats < minSats ||
            result.sigmaP > kMaxSigmaPM) {
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
                ekf.setBootstrapPosition(pred, GnssEKF::kBootstrapPosVar);
                return false;
            }
        }
    }
    if (hasSol) {
        // 偏离本历元锚点超过容差就不发布：锚点可能是 SPP 解，也可能是断档后的运动外推。
        // 实测卫星数掉到 5、SPP 仍达标但滤波已偏离 477 m 时，σP 还只有几百米，下一历元就彻底发散。
        if (const double jump = (result.xyz - mReanchorPos).norm(); jump > (mGeomWeak ? 50.0 : kSppFailJumpM)) {
            result.xyz = Eigen::Vector3d::Zero();
            result.numSats = 0;
            return false;
        }
    }
    if (mSppGood) {
        xyz = mReanchorPos;
        computeElevAzim(satPVTRecTime, xyz, satElevData, satAzimData);
        // 加速度取相邻历元速度的差分。退化几何下速度本身就带百米级误差，差出来的不是加速度，
        // 只在 SPP 锚点可信的连续历元上更新，其余历元沿用上一段的有效值。
        if (ok && isSaneECEF(result.xyz)) {
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
    mPrevEpochSolved = isSaneECEF(result.xyz);
    return ok;
}

void LEO::applyOptions(const SolverOptions &o) {
    PPPStatic::applyOptions(o);
    setTropo(false, o.spp.ztdInitValue, o.spp.ztdInitSigma, o.spp.ztdProcSigma);
    mMotionThresh = o.motionThresh;
    mSppVsPredThresh = o.sppVsPredThresh;
    mConfig.sppResidualThr = o.spp.sppResidualThr;
}

void LEO::prepareCode(ObsData &obs) {
    for (auto &[sat, tv]: obs.satTypeValueData) {
        CodeRef &ref = mCodeRef[sat];
        const auto hasCode = [&tv] {
            return std::any_of(tv.begin(), tv.end(),
                               [](const auto &kv) { return kv.first[0] == 'C'; });
        };
        if (hasCode()) {
            const FreqCombo def = FreqCombo::detect(sat.system, FreqCombo::collectCodes(tv), sat.chan);
            const auto p1 = tv.find(def.code1), p2 = tv.find(def.code2);
            const auto l1 = tv.find(def.phase1), l2 = tv.find(def.phase2);
            if (!def.code1.empty() && p1 != tv.end() && p2 != tv.end() && l1 != tv.end() && l2 != tv.end()) {
                // 本历元有真码：刷新基准，之后的相位增量从这一历元起算
                ref = {obs.epoch, def.code1, def.code2, def.phase1, def.phase2,
                       p1->second, p2->second, l1->second, l2->second, true};
            } else {
                ref.valid = false;
            }
            continue;
        }
        if (!ref.valid) continue;
        if (std::fabs(obs.epoch - ref.epoch) > kCodeBridgeMaxGapS) { ref.valid = false; continue; }
        const auto r1 = tv.find(ref.l1), r2 = tv.find(ref.l2);
        if (r1 == tv.end() || r2 == tv.end()) continue;
        // 失锁后相位整周数已断，不能拿断前的码往下推
        if (const auto it = obs.satTypeLLI.find(sat); it != obs.satTypeLLI.end() &&
            (it->second.count(ref.l1) || it->second.count(ref.l2))) {
            ref.valid = false;
            continue;
        }
        tv[ref.c1] = ref.p1 + (r1->second - ref.ph1);
        tv[ref.c2] = ref.p2 + (r2->second - ref.ph2);
    }
}

void LEO::bindRefPosition() {
    if (!mFirstEpoch) return;
    // 初值只许来自观测文件头或解算自身。参考轨道是真值，只用于 commitEpoch() 事后对比，
    // 拿它当起算点会让精度评估自证其说。星载 RINEX 缺 APPROX POSITION XYZ 时这里留 0，
    // 由后续 LSQ 迭代从该线性化点自行收敛；不私造粗定位，不凭空造初值。
    if (!isSaneECEF(mRefPos)) mRefPos = mCurObs->antennaPosition;
    xyz = mRefPos;
    mFirstEpoch = false;
}

Eigen::Vector3d LEO::predictPosition(const double dt, const CommonTime &epoch) {
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
    if (mGeomWeak) {
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
void LEO::buildEquSys() {
    PPPStatic::buildEquSys();
    if (!mGeomWeak || mEpochPdop <= kPdopReject) return;
    // 4 颗星近共面（实测 PDOP 7.5e6）时位置在法向完全不可观：码观测会把位置状态直接
    // 拉到 3e8 m，之后整段解不回来；降到 PDOP 200 时码解也已带 70–140 m 误差。
    // 这类历元不给解比给错解好。
    curEq.obsEquData.clear();
}
void LEO::setAntennaOffset(const AntennaOffset &offset, const std::string &source) {
    prod->recvAnt = offset;
    prod->recvAntSrc = source;
    const auto vec = [](const Eigen::Vector3d &v) {
        std::ostringstream s;
        s << v.transpose();
        return s.str();
    };
    const AntennaOffset &a = prod->recvAnt;
    const char *unset = " (undefined, 0 assumed)";
    LOG_INFO << "[LEO] antenna correction: COM->ARP = " << vec(a.comToArp) << " m"
            << (a.hasArp ? "" : unset) << ", ARP->APC = " << vec(a.pco) << " m"
            << (a.hasPco ? "" : unset) << ", total " << vec(a.total()) << " m <- " << prod->recvAntSrc;
}

Eigen::Vector3d LEO::recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch,
                                            const SatID &sat) const {
    (void) epoch;
    (void) sat;
    if (!prod->recvAnt.any()) return Eigen::Vector3d::Zero();
    // 第一历元还没有速度估计，LVLH 的沿迹轴无从确定（退化兜底会拿 ECEF 的 Z 轴顶上），
    // 于是 0.3 m 的沿迹分量会被投到一个无关方向。宁可这一历元不做改正，也不要错方向。
    if (mRecvVel.squaredNorm() < 1.0e6) return Eigen::Vector3d::Zero();
    const DCM M = DCM::fromECEFToLVLH(xyzEst, mRecvVel);
    return M.transpose() * prod->recvAnt.total();
}
