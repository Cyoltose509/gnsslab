#include "Static.h"
#include "SolverLSQ.h"
#include "core/Const.h"
#include "core/geom/CoordConvert.h"
#include "core/obs/Weight.h"
#include <algorithm>
#include <cmath>
#include <map>

// 首历元精确初位置
std::pair<Eigen::Vector3d, bool> PPPStatic::estimateInitialPosition() {
    Eigen::Vector3d xc = mRefPos;
    if (!isSaneECEF(xc))
        xc = mCurObs ? mCurObs->antennaPosition : Eigen::Vector3d::Zero();
    const bool anchorTrusted = isSaneECEF(xc);
    if (!anchorTrusted) {
        xyz = xc;
        computeElevAzim(satPVTRecTime, xyz, satElevData, satAzimData);
    }
    const int maxIter = mConfig.maxReanchorIter > 0 ? mConfig.maxReanchorIter : 8;
    for (int it = 0; it < maxIter; ++it) {
        mEpochXyz = xc;
        buildEquSys();
        for (auto it2 = curEq.obsEquData.begin(); it2 != curEq.obsEquData.end();) {
            if (it2->second.isPhase) it2 = curEq.obsEquData.erase(it2);
            else {
                for (auto vit = it2->second.varCoefficient.begin();
                     vit != it2->second.varCoefficient.end();) {
                    if (vit->first.par == Parameter::codeBias &&
                        osbCoversSystem(vit->first.sat.system))
                        vit = it2->second.varCoefficient.erase(vit);
                    else
                        ++vit;
                }
                ++it2;
            }
        }
        curEq.varSet.clear();
        for (const auto &[eid, ed]: curEq.obsEquData)
            for (const auto &[v, c]: ed.varCoefficient) curEq.varSet.insert(v);
        {
            const auto blhAnchor = Coord::XYZToBLH(xc, Frame::WGS84); // B/L 为弧度
            const double cB = std::cos(blhAnchor.B()), sB = std::sin(blhAnchor.B());
            const double cL = std::cos(blhAnchor.L()), sL = std::sin(blhAnchor.L());
            const Eigen::Vector3d up(cB * cL, cB * sL, sB); // 测地垂直基向量
            const Variable vdx(mCurObs->station, Parameter::dX);
            const Variable vdy(mCurObs->station, Parameter::dY);
            const Variable vdz(mCurObs->station, Parameter::dZ);
            // 钉高只许用于可信的线性化点（接收机自报/文件头坐标）：不可信锚点可能差几十米，
            // 钉住会把首历元的高程错误锁进先验与模糊度，之后靠伪距极难爬出来。
            if (mPinInitHeight && anchorTrusted) {
                EquData ed;
                ed.prefit = 0.0;
                ed.varCoefficient[vdx] = up.x();
                ed.varCoefficient[vdy] = up.y();
                ed.varCoefficient[vdz] = up.z();
                ed.weight = 1e4;
                curEq.obsEquData[EquID(SatID(static_cast<SatSystem>('P'), 0), std::string("INIT_UP"))] = ed;
            }
            curEq.varSet.insert(vdx);
            curEq.varSet.insert(vdy);
            curEq.varSet.insert(vdz);
        }
        int nRealSat = 0;
        {
            std::set<SatID> satsInEq;
            for (const auto &[eid, ed]: curEq.obsEquData)
                satsInEq.insert(eid.sat);
            nRealSat = static_cast<int>(satsInEq.size());
        }
        if (nRealSat < 4) return {mRefPos, false};
        try {
            lsq.solve(curEq, curEq.weightVector().asDiagonal().toDenseMatrix());
        } catch (...) {
            return {mRefPos, false};
        }
        const double dx = lsq.getSolution(Parameter::dX);
        const double dy = lsq.getSolution(Parameter::dY);
        const double dz = lsq.getSolution(Parameter::dZ);
        if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) return {mRefPos, false};
        const Eigen::Vector3d d(dx, dy, dz);
        xc += d;
        if (d.norm() < mConfig.convEps) break;
    }
    for (const auto &[sys, par]: mCdtParam) {
        // 配置滤掉的系统不在 varSet 里，getSolution 会抛异常。
        if (!curEq.varSet.count(Variable(mCurObs->station, par))) continue;
        try { mClkBase[sys] = lsq.getSolution(par); } catch (...) { mClkBase[sys] = 0.0; }
    }
    try {
        mZtdBase = lsq.getSolution(Parameter::ztd);
        mZtdBaseValid = std::isfinite(mZtdBase);
    } catch (...) {
        mZtdBaseValid = false;
    }
    mRefPos = xc; // 引导解出的位置即参考位置：LEO 文件头的 APPROX POSITION 可能与实际轨道差上千公里
    return {xc, true};
}

void PPPStatic::buildEquSys() {
    curEq.reset();
    curEq.station = mCurObs->station;
    mAmbResets.clear();

    if (!mWeightInit) {
        const double minSin = mConfig.sinElevFloor(); // 0 = 跟随截止角
        mWeightCode = Weight(mConfig.codeSigma / std::sqrt(2.0), mConfig.codeSigma / std::sqrt(2.0), minSin);
        mWeightPhase = Weight(mConfig.phaseSigma / std::sqrt(2.0), mConfig.phaseSigma / std::sqrt(2.0), minSin);
        mWeightInit = true;
    }

    const Variable vdx(mCurObs->station, Parameter::dX);
    const Variable vdy(mCurObs->station, Parameter::dY);
    const Variable vdz(mCurObs->station, Parameter::dZ);
    curEq.varSet.insert(vdx);
    curEq.varSet.insert(vdy);
    curEq.varSet.insert(vdz);

    resetSlipDetectors();

    // 填充每历元共享派生量（mEpochXyz 由调用方按本轮的线性化点设好）
    mEpochTide = computeTideDisplacement(mEpochXyz, mCurObs->epoch);
    // 锚点不可信时（低于截止角的星不足 4 颗）本历元不筛星。
    int nAboveCutoff = 0;
    for (const auto &[sat, e]: satElevData) if (e >= mConfig.cutoffElevRad) ++nAboveCutoff;
    mSkipElevCutoff = nAboveCutoff < 4;
    mSatMap.clear();
    mSatTropoFixed.clear();
    mSysSeen.clear();
    mCurrentSatSet.clear();
    mGFEpoch = SlipEpoch{};
    mCurCodePhase.clear();

    mEpochTropo.update(prod->tropo, mConfig.tropoMap, mEpochXyz, mCurObs->epoch);
    if (!prod->tro.empty()) {
        if (double gn = 0.0, ge = 0.0, sgn = 0.0, sge = 0.0;
            prod->tro.getTropoGrad(mCurObs->station, mCurObs->epoch, gn, ge, sgn, sge)) {
            mEpochTropo.gradN = gn;
            mEpochTropo.gradE = ge;
        }
    }

    for (const auto &[sat, tv]: mCurObs->satTypeValueData)
        prepareSatellite(sat, tv);

    mPrefitProbe.clear();
    for (const auto &[sat, tv]: mCurObs->satTypeValueData)
        if (mSatMap.count(sat)) addSatEquation(sat, tv, true);

    decidePrefitGate();
    for (const auto &[sat, tv]: mCurObs->satTypeValueData)
        if (mSatMap.count(sat)) addSatEquation(sat, tv, false);

    {
        std::set<SatID> GFExcludeSet;
        const std::set<SatID> GFApplySet = mGFEpoch.decide(mGFCommonModeMinSats, GFExcludeSet);
        for (const auto &sat: GFApplySet) {
            const TypeValueMap &tv = mCurObs->satTypeValueData.at(sat);
            const double map = mSatMap.at(sat);
            resetAmbiguity(sat, tv, map, mSatTropoFixed.at(sat), true);
        }

        for (auto &[eid, ed]: curEq.obsEquData)
            if (GFExcludeSet.count(eid.sat))
                ed.weight = 1.0e-20;
        mForceAmbReset = false;
    }
    for (SatSystem sys: mSysSeen)
        if (auto it = mCdtParam.find(sys); it != mCdtParam.end())
            curEq.varSet.insert(Variable(mCurObs->station, it->second));
    // 公共钟模型下每颗星都引用 cdt，参考系统本历元未必有星，状态集必须无条件带上它。
    if (mCommonClockActive) curEq.varSet.insert(Variable(mCurObs->station, Parameter::cdt1));

    addZTDConstraint();

    mPrevSatSet = std::move(mCurrentSatSet);
}

void PPPStatic::decidePrefitGate() {
    mPrefitBad.clear();
    mPrefitPhaseBad.clear();
    // 首历元钟差基值未建立，判不出单星粗差，从第二历元起逐历元判。
    if (mProcEpoch <= 1 || mPrefitProbe.empty()) return;

    const auto medianBySys = [this] {
        std::map<SatSystem, std::vector<double>> bySys;
        for (const auto &[sat, r]: mPrefitProbe) bySys[sat.system].push_back(r.code);
        std::map<SatSystem, double> med;
        for (auto &[sys, v]: bySys) {
            const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
            std::nth_element(v.begin(), mid, v.end());
            med[sys] = *mid;
        }
        return med;
    };

    // 同系统中位数是公共偏差，扣掉它才不会把钟跳误判成整历元粗差。
    std::map<SatSystem, double> offset = medianBySys();
    const double phaseGate = mKinematic ? kPhasePrefitGateKinematicM : kPhasePrefitGateStaticM;

    int pass = 0, codePass = 0, phasePass = 0;
    for (const auto &[sat, r]: mPrefitProbe) {
        const double o = offset[sat.system];
        // 新弧段的相位预fit 已把钟差反算进模糊度，扣公共偏移反而会误杀
        const double phaseRes = std::abs(r.freshAmb ? r.phase : r.phase - o);
        const bool cOk = std::abs(r.code - o) <= kPrefitGateM;
        const bool pOk = phaseRes <= phaseGate;
        if (cOk) ++codePass;
        if (pOk) ++phasePass;
        if (!cOk || !pOk) mPrefitBad.insert(sat);
        else ++pass;
    }
    // 大面积同向超限是公共项未收敛，不是单星粗差。
    if (pass == 0 || pass < mConfig.minSatsForUpdate) {
        mPrefitBad.clear();
        // 相位整片超限而码够用：只留码定位置，模糊度整段重建。
        if (codePass >= mConfig.minSatsForUpdate && phasePass < mConfig.minSatsForUpdate) {
            for (const auto &[sat, r]: mPrefitProbe) {
                if (const double o = offset[sat.system]; std::abs(r.freshAmb ? r.phase : r.phase - o) > phaseGate) mPrefitPhaseBad.insert(sat);
            }
            mForceAmbReset = true;
        }
    }
}

// 对流层 ZTD 状态 + 先验/外部产品约束方程
void PPPStatic::addZTDConstraint() {
    if (!mHasTropo) return;
    const Variable vztd(mCurObs->station, Parameter::ztd);
    curEq.varSet.insert(vztd);
    for (auto &[eid, ed]: curEq.obsEquData) {
        const double map = mSatMap.count(eid.sat) ? mSatMap.at(eid.sat) : 1.0;
        ed.varCoefficient[vztd] = map;
    }

    const double zwd = mEpochTropo.zwdResolved();
    double ztdTarget = zwd;
    double ztdSigma = mConfig.ztdPriorSigma;
    auto src = "ZTD_PRIOR";
    if (!prod->tro.empty() && mConfig.troZtdSigma > 0.0) {
        if (double ztdTro = 0.0, sigTro = 0.0; prod->tro.getZTD(mCurObs->station, mCurObs->epoch, ztdTro, sigTro)) {
            ztdTarget = ztdTro - mEpochTropo.zhd;
            ztdSigma = mConfig.troZtdSigma;
            src = "ZTD_TRO";
        }
    }
    if (ztdSigma > 0.0) {
        const EquID eid(SatID(static_cast<SatSystem>('T'), 0), std::string(src));
        EquData ed;
        const double ztdCur = ztdPrior();
        ed.prefit = ztdTarget - ztdCur;
        ed.varCoefficient[vztd] = 1.0;
        ed.weight = 1.0 / (ztdSigma * ztdSigma);
        curEq.obsEquData[eid] = ed;
    }
}
