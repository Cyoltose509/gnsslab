#include "LEO.h"
#include "Const.h"
#include <Eigen/Eigen>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <map>
#include <vector>
#include "SolverLSQ.h"
#include "GnssStruct.h"

// 位置向量合理性判据：有限 + 处于地球附近（模长 1e5~1e8.5 m 量级）
static bool saneVec(const Vector3d &v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z())
           && v.squaredNorm() > 1.0e10 && v.squaredNorm() < 1.0e17;
}


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
    R.col(0) = ut; // 沿迹 Xb
    R.col(1) = uc; // 轨道法向 Yb
    R.col(2) = ur; // 径向 Zb
    return R;
}

// 星载 CoG→ARP 杠杆臂(ECEF)
Vector3d LEO::recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const {
    if (!mHasAux) return Vector3d::Zero();
    const Vector3d offset = mLeoAntOffset;
    const Matrix3d R = BODY2ECEF(xyzEst, mLeoVel);
    return -R * offset;
}

// 相位缠绕接收机天线基
void LEO::recvWindupBasis(const SatID &, const Vector3d &recv, const CommonTime &epoch,
                          Vector3d &exr, Vector3d &eyr) const {
    const Vector3d v = mLeoVel;
    const Vector3d ur = recv.normalized();
    const Vector3d ut = v.squaredNorm() > 1.0 ? v.normalized() : Vector3d(0, 0, 1);
    Vector3d uc = ur.cross(ut);
    if (uc.squaredNorm() < 1e-6) uc = Vector3d(1, 0, 0);
    uc.normalize();
    exr = ut;
    eyr = -uc;
}

void LEO::bootstrapFirstEpoch(ObsData &obsData) {
    if (!mFirstEpoch) return;
    mRefPos = mRefPos.squaredNorm() > 1e12 ? mRefPos : obsData.antennaPosition;
    mFirstEpoch = false;
}

Vector3d LEO::predictPosition([[maybe_unused]] double dt, [[maybe_unused]] const CommonTime &epoch) const {
    const Vector3d fresh = saneVec(mRefPos) ? mRefPos : mCurObs ? mCurObs->antennaPosition : Vector3d::Zero();
    const Vector3d post = mEkfInit ? currentEstimatedPosition() : Vector3d::Zero();
    Vector3d x;
    if (mGapThisEpoch || !saneVec(post))
        x = fresh;
    else
        x = post;
    if (!saneVec(x)) x = mRefPos;

    if (!mCurObs) {
        mReanchorPos = x;
        return x;
    }

    // 维护备用锚点（fallback 种子来源）
    constexpr double sppThr = 10.0; // SPP 后验残差 RMS 阈值(m)
    if (mGapThisEpoch && saneVec(fresh)) mLastGoodAnchor = fresh;
    else if (!saneVec(mLastGoodAnchor)) mLastGoodAnchor = saneVec(fresh) ? fresh : mRefPos;

    // 选线性化点：seed 解 → 质量差则用 lastGoodAnchor/fresh 重算挑 RMS 最小
    auto [xf, n, q] = sppReanchor(x);
    if (q > sppThr && saneVec(mLastGoodAnchor)) {
        if (auto [xa, na, qa] = sppReanchor(mLastGoodAnchor); na >= 4 && qa < q) {
            xf = xa;
            n = na;
            q = qa;
        }
    }
    if (q > sppThr && saneVec(fresh)) {
        if (auto [xb, nb, qb] = sppReanchor(fresh); nb >= 4 && qb < q) {
            xf = xb;
            n = nb;
            q = qb;
        }
    }
    return applyReanchorResult(xf, n, q, fresh);
}

// 重锚定质量判据 + 锚点簿记，返回本历元最终线性化点
Vector3d LEO::applyReanchorResult(Vector3d xf, const int n, const double q, const Vector3d &fresh) const {
    constexpr double sppThr = 10.0; // SPP 后验残差 RMS 阈值(m)
    mSppGood = n >= 4 && q <= sppThr;
    if (mSppGood) {
        mLastGoodAnchor = xf; // 仅质量好时更新可信锚点
        mReanchorPos = xf;
    } else {
        const Vector3d fb = saneVec(fresh) ? fresh : mLastGoodAnchor;
        xf = fb;
        mReanchorPos = fb;
    }
    return xf;
}

std::tuple<Vector3d, int, double> LEO::sppReanchor(const Vector3d &seed) const {
    if (!mCurObs) return {seed, 0, 1.0e9};
    const CommonTime epoch = mCurObs->epoch;
    const std::string &station = mCurObs->station;

    // 未知数与 SPP 对齐：dx/dy/dz + 各系统钟差（cdt / cdt2 ...）
    const Variable dx(station, Parameter::dX);
    const Variable dy(station, Parameter::dY);
    const Variable dz(station, Parameter::dZ);
    std::map<char, Variable> cdtVars;
    for (const auto &[sys, param]: sysCdtParam)
        cdtVars.try_emplace(sys, station, param);

    Vector3d xc = seed;
    std::map<char, double> clk;
    for (const auto &[sys, _]: cdtVars) clk[sys] = 0.0;
    int lastN = 0;

    // 单星几何量（含高度角/可用性过滤）；返回 false 表示该星跳过
    auto satGeom = [&](const SatID &sat, const auto &tv, const Vector3d &recvPCOe,
                       double &elev, double &rho0, Vector3d &los, double &obsMinusClk) -> bool {
        if (!ifCodeTypes.count(sat.system)) return false;
        const FreqCombo &def = ifCodeTypes.at(sat.system);
        if (!def.available(tv, true)) return false;
        const auto itP = satPVTRecTime.find(sat);
        if (itP == satPVTRecTime.end()) return false;
        elev = 0.0;
        if (const auto itE = satElevData.find(sat); itE != satElevData.end()) elev = itE->second;
        if (elev < cutOffElev) return false;
        const PVT &pvt = itP->second;
        const Vector3d recvEff = xc + recvPCOe;
        rho0 = (pvt.p - recvEff).norm();
        los = -(pvt.p - recvEff) / rho0; // d(rho)/d(x)
        const double dts = pvt.clockBias * C_MPS;
        const double rel = pvt.relativityCorrection * C_MPS;
        const double P_IF = def.combineCodeFromObs(tv) - mOsb.codeBias(def, sat);
        obsMinusClk = P_IF + dts + rel - clk[sat.system];
        return true;
    };

    SolverLSQ solver;
    for (int it = 0; it < 8; ++it) {
        EquSys eq;
        eq.station = station;
        eq.varSet.insert(dx);
        eq.varSet.insert(dy);
        eq.varSet.insert(dz);
        for (const auto &[sys, v]: cdtVars) eq.varSet.insert(v);

        const Vector3d recvPCOe = recvAntennaOffsetECEF(xc, epoch);
        int nObs = 0;
        for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
            double elev, rho0; Vector3d los; double obsMinusClk;
            if (!satGeom(sat, tv, recvPCOe, elev, rho0, los, obsMinusClk)) continue;
            double weight = 1.0 / (sigIFCode * sigIFCode);
            if (elev < PI / 6) {
                const double s = std::sin(elev);
                weight *= s * s;
            }
            EquData ed;
            ed.prefit = obsMinusClk - rho0;
            ed.varCoeffData[dx] = los[0];
            ed.varCoeffData[dy] = los[1];
            ed.varCoeffData[dz] = los[2];
            ed.varCoeffData[cdtVars[sat.system]] = 1.0;
            ed.weight = weight;
            eq.obsEquData[EquID(sat, "IF")] = ed;
            ++nObs;
        }
        if (nObs < 4) break;
        lastN = nObs;
        try {
            solver.solve(eq); // 统一走 SolverLSQ（LDCT 求解法方程），替代手写 LLT
        } catch (...) {
            break; // 法方程奇异 → 停止迭代
        }
        const Vector3d dpos(solver.getSolution(Parameter::dX),
                            solver.getSolution(Parameter::dY),
                            solver.getSolution(Parameter::dZ));
        xc += dpos;
        for (const auto &[sys, v]: cdtVars) clk[sys] += solver.getSolution(v.getParaType());
        if (dpos.norm() < 1e-4) break; // 收敛
    }
    // 后验残差 RMS：SolverLSQ 解完即填 v（等价于原第二遍手算 prefit）
    const double rms = (lastN >= 4 && solver.v.size() > 0)
                       ? std::sqrt(solver.v.squaredNorm() / solver.v.size()) : 1.0e9;
    return {xc, lastN, rms};
}

bool LEO::process(ObsData &obs) {
    mCurObs = &obs; // 供 predictPosition 重锚定读伪距
    const bool ok = PPP::process(obs);
    mCurObs = nullptr;
    if (ok && result.xyz.squaredNorm() > 1e12 && !mSppGood) {
        result.xyz = Vector3d::Zero();
        mForceAmbReset = true;
        ++mExcludedEpochs;
        return false;
    }
    if (ok && result.xyz.squaredNorm() > 1e12) {
        // 纯运动学发散兜底
        if (mReanchorPos.squaredNorm() > 1e12) {
            if ((result.xyz - mReanchorPos).norm() > mDivergeJumpThresh) {
                result.xyz = mReanchorPos;
                mForceAmbReset = true;
                ++mDivergeRecovers;
            }
        }
        if (mHasPrev && obs.epoch > mPrevEpoch)
            mLeoVel = (result.xyz - mPrevEstPos) / (obs.epoch - mPrevEpoch);
        mPrevEstPos = result.xyz;
        mPrevEpoch = obs.epoch;
        mHasPrev = true;
    }
    if (mSppGood) {
        xyz = mReanchorPos;
        computeElevAzim();
    }
    return ok;
}
