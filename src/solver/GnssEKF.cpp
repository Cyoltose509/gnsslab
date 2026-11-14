
#include <algorithm>
#include <cmath>

#include "Log.h"
#include "GnssEKF.h"

namespace {
    constexpr double kMinVeloSigma = 0.01; // m/s：多普勒测速 σ 下限，防止残余估出 1e-4 量级后变成无穷权
    constexpr double kVeloGate = 4.0;      // 速度新息 χ² 门限（σ 倍数）
}


void GnssEKF::solve(EquSys &equSys, VariableDataMap &csData) {
    timeUpdate(equSys.varSet, &csData);
    measUpdate(equSys);
}

void GnssEKF::timeUpdate(const VariableSet &varSet, VariableDataMap *csData) {
    currentUnkSet = varSet;
    const int numUnk = static_cast<int>(currentUnkSet.size());

    currentIndexData.clear();
    createIndex(currentUnkSet);

    if (firstTime || oldUnkSet.empty()) {
        // 无先前状态可映射
        if (xhat.size() != numUnk) {
            xhat = Eigen::VectorXd::Zero(numUnk);
            P = Eigen::MatrixXd::Zero(numUnk, numUnk);
            for (const auto &var: currentUnkSet) {
                const int idx = currentIndexData[var];
                P(idx, idx) = initVariance(var.getParaType());
                if (var.getParaType() == Parameter::ztd) xhat(idx) = ztdInitValue;
            }
        }
        firstTime = false;
    } else {
        // 新旧变量映射
        Eigen::VectorXd currentState = Eigen::VectorXd::Zero(numUnk);
        Eigen::MatrixXd currentCov = Eigen::MatrixXd::Zero(numUnk, numUnk);

        for (const auto &var: currentUnkSet) {
            const int ci = currentIndexData[var];
            // 周跳软重置(slip)：值用调用方估的初值，方差放到 ambInitSigma 让本历元相位重新定住它
            if (const auto p = var.getParaType(); csData && p == Parameter::ambiguity && (*csData)[var] >= 2.0 && mForceValue.count(var)) {
                currentState(ci) = mForceValue[var];
                currentCov(ci, ci) = ambInitSigma * ambInitSigma;
                continue;
            }
            if (oldUnkSet.find(var) != oldUnkSet.end()) {
                const int oi = oldIndexData[var];
                currentState(ci) = solution(oi);
                currentCov(ci, ci) = covMatrix(oi, oi);
                for (const auto &v2: currentUnkSet) {
                    if (oldUnkSet.find(v2) != oldUnkSet.end()) {
                        const int c2 = currentIndexData[v2];
                        const int o2 = oldIndexData[v2];
                        currentCov(ci, c2) = covMatrix(oi, o2);
                    }
                }
            } else {
                currentState(ci) = var.getParaType() == Parameter::ztd ? ztdInitValue : 0.0;
                currentCov(ci, ci) = initVariance(var.getParaType());
            }
        }
        xhat = currentState;
        P = currentCov;
    }

    Eigen::MatrixXd phi = Eigen::MatrixXd::Identity(numUnk, numUnk);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(numUnk, numUnk);

    int ip[3] = {-1, -1, -1}, iv[3] = {-1, -1, -1};
    for (int ii = 0; ii < numUnk; ++ii) {
        const auto p = idxToVar_[ii].getParaType();
        if (p == Parameter::dX) ip[0] = ii;
        else if (p == Parameter::dY) ip[1] = ii;
        else if (p == Parameter::dZ) ip[2] = ii;
        else if (p == Parameter::dVX) iv[0] = ii;
        else if (p == Parameter::dVY) iv[1] = ii;
        else if (p == Parameter::dVZ) iv[2] = ii;
    }
    // 常速度预测只在位置与速度三态齐备、且已知历元间隔时启用
    const bool cvModel = accProcSigma > 0.0 && motionDt > 0.0 &&
                         ip[0] >= 0 && ip[1] >= 0 && ip[2] >= 0 && iv[0] >= 0 && iv[1] >= 0 && iv[2] >= 0;
    velocityCoupled = cvModel;
    if (cvModel) {
        // 标准 CV 的完整过程噪声：Δt 内加速度视作常值 a~N(0,accProcSigma²)，则
        // Δp=½aΔt²、Δv=aΔt 同源全相关，Q 的位置-速度块为 [[(½aΔt²)², ½a²Δt³], [½a²Δt³, (aΔt)²]]。
        // 交叉项给的是「本历元新生的位置误差」与「新生的速度误差」同向这部分，缺了它两者被当成独立扰动，
        // 先验里位置-速度相关性偏低，增益偏保守。
        const double acc = accProcSigma;
        const double dt = motionDt;
        const double dp = 0.5 * acc * dt * dt;            // 位置增量 σ = ½·a·Δt²
        const double dv = acc * dt;                       // 速度增量 σ = a·Δt
        const double dpv = 0.5 * acc * acc * dt * dt * dt; // 位置-速度协方差 = ½·a²·Δt³
        for (int k = 0; k < 3; ++k) {
            phi(ip[k], iv[k]) = dt;
            Q(ip[k], ip[k]) = dp * dp;
            Q(iv[k], iv[k]) = dv * dv;
            Q(ip[k], iv[k]) = dpv;
            Q(iv[k], ip[k]) = dpv;
        }
    }

    for (int ii = 0; ii < numUnk; ++ii) {
        const Variable &var = idxToVar_[ii];
        const auto p = var.getParaType();
        double q = 0.0;     // 过程噪声（对角，方差；各 σ 成员平方后进阵）
        if (p == Parameter::dX || p == Parameter::dY || p == Parameter::dZ) {
            // 位置过程噪声二选一：常速度模型下位置增量全由加速度项给出（标准 CV），
            // 再叠加逐历元随机游走会把增益顶到 1、预测被压死；无速度态时才用 posProcSigma。
            q = cvModel ? Q(ii, ii) : posProcSigma * posProcSigma;
        } else if (p == Parameter::dVX || p == Parameter::dVY || p == Parameter::dVZ) {
            // 速度只存在于常速度模型里；本历元不预测（首历元/断档）时不给增量，靠多普勒量测拉回。
            q = cvModel ? Q(ii, ii) : 0.0;
        } else if (p.isClock()) {
            const bool isRef = p == Parameter::cdt;
            const bool isbOn = !isRef && isbInitSigma > 0.0;
            const double pn = isbOn ? isbProcSigma * isbProcSigma : clockProcSigma * clockProcSigma; // 参考钟走 clock*；ISB 载体(cdt2~4)走 isb*
            const double iv = isbOn ? isbInitSigma * isbInitSigma : clockInitSigma * clockInitSigma;
            if (pn > 0.0) {
                q = pn;                             // 随机游走钟差
            } else {
                // 随机常数钟差：方差钉死、与一切解耦（参考钟用 clockInitSigma；ISB 用 isbInitSigma）
                for (int j = 0; j < numUnk; ++j) {
                    P(ii, j) = 0.0;
                    P(j, ii) = 0.0;
                }
                P(ii, ii) = iv;
            }
        } else if (p == Parameter::ztd) {
            q = ztdProcSigma * ztdProcSigma;        // 对流层湿延迟：随机游走
        } else if (p == Parameter::ambiguity) {
            // 新弧段/周跳：方差回初值让模糊度重新自由；若留 0 方差，相位就成了硬约束，把码的偏差拽进位置
            q = csData && (*csData)[var] > 0.5 ? ambInitSigma * ambInitSigma : ambProcSigma * ambProcSigma;
        } else if (p == Parameter::dOX || p == Parameter::dOY || p == Parameter::dOZ) {
            q = 1.0E-10;                            // 接收机天线/参考点常值 ECEF 偏移
        } else if (p == Parameter::codeBias) {
            q = codeBiasProcSigma * codeBiasProcSigma; // 每星伪距系统偏置
        } else if (p == Parameter::ifb) {
            q = gloIfcbProcSigma * gloIfcbProcSigma;   // 逐 GLONASS 星 IFCB
        } else if (p == Parameter::iono) {
            q = 1.0;
        }
        Q(ii, ii) = q;
    }

    kalmanFilter.reset(xhat, P);
    kalmanFilter.timeUpdate(phi, Q);
}

void GnssEKF::reparameterize(const VariableSet &newOrder, const Eigen::MatrixXd &T) {
    const int n = static_cast<int>(solution.size());
    if (n <= 0 || T.cols() != n || T.rows() != static_cast<int>(newOrder.size())) return;

    // 换基准是线性重参数化：新息按新基准解释，协方差必须一起转，否则位置-模糊度耦合立刻断掉
    const Eigen::VectorXd xNew = T * solution;
    const Eigen::MatrixXd pNew = T * covMatrix * T.transpose();

    currentUnkSet = newOrder;
    currentIndexData.clear();
    idxToVar_.clear();
    int k = 0;
    for (const auto &v: currentUnkSet) {
        currentIndexData[v] = k;
        idxToVar_.push_back(v);
        ++k;
    }
    solution = xNew;
    covMatrix = pNew;
    xhat = xNew;
    P = pNew;
    kalmanFilter.xhat = xNew;
    kalmanFilter.P = pNew;
    kalmanFilter.xhatminus = xNew;
    kalmanFilter.Pminus = pNew;
    oldUnkSet = currentUnkSet;
    oldIndexData = currentIndexData;
    mForceValue.clear();
}

double GnssEKF::initVariance(const Parameter &p) const {
    if (p == Parameter::dX || p == Parameter::dY || p == Parameter::dZ)
        return posProcSigma > 0.0 ? posProcSigma * posProcSigma : 100.0 * 100.0; // σ=100 m
    if (p == Parameter::dVX || p == Parameter::dVY || p == Parameter::dVZ)
        return velInitSigma * velInitSigma;
    if (p == Parameter::cdt) return clockInitSigma * clockInitSigma;
    // ISB 载体(cdt2~4)：isbInitSigma<=0 表示未启用 ISB 改造，沿用 clockInitSigma 白噪声旧口径
    if (p == Parameter::cdt2 || p == Parameter::cdt3 || p == Parameter::cdt4)
        return isbInitSigma > 0.0 ? isbInitSigma * isbInitSigma : clockInitSigma * clockInitSigma;
    if (p == Parameter::ztd)
        return ztdInitSigma * ztdInitSigma;
    if (p == Parameter::ambiguity)
        return ambInitSigma * ambInitSigma;
    if (p == Parameter::dOX || p == Parameter::dOY || p == Parameter::dOZ)
        return 10.0; // 常值偏移初方差 (≈3.2 m)²，允许收敛到真实偏置
    if (p == Parameter::codeBias)
        return codeBiasInitSigma * codeBiasInitSigma;
    if (p == Parameter::ifb)
        return gloIfcbInitSigma * gloIfcbInitSigma;
    return 1.0E4;
}

void GnssEKF::measUpdate(EquSys &equSys, const Eigen::MatrixXd *fullWeight) {
    const auto numObs = static_cast<int>(equSys.obsEquData.size());
    const auto numUnk = static_cast<int>(currentUnkSet.size());

    Eigen::VectorXd prefit = Eigen::VectorXd::Zero(numObs);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(numObs, numUnk);

    int iobs = 0;
    for (const auto &[eid, ed]: equSys.obsEquData) {
        prefit(iobs) = ed.prefit;
        for (const auto &[var, coeff]: ed.varCoeffData) {
            const int idx = currentIndexData.at(var);
            H(iobs, idx) = coeff;
        }
        iobs++;
    }

    // 满阵权由调用方给（双差站间相关），缺省按方程标量权取对角。
    const Eigen::MatrixXd W = fullWeight ? *fullWeight : equSys.weightVector().asDiagonal().toDenseMatrix();
    if (const int rc = kalmanFilter.measUpdate(prefit, H, W); rc != 0) {
        std::cerr << "measUpdate skipped: singular geometry (rc=" << rc << ")\n";
        return;
    }
    solution = kalmanFilter.xhat;
    covMatrix = kalmanFilter.P;
    xhat = kalmanFilter.xhat;
    P = kalmanFilter.P;
    postfitResidual = kalmanFilter.postfitResidual;


    // 位置后验 σ 下限(各向异性)
    if (const bool useVecFloor = posFloorSigmaVec.x() > 0.0 || posFloorSigmaVec.y() > 0.0 || posFloorSigmaVec.z() > 0.0;
        posFloorSigma > 0.0 || useVecFloor) {
        for (const auto &var: currentUnkSet) {
            const auto pt = var.getParaType();
            if (pt != Parameter::dX && pt != Parameter::dY && pt != Parameter::dZ) continue;
            const int idx = currentIndexData.at(var);
            double fl = posFloorSigma;
            if (useVecFloor) {
                if (pt == Parameter::dX) fl = posFloorSigmaVec.x();
                else if (pt == Parameter::dY) fl = posFloorSigmaVec.y();
                else fl = posFloorSigmaVec.z();
            }
            const double flVar = fl * fl;
            if (fl > 0.0 && P(idx, idx) < flVar) {
                const double add = flVar - P(idx, idx);
                P(idx, idx) += add;
                kalmanFilter.P(idx, idx) += add;
                covMatrix(idx, idx) += add;
            }
        }
    }

    oldUnkSet = currentUnkSet;
    oldIndexData = currentIndexData;
}

void GnssEKF::updateVelocity(const Eigen::Vector3d &vel, const Eigen::Vector3d &sigma) {
    int iv[3] = {-1, -1, -1};
    for (int ii = 0; ii < static_cast<int>(idxToVar_.size()); ++ii) {
        const auto p = idxToVar_[ii].getParaType();
        if (p == Parameter::dVX) iv[0] = ii;
        else if (p == Parameter::dVY) iv[1] = ii;
        else if (p == Parameter::dVZ) iv[2] = ii;
    }
    if (iv[0] < 0 || iv[1] < 0 || iv[2] < 0) return;

    const int numUnk = static_cast<int>(kalmanFilter.xhat.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, numUnk);
    Eigen::VectorXd z(3);
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero(); // 权重阵；SolverKalman 内部取逆得方差阵
    double chi2 = 0.0;
    for (int k = 0; k < 3; ++k) {
        // 多普勒解的 σ 由后验残差估，数据好时会低到 1e-4 m/s，当成无穷权会把位置拽走；
        // 一是给 σ 下限，二是按新息算 χ²，超门限就把量测方差等比放大（软门限，不硬丢历元）。
        const double s = (std::max)(sigma[k], kMinVeloSigma);
        const double sSq = s * s;
        H(k, iv[k]) = 1.0;
        z(k) = vel[k] - kalmanFilter.xhat(iv[k]);
        chi2 += z(k) * z(k) / (kalmanFilter.P(iv[k], iv[k]) + sSq);
        W(k, k) = 1.0 / sSq;
    }
    if (chi2 > kVeloGate * kVeloGate) W /= chi2 / (kVeloGate * kVeloGate);
    // postfitResidual 的语义是位置观测的后验残差（readback 与 sigma0 都读它），速度更新后还原
    const Eigen::VectorXd posPostfit = postfitResidual;
    // 序贯更新：correct() 以 xhatminus/Pminus 为先验，速度量测必须接在位置后验之后，
    // 否则会把位置量测重新拉回预测值（同一历元内位置更新被撤销）。
    kalmanFilter.xhatminus = kalmanFilter.xhat;
    kalmanFilter.Pminus = kalmanFilter.P;
    if (kalmanFilter.measUpdate(z, H, W) != 0) return;
    postfitResidual = posPostfit;
    solution = kalmanFilter.xhat;
    covMatrix = kalmanFilter.P;
    xhat = kalmanFilter.xhat;
    P = kalmanFilter.P;
}

bool GnssEKF::posteriorVelocity(const std::string &station, Eigen::Vector3d &vel,
                                Eigen::Vector3d &sigma) const {
    const Variable vx(station, Parameter::dVX), vy(station, Parameter::dVY), vz(station, Parameter::dVZ);
    const Variable *vars[3] = {&vx, &vy, &vz};
    for (int k = 0; k < 3; ++k) {
        const auto it = currentIndexData.find(*vars[k]);
        if (it == currentIndexData.end() || it->second >= solution.size()) return false;
        vel[k] = solution(it->second);
        sigma[k] = std::sqrt((std::max)(covMatrix(it->second, it->second), 0.0));
    }
    return true;
}

void GnssEKF::createIndex(const VariableSet &varSet) {
    int index = 0;
    idxToVar_.clear();
    idxToVar_.reserve(varSet.size());
    for (const auto &var: varSet) {
        idxToVar_.push_back(var);
        currentIndexData[var] = index++;
    }
}

double GnssEKF::getSolution(const Parameter &type,
                            const VariableSet &currentUnkSet,
                            const Eigen::VectorXd &stateVec) {
    auto it = currentUnkSet.begin();
    int idx = 0;
    while (it != currentUnkSet.end()) {
        if (it->getParaType() == type) return stateVec(idx);
        idx++;
        ++it;
    }
    throw InvalidRequest("GnssEKF::getSolution: type not found");
}

bool GnssEKF::tryGetSolution(const Parameter &type, const VariableSet &currentUnkSet,
                             const Eigen::VectorXd &stateVec, double &out) {
    auto it = currentUnkSet.begin();
    int idx = 0;
    while (it != currentUnkSet.end()) {
        if (it->getParaType() == type) {
            out = stateVec(idx);
            return true;
        }
        idx++;
        ++it;
    }
    return false;
}

// SPP 式读取：按 Variable 取回状态分量（set 序索引由 currentIndexData 给出）。
double GnssEKF::getSolution(const Variable &var) const {
    const auto it = currentIndexData.find(var);
    if (it == currentIndexData.end()) return 0.0; // 该变量不在当前未知集中
    const int idx = it->second;
    if (idx < 0 || idx >= solution.size()) return 0.0;
    return solution(idx);
}

double GnssEKF::priorOf(const Variable &var) const {
    const auto it = currentIndexData.find(var);
    if (it == currentIndexData.end()) return 0.0;
    const int idx = it->second;
    if (idx < 0 || idx >= kalmanFilter.xhatminus.size()) return 0.0;
    return kalmanFilter.xhatminus(idx);
}

Eigen::VectorXd GnssEKF::priorInnovationStd(const EquSys &equSys) const {
    const int n = static_cast<int>(equSys.obsEquData.size());
    Eigen::VectorXd out = Eigen::VectorXd::Zero(n);
    const int dim = static_cast<int>(kalmanFilter.Pminus.rows());
    int i = 0;
    for (const auto &[eid, ed]: equSys.obsEquData) {
        double v = 0.0;
        for (const auto &[va, ca]: ed.varCoeffData) {
            const auto ia = currentIndexData.find(va);
            if (ia == currentIndexData.end() || ia->second >= dim) continue;
            for (const auto &[vb, cb]: ed.varCoeffData) {
                const auto ib = currentIndexData.find(vb);
                if (ib == currentIndexData.end() || ib->second >= dim) continue;
                v += ca * cb * kalmanFilter.Pminus(ia->second, ib->second);
            }
        }
        out(i++) = std::sqrt((std::max)(v, 0.0));
    }
    return out;
}
