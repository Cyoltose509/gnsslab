#include "Log.h"
#include "GnssEKF.h"

using namespace Eigen;

void GnssEKF::solve(EquSys &equSys, VariableDataMap &csData)  {
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
            xhat = VectorXd::Zero(numUnk);
            P = MatrixXd::Zero(numUnk, numUnk);
            for (const auto &var : currentUnkSet) {
                const int idx = currentIndexData[var];
                P(idx, idx) = initVariance(var.getParaType());
                if (var.getParaType() == Parameter::ztd) xhat(idx) = ztdInitValue;
            }
        }
        firstTime = false;
    } else {
        // 新旧变量映射
        VectorXd currentState = VectorXd::Zero(numUnk);
        MatrixXd currentCov = MatrixXd::Zero(numUnk, numUnk);

        for (const auto &var : currentUnkSet) {
            const int ci = currentIndexData[var];
            if (oldUnkSet.find(var) != oldUnkSet.end()) {
                const int oi = oldIndexData[var];
                currentState(ci) = solution(oi);
                currentCov(ci, ci) = covMatrix(oi, oi);
                for (const auto &v2 : currentUnkSet) {
                    if (oldUnkSet.find(v2) != oldUnkSet.end()) {
                        const int c2 = currentIndexData[v2];
                        const int o2 = oldIndexData[v2];
                        currentCov(ci, c2) = covMatrix(oi, o2);
                    }
                }
            } else {
                currentState(ci) = (var.getParaType() == Parameter::ztd) ? ztdInitValue : 0.0;
                currentCov(ci, ci) = initVariance(var.getParaType());
            }
        }
        xhat = currentState;
        P = currentCov;
    }

    // 状态转移与过程噪声
    MatrixXd phi = MatrixXd::Identity(numUnk, numUnk);
    MatrixXd Q = MatrixXd::Zero(numUnk, numUnk);

    int ii = 0;
    for (const auto &var : currentUnkSet) {
        auto p = var.getParaType();
        if (p == Parameter::dX || p == Parameter::dY || p == Parameter::dZ) {
            phi(ii, ii) = 1.0;       // 位置：携均值（随机常数）
            if (posProcNoise > 0.0) Q(ii, ii) = posProcNoise;   // 运动学 PPP：位置白噪声过程
        } else if (p == Parameter::cdt || p == Parameter::cdt2 ||
                   p == Parameter::cdt3 || p == Parameter::cdt4) {
            phi(ii, ii) = 1.0;       // 钟差：携均值
            if (clockProcNoise > 0.0) {
                // 随机游走钟差
                Q(ii, ii) = clockProcNoise;
            } else {
                // 白噪声钟差(默认，地基 PPP)
                Q(ii, ii) = 0.0;
                P(ii, ii) = clockInitVar;
            }
        } else if (p == Parameter::ztd) {
            phi(ii, ii) = 1.0;       // 对流层湿延迟：随机游走
            Q(ii, ii) = ztdProcNoise;
        } else if (p == Parameter::ambiguity) {
            if (csData && (*csData)[var] > 0.5) {
                phi(ii, ii) = 0.0;
                Q(ii, ii) = 9.0E+10;  // 周跳：重置
            } else {
                phi(ii, ii) = 1.0;    // 常数模糊度（随机游走方差=ambProcNoise）
                Q(ii, ii) = ambProcNoise;
            }
        } else if (p == Parameter::dOX || p == Parameter::dOY || p == Parameter::dOZ) {
            // 接收机天线/参考点常值 ECEF 偏移：随机常数（极小过程噪声→跨历元钉住）
            phi(ii, ii) = 1.0;
            Q(ii, ii) = 1.0E-10;
        } else if (p == Parameter::iono) {
            phi(ii, ii) = 1.0;
            Q(ii, ii) = 1.0;
        }
        ii++;
    }

    kalmanFilter.reset(xhat, P);
    kalmanFilter.timeUpdate(phi, Q);
}

double GnssEKF::initVariance(const Parameter &p) const {
    if (p == Parameter::dX || p == Parameter::dY || p == Parameter::dZ)
        return posProcNoise > 0.0 ? posProcNoise : 1.0E4;
    if (p == Parameter::cdt || p == Parameter::cdt2 || p == Parameter::cdt3 || p == Parameter::cdt4)
        return clockInitVar;   // 默认 1e4(地基 PPP)
    if (p == Parameter::ztd)
        return ztdInitVar;
    if (p == Parameter::ambiguity)
        return 100.0;
    if (p == Parameter::dOX || p == Parameter::dOY || p == Parameter::dOZ)
        return 10.0;   // 常值偏移初方差 (≈3.2 m)²，允许收敛到真实偏置
    return 1.0E4;
}

void GnssEKF::measUpdate(EquSys &equSys) {
    const auto numObs = static_cast<int>(equSys.obsEquData.size());
   const  auto numUnk = static_cast<int>(currentUnkSet.size());

    VectorXd prefit = VectorXd::Zero(numObs);
    MatrixXd H = MatrixXd::Zero(numObs, numUnk);
    MatrixXd W = MatrixXd::Zero(numObs, numObs);

    int iobs = 0;
    for (const auto &[eid, ed] : equSys.obsEquData) {
        prefit(iobs) = ed.prefit;
        for (const auto &[var, coeff] : ed.varCoeffData) {
            const int idx = currentIndexData.at(var);
            H(iobs, idx) = coeff;
        }
        W(iobs, iobs) = ed.weight;
        iobs++;
    }

    if (const int rc =     kalmanFilter.measUpdate(prefit, H, W); rc != 0) {
        // Singular / ill-conditioned geometry (e.g. low-elevation rank deficiency):
        // skip the measurement update and keep the a-priori state untouched.
        std::cerr << "measUpdate skipped: singular geometry (rc=" << rc << ")\n";
        return;
    }

    solution = kalmanFilter.xhat;
    covMatrix = kalmanFilter.P;
    xhat = kalmanFilter.xhat;
    P = kalmanFilter.P;
    postfitResidual = kalmanFilter.postfitResidual;

    kalmanFilter.xhatminus = kalmanFilter.xhat;
    kalmanFilter.Pminus = kalmanFilter.P;

    const double dx = getSolution(Parameter::dX, currentUnkSet, solution);
    const double dy = getSolution(Parameter::dY, currentUnkSet, solution);
    const double dz = getSolution(Parameter::dZ, currentUnkSet, solution);
    dxyz = Vector3d(dx, dy, dz);

    oldUnkSet = currentUnkSet;
    oldIndexData = currentIndexData;
}

void GnssEKF::createIndex(const VariableSet &varSet) {
    int index = 0;
    for (const auto &var : varSet) currentIndexData[var] = index++;
}

VectorXd GnssEKF::getState(const std::map<int, Variable> &idxToVar) const {
    const int n = static_cast<int>(idxToVar.size());
    VectorXd out = VectorXd::Zero(n);
    for (const auto &[i, var] : idxToVar)
        out(i) = solution[currentIndexData.at(var)];
    return out;
}

MatrixXd GnssEKF::getCovMatrix(const std::map<int, Variable> &idxToVar) const {
    const int n = static_cast<int>(idxToVar.size());
    MatrixXd out = MatrixXd::Zero(n, n);
    for (const auto &[i, var] : idxToVar) {
        const int fi = currentIndexData.at(var);
        for (const auto &[j, var2] : idxToVar) {
            const int fj = currentIndexData.at(var2);
            out(i, j) = covMatrix(fi, fj);
        }
    }
    return out;
}

VectorXd GnssEKF::getPredState(const std::map<int, Variable> &idxToVar) const {
    const int n = static_cast<int>(idxToVar.size());
    VectorXd out = VectorXd::Zero(n);
    for (const auto &[i, var] : idxToVar)
        out(i) = kalmanFilter.xhatminus[currentIndexData.at(var)];
    return out;
}

MatrixXd GnssEKF::getPredCov(const std::map<int, Variable> &idxToVar) const {
    const int n = static_cast<int>(idxToVar.size());
    MatrixXd out = MatrixXd::Zero(n, n);
    for (const auto &[i, var] : idxToVar) {
        const int fi = currentIndexData.at(var);
        for (const auto &[j, var2] : idxToVar) {
            const int fj = currentIndexData.at(var2);
            out(i, j) = kalmanFilter.Pminus(fi, fj);
        }
    }
    return out;
}

double GnssEKF::getSolution(const Parameter &type,
                                 const VariableSet &currentUnkSet,
                                 const VectorXd &stateVec)  {
    auto it = currentUnkSet.begin();
    int idx = 0;
    while (it != currentUnkSet.end()) {
        if (it->getParaType() == type) return stateVec(idx);
        idx++; ++it;
    }
    throw InvalidRequest("GnssEKF::getSolution: type not found");
}

// SPP 式读取：按 Variable 取回状态分量（set 序索引由 currentIndexData 给出）。
double GnssEKF::getSolution(const Variable &var) const {
    const auto it = currentIndexData.find(var);
    if (it == currentIndexData.end()) return 0.0;   // 该变量不在当前未知集中
    const int idx = it->second;
    if (idx < 0 || idx >= solution.size()) return 0.0;
    return solution(idx);
}
