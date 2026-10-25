
#include "Log.h"
#include "GnssEKF.h"


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
            // 周跳软重置(slip)
            if (const auto p = var.getParaType(); csData && p == Parameter::ambiguity && (*csData)[var] >= 2.0 && mForceValue.count(var)) {
                currentState(ci) = mForceValue[var];
                currentCov(ci, ci) = 3600.0;
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

    // 状态转移与过程噪声
    Eigen::MatrixXd phi = Eigen::MatrixXd::Identity(numUnk, numUnk);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(numUnk, numUnk);

    int ii = 0;
    for (const auto &var: currentUnkSet) {
        if (auto p = var.getParaType(); p == Parameter::dX || p == Parameter::dY || p == Parameter::dZ) {
            phi(ii, ii) = 1.0; // 位置：携均值（随机常数）
            if (posProcNoise > 0.0) Q(ii, ii) = posProcNoise; // 运动学 PPP：位置白噪声过程
        } else if (p == Parameter::cdt || p == Parameter::cdt2 ||
                   p == Parameter::cdt3 || p == Parameter::cdt4) {
            phi(ii, ii) = 1.0; // 钟差：携均值
            const bool isRef = p == Parameter::cdt;
            const bool isbOn = !isRef && isbInitVar > 0.0;
            const double pn = isbOn ? isbProcNoise : clockProcNoise; // 参考钟走 clock*；ISB 载体(cdt2~4)走 isb*
            const double iv = isbOn ? isbInitVar : clockInitVar;
            if (pn > 0.0) {
                // 随机游走钟差
                Q(ii, ii) = pn;
            } else if (isRef || !isbOn) {
                for (int j = 0; j < numUnk; ++j) {
                    P(ii, j) = 0.0;
                    P(j, ii) = 0.0;
                }
                P(ii, ii) = iv;
            }
        } else if (p == Parameter::ztd) {
            phi(ii, ii) = 1.0; // 对流层湿延迟：随机游走
            Q(ii, ii) = ztdProcNoise;
        } else if (p == Parameter::ambiguity) {
            if (csData && (*csData)[var] >= 2.0) {
                phi(ii, ii) = 1.0; // 软重置：钉在 reinit(prefit)，不漂移、不动位置
                Q(ii, ii) = 0.0; // 无过程噪声
            } else if (csData && (*csData)[var] > 0.5) {
                phi(ii, ii) = 0.0;
                Q(ii, ii) = ambInitVar;
            } else {
                phi(ii, ii) = 1.0; // 常数模糊度（随机游走方差=ambProcNoise）
                Q(ii, ii) = ambProcNoise;
            }
        } else if (p == Parameter::dOX || p == Parameter::dOY || p == Parameter::dOZ) {
            // 接收机天线/参考点常值 ECEF 偏移
            phi(ii, ii) = 1.0;
            Q(ii, ii) = 1.0E-10;
        } else if (p == Parameter::codeBias) {
            // 每星伪距系统偏置
            phi(ii, ii) = 1.0;
            Q(ii, ii) = codeBiasProcNoise;
        } else if (p == Parameter::ifb) {
            // 逐 GLONASS 星 IFCB
            phi(ii, ii) = 1.0;
            Q(ii, ii) = gloIfcbProcNoise;
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
    if (p == Parameter::cdt) return clockInitVar; // 默认 1e4(地基 PPP)
    // ISB 载体(cdt2~4)：isbInitVar<=0 表示未启用 ISB 改造，沿用 clockInitVar 白噪声旧口径
    if (p == Parameter::cdt2 || p == Parameter::cdt3 || p == Parameter::cdt4)
        return isbInitVar > 0.0 ? isbInitVar : clockInitVar;
    if (p == Parameter::ztd)
        return ztdInitVar;
    if (p == Parameter::ambiguity)
        return ambInitVar;
    if (p == Parameter::dOX || p == Parameter::dOY || p == Parameter::dOZ)
        return 10.0; // 常值偏移初方差 (≈3.2 m)²，允许收敛到真实偏置
    if (p == Parameter::codeBias)
        return codeBiasInitVar;
    if (p == Parameter::ifb)
        return gloIfcbInitVar;
    return 1.0E4;
}

void GnssEKF::measUpdate(EquSys &equSys) {
    const auto numObs = static_cast<int>(equSys.obsEquData.size());
    const auto numUnk = static_cast<int>(currentUnkSet.size());

    Eigen::VectorXd prefit = Eigen::VectorXd::Zero(numObs);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(numObs, numUnk);
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(numObs, numObs);

    int iobs = 0;
    for (const auto &[eid, ed]: equSys.obsEquData) {
        prefit(iobs) = ed.prefit;
        for (const auto &[var, coeff]: ed.varCoeffData) {
            const int idx = currentIndexData.at(var);
            H(iobs, idx) = coeff;
        }
        W(iobs, iobs) = ed.weight;
        iobs++;
    }


    if (const int rc = kalmanFilter.measUpdate(prefit, H, W); rc != 0) {
        std::cerr << "measUpdate skipped: singular geometry (rc=" << rc << ")\n";
        return;
    }

    solution = kalmanFilter.xhat;
    covMatrix = kalmanFilter.P;
    xhat = kalmanFilter.xhat;
    P = kalmanFilter.P;
    postfitResidual = kalmanFilter.postfitResidual;


    // 位置后验方差下限(各向异性)
    if (const bool useVecFloor = posVarFloorVec.x() > 0.0 || posVarFloorVec.y() > 0.0 || posVarFloorVec.z() > 0.0;
        posVarFloor > 0.0 || useVecFloor) {
        for (const auto &var: currentUnkSet) {
            const auto pt = var.getParaType();
            if (pt != Parameter::dX && pt != Parameter::dY && pt != Parameter::dZ) continue;
            const int idx = currentIndexData.at(var);
            double fl = posVarFloor;
            if (useVecFloor) {
                if (pt == Parameter::dX) fl = posVarFloorVec.x();
                else if (pt == Parameter::dY) fl = posVarFloorVec.y();
                else fl = posVarFloorVec.z();
            }
            if (fl > 0.0 && P(idx, idx) < fl) {
                const double add = fl - P(idx, idx);
                P(idx, idx) += add;
                kalmanFilter.P(idx, idx) += add;
                covMatrix(idx, idx) += add;
            }
        }
    }

    kalmanFilter.xhatminus = kalmanFilter.xhat;
    kalmanFilter.Pminus = kalmanFilter.P;

    const double dx = getSolution(Parameter::dX, currentUnkSet, solution);
    const double dy = getSolution(Parameter::dY, currentUnkSet, solution);
    const double dz = getSolution(Parameter::dZ, currentUnkSet, solution);
    dxyz = Eigen::Vector3d(dx, dy, dz);

    oldUnkSet = currentUnkSet;
    oldIndexData = currentIndexData;
}

void GnssEKF::createIndex(const VariableSet &varSet) {
    int index = 0;
    for (const auto &var: varSet) currentIndexData[var] = index++;
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
