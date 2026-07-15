#include <iostream>
#include "Log.h"
#include "SolverKalman.h"

using namespace Eigen;

#define debug 0

// ================================================================
// solve: 完整一步 TimeUpdate + MeasUpdate
// ================================================================
void SolverKalman::solve(EquSys &equSys, VariableDataMap &csData)  {
    timeUpdate(equSys.varSet, &csData);
    measUpdate(equSys);
}

// ================================================================
// timeUpdate: 构建 phi/q 矩阵，执行 Kalman 预测
// ================================================================
void SolverKalman::timeUpdate(const VariableSet &varSet, VariableDataMap *csData) {
    currentUnkSet = varSet;
    int numUnk = static_cast<int>(currentUnkSet.size());

    currentIndexData.clear();
    createIndex(currentUnkSet);

    if (firstTime) {
        VectorXd initialState(numUnk);
        initialState.setZero();
        MatrixXd initialCov = MatrixXd::Zero(numUnk, numUnk);
        for (const auto &var : currentUnkSet) {
            int idx = currentIndexData[var];
            initialCov(idx, idx) = 9.0E+10;
        }
        xhat = initialState;
        P = initialCov;
        firstTime = false;
    } else {
        // 新旧变量映射
        VectorXd currentState = VectorXd::Zero(numUnk);
        MatrixXd currentCov = MatrixXd::Zero(numUnk, numUnk);

        for (const auto &var : currentUnkSet) {
            int ci = currentIndexData[var];
            if (oldUnkSet.find(var) != oldUnkSet.end()) {
                int oi = oldIndexData[var];
                currentState(ci) = solution(oi);
                currentCov(ci, ci) = covMatrix(oi, oi);
                for (const auto &v2 : currentUnkSet) {
                    if (oldUnkSet.find(v2) != oldUnkSet.end()) {
                        int c2 = currentIndexData[v2];
                        int o2 = oldIndexData[v2];
                        currentCov(ci, c2) = covMatrix(oi, o2);
                    }
                }
            } else {
                currentCov(ci, ci) = 9.0E+10;
            }
        }
        xhat = currentState;
        P = currentCov;
    }

    // 状态转移与过程噪声
    MatrixXd phi = MatrixXd::Identity(numUnk, numUnk);
    phi.setZero();
    MatrixXd Q = MatrixXd::Zero(numUnk, numUnk);

    int ii = 0;
    for (const auto &var : currentUnkSet) {
        auto p = var.getParaType();
        if (p == Parameter::dX || p == Parameter::dY || p == Parameter::dZ) {
            phi(ii, ii) = 1.0;       // 静态位置
            Q(ii, ii) = 1.0E+4;
        } else if (p == Parameter::cdt || p == Parameter::cdt2) {
            phi(ii, ii) = 0.0;       // 白噪声钟差
            Q(ii, ii) = 9.0E+10;
        } else if (p == Parameter::ambiguity) {
            bool slip = csData && (*csData)[var] > 0.5;
            if (slip) {
                phi(ii, ii) = 0.0;
                Q(ii, ii) = 9.0E+10;  // 周跳：重置
            } else {
                phi(ii, ii) = 1.0;    // 常数模糊度
                Q(ii, ii) = 0.0;
            }
        } else if (p == Parameter::iono) {
            phi(ii, ii) = 1.0;        // 电离层缓变
            Q(ii, ii) = 1.0;
        }
        ii++;
    }

    kalmanFilter.Reset(xhat, P);
    kalmanFilter.TimeUpdate(phi, Q);
}

// ================================================================
// measUpdate: 构建 H/W/prefit 矩阵，执行测量更新
// ================================================================
void SolverKalman::measUpdate(EquSys &equSys) {
    int numObs = static_cast<int>(equSys.obsEquData.size());
    int numUnk = static_cast<int>(currentUnkSet.size());

    VectorXd prefit = VectorXd::Zero(numObs);
    MatrixXd H = MatrixXd::Zero(numObs, numUnk);
    MatrixXd W = MatrixXd::Zero(numObs, numObs);

    int iobs = 0;
    for (const auto &[eid, ed] : equSys.obsEquData) {
        prefit(iobs) = ed.prefit;
        for (const auto &[var, coeff] : ed.varCoeffData) {
            int idx = currentIndexData.at(var);
            H(iobs, idx) = coeff;
        }
        W(iobs, iobs) = ed.weight;
        iobs++;
    }

    kalmanFilter.MeasUpdate(prefit, H, W);

    solution = kalmanFilter.xhat;
    covMatrix = kalmanFilter.P;
    postfitResidual = kalmanFilter.postfitResidual;

    double dx = getSolution(Parameter::dX, currentUnkSet, solution);
    double dy = getSolution(Parameter::dY, currentUnkSet, solution);
    double dz = getSolution(Parameter::dZ, currentUnkSet, solution);
    dxyz = Vector3d(dx, dy, dz);

    oldUnkSet = currentUnkSet;
    oldIndexData = currentIndexData;
}

void SolverKalman::createIndex(const VariableSet &varSet) {
    int index = 0;
    for (const auto &var : varSet) currentIndexData[var] = index++;
}

double SolverKalman::getSolution(const Parameter &type,
                                 VariableSet &unkSet,
                                 const VectorXd &stateVec)  {
    auto it = unkSet.begin();
    int idx = 0;
    while (it != unkSet.end()) {
        if (it->getParaType() == type) return stateVec(idx);
        idx++; it++;
    }
    throw InvalidRequest("SolverKalman::getSolution: type not found");
}
