#include "SolverLSQ.h"

using namespace std;


void SolverLSQ::solve(EquSys &equSys) {
    currentUnkSet = equSys.varSet;
    const auto numUnk = static_cast<int>(currentUnkSet.size());
    const auto numObs = static_cast<int>(equSys.obsEquData.size());

    // ---- 矩阵预分配：只在维度变化时 resize，平时 setZero 复用内存 ----
    if (prefit.size() != numObs) {
        prefit.resize(numObs);
        weights.resize(numObs);
    }
    if (hMatrix.rows() != numObs || hMatrix.cols() != numUnk) {
        hMatrix.resize(numObs, numUnk);
    }
    prefit.setZero();
    hMatrix.setZero();
    weights.setZero();

    // ---- 组装设计矩阵 H 和观测值向量 prefit ----
    int iobs = 0;
    for (const auto &[id, data]: equSys.obsEquData) {
        prefit(iobs) = data.prefit;

        for (const auto &[var, value]: data.varCoeffData) {
            const int indexUnk = getIndex(currentUnkSet, var);
            hMatrix(iobs, indexUnk) = value;
        }
        weights(iobs) = data.weight;

        iobs++;
    }

    const MatrixXd hT = hMatrix.transpose();

    if (prefit.size() != hMatrix.rows()) {
        throw InvalidSolver("prefit size don't equal with rows of hMatrix");
    }

    // ---- 正规方程：使用 weights.asDiagonal() 替代稠密 wMatrix ----
    MatrixXd N = hT * weights.asDiagonal() * hMatrix;
    const VectorXd b = hT * weights.asDiagonal() * prefit;

    try {
        const LDLT<MatrixXd> ldlt(N);
        state = ldlt.solve(b);
        covMatrix = ldlt.solve(MatrixXd::Identity(N.rows(), N.cols()));
    } catch (...) {
        throw InvalidSolver("LDLT failed, matrix singular or ill-conditioned");
    }

    v = prefit - hMatrix * state;
    const int dof = numObs - numUnk;
    if (dof > 0) {
        const double sigma0_sq = (v.array() * weights.array() * v.array()).sum() / dof;
        sigma0 = sqrt(sigma0_sq);
    } else {
        sigma0 = 1.0;  // 方程数刚好等于未知数，无法估计 sigma0
    }
}

int SolverLSQ::getIndex(const VariableSet &varSet, const Variable &thisVar) {
    int index(0);
    for (const auto &var: varSet) {
        if (var == thisVar) {
            break;
        }
        index++;
    }
    return index;
}

double SolverLSQ::getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const VectorXd &stateVec) {
    auto varIt = currentUnkSet.begin();
    int index = 0;
    while (varIt != currentUnkSet.end()) {
        if (varIt->getParaType() == type) {
            return stateVec(index);
        }
        index++;
        ++varIt;
    }

    throw InvalidRequest("SolverLSQ::Type not found in state vector.");
} // End of method 'SolverGeneral::getSolution()'
