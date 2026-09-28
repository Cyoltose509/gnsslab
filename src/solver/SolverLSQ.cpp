#include "SolverLSQ.h"

using namespace std;


void SolverLSQ::solve(EquSys &equSys) {
    solveImpl(equSys, nullptr);
}

void SolverLSQ::solve(EquSys &equSys, const MatrixXd *fullWeight) {
    solveImpl(equSys, fullWeight);
}

void SolverLSQ::solveImpl(EquSys &equSys, const MatrixXd *fullWeight) {
    // 仅在变量集合内容变化时重建索引缓存，避免每次求解 O(n) 线性扫描
    if (m_varSetCache != equSys.varSet) {
        m_varSetCache = equSys.varSet;
        m_varIndex.clear();
        int i = 0;
        for (const auto &v: m_varSetCache) m_varIndex[v] = i++;
    }
    currentUnkSet = m_varSetCache;
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
            const int indexUnk = getIndex(var);
            hMatrix(iobs, indexUnk) = value;
        }
        weights(iobs) = data.weight;

        iobs++;
    }

    if (prefit.size() != hMatrix.rows()) {
        throw InvalidSolver("prefit size don't equal with rows of hMatrix");
    }

    if (fullWeight && fullWeight->rows() != numObs) {
        throw InvalidSolver("fullWeight rows mismatch hMatrix rows");
    }

    // ---- 正规方程 ----
    // 满阵权（观测误差相关，如双差共享参考星）走广义最小二乘 N = Hᵀ·P·H；
    // 否则退化为 weights.asDiagonal() 的对角加权最小二乘。
    // 两种分支必须分开写：Product 与 Product·DiagonalWrapper 之间 Eigen 无法归约公共类型。
    MatrixXd N;
    VectorXd b;
    if (fullWeight) {
        N = hMatrix.transpose() * (*fullWeight) * hMatrix;
        b = hMatrix.transpose() * (*fullWeight) * prefit;
    } else {
        N = hMatrix.transpose() * weights.asDiagonal() * hMatrix;
        b = hMatrix.transpose() * weights.asDiagonal() * prefit;
    }

    try {
        const LDLT<MatrixXd> ldlt(N);
        state = ldlt.solve(b);
        covMatrix = ldlt.solve(MatrixXd::Identity(N.rows(), N.cols()));
    } catch (...) {
        throw InvalidSolver("LDLT failed, matrix singular or ill-conditioned");
    }

    v = prefit - hMatrix * state;
    if (const int dof = numObs - numUnk; dof > 0) {
        // 后验方差因子：满阵时后验二次型用 P，否则用 diag(weights)
        sigma0 = fullWeight ? sqrt(v.dot((*fullWeight) * v) / dof)
                            : sqrt((v.array() * weights.array() * v.array()).sum() / dof);
    } else {
        sigma0 = 1.0; // 方程数刚好等于未知数，无法估计 sigma0
    }
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
}
