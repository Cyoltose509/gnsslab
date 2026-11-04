#include "SolverLSQ.h"
#include <cmath>

void SolverLSQ::solve(EquSys &equSys, const Eigen::MatrixXd &weightMatrix, const bool computeCov) {
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
    }
    if (hMatrix.rows() != numObs || hMatrix.cols() != numUnk) {
        hMatrix.resize(numObs, numUnk);
    }
    prefit.setZero();
    hMatrix.setZero();

    // ---- 组装设计矩阵 H 和观测值向量 prefit ----
    int iobs = 0;
    for (const auto &[id, data]: equSys.obsEquData) {
        prefit(iobs) = data.prefit;

        for (const auto &[var, value]: data.varCoeffData) {
            hMatrix(iobs, getIndex(var)) = value;
        }
        iobs++;
    }

    if (prefit.size() != hMatrix.rows()) {
        throw InvalidSolver("prefit size don't equal with rows of hMatrix");
    }
    if (weightMatrix.rows() != numObs || weightMatrix.cols() != numObs) {
        throw InvalidSolver("weightMatrix size mismatch hMatrix rows");
    }

    const Eigen::MatrixXd N = hMatrix.transpose() * weightMatrix * hMatrix;
    const Eigen::VectorXd b = hMatrix.transpose() * weightMatrix * prefit;

    try {
        // 缓存分解供 computeCovariance() 复用；computeCov=false 时跳过协方差阵 O(n³) 计算。
        mLastDecomp.compute(N);
        state = mLastDecomp.solve(b);
        if (computeCov)
            covMatrix = mLastDecomp.solve(Eigen::MatrixXd::Identity(N.rows(), N.cols()));
        mHasDecomp = true;
    } catch (...) {
        throw InvalidSolver("Eigen::LDLT failed, matrix singular or ill-conditioned");
    }

    v = prefit - hMatrix * state;
    if (const int dof = numObs - numUnk; dof > 0) {
        const Eigen::VectorXd Wv = weightMatrix * v;
        sigma0 = std::sqrt((v.array() * Wv.array()).sum() / dof);
    } else {
        sigma0 = 1.0; // 方程数刚好等于未知数，无法估计 sigma0
    }
}

void SolverLSQ::computeCovariance() {
    if (!mHasDecomp) return;  // 尚未求解过，无分解可复用
    const int n = static_cast<int>(mLastDecomp.rows());
    covMatrix = mLastDecomp.solve(Eigen::MatrixXd::Identity(n, n));
}

double SolverLSQ::getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const Eigen::VectorXd &stateVec) {
    auto varIt = currentUnkSet.begin();
    int index = 0;
    while (varIt != currentUnkSet.end()) {
        if (varIt->getParaType() == type) {
            return stateVec(index);
        }
        index++;
        ++varIt;
    }

    throw InvalidRequest(std::string("SolverLSQ::Type not found in state vector: ") + type.toString());
}
