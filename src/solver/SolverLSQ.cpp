#include "SolverLSQ.h"

using namespace std;


void SolverLSQ::computeCovariance() {
    if (!mHasDecomp) return;  // 尚未求解过，无分解可复用
    const int n = static_cast<int>(mLastDecomp.rows());
    covMatrix = mLastDecomp.solve(Eigen::MatrixXd::Identity(n, n));
}

void SolverLSQ::solve(EquSys &equSys, const Eigen::MatrixXd *fullWeight, const bool computeCov) {
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

    Eigen::MatrixXd N;
    Eigen::VectorXd b;
    if (fullWeight) {
        if (mWeightBlocks && !mWeightBlocks->empty()) {
            N = Eigen::MatrixXd::Zero(numUnk, numUnk);
            b = Eigen::VectorXd::Zero(numUnk);
            for (const auto &blk: *mWeightBlocks) {
                const int kk = static_cast<int>(blk.size());
                if (kk == 0) continue;
                Eigen::MatrixXd Hblk(kk, numUnk);  // kk 行 × n 列，行 j = hMatrix 第 blk[j] 行
                Eigen::VectorXd yblk(kk);
                for (int j = 0; j < kk; ++j) {
                    Hblk.row(j) = hMatrix.row(blk[j]);
                    yblk(j) = prefit(blk[j]);
                }
                Eigen::MatrixXd Pblk(kk, kk);
                for (int a = 0; a < kk; ++a)
                    for (int c = 0; c < kk; ++c)
                        Pblk(a, c) = (*fullWeight)(blk[a], blk[c]);
                Eigen::MatrixXd HP = Hblk.transpose() * Pblk;  // n × kk
                N += HP * Hblk;                         // n × n
                b += HP * yblk;                         // n × 1
            }
        } else {
            N = hMatrix.transpose() * *fullWeight * hMatrix;
            b = hMatrix.transpose() * *fullWeight * prefit;
        }
    } else {
        N = hMatrix.transpose() * weights.asDiagonal() * hMatrix;
        b = hMatrix.transpose() * weights.asDiagonal() * prefit;
    }

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
        // 后验方差因子：满阵时后验二次型用 P，否则用 diag(weights)
        sigma0 = fullWeight ? sqrt(v.dot(*fullWeight * v) / dof)
                            : sqrt((v.array() * weights.array() * v.array()).sum() / dof);
    } else {
        sigma0 = 1.0; // 方程数刚好等于未知数，无法估计 sigma0
    }
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
