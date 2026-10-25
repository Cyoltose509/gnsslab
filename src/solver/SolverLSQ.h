#pragma once

#include "GnssStruct.h"
#include <map>
#include <Eigen/Eigen>


class SolverLSQ {
public:
    SolverLSQ() = default;

    // 最小二乘解算。fullWeight 为 m×m 观测权阵 P=R⁻¹
    void solve(EquSys &equSys,
               const Eigen::MatrixXd *fullWeight = nullptr,
               bool computeCov = true);

    // 用最近一次 solve 的正规方程分解补算协方差阵（等价于 solve 内计算，结果逐位相同）。
    void computeCovariance();

    double getSolution(const Parameter &type) {
        return getSolution(type, currentUnkSet, state);
    }

    void setWeightBlocks(const std::vector<std::vector<int>> *blocks) { mWeightBlocks = blocks; }

    /// Destructor.
    virtual ~SolverLSQ() = default;

    Eigen::MatrixXd covMatrix;
    double sigma0{};

    Eigen::VectorXd state;
    Eigen::VectorXd prefit;
    Eigen::MatrixXd hMatrix;
    Eigen::VectorXd weights;
    Eigen::VectorXd v;
    VariableSet currentUnkSet;

private:
    // 变量→列索引缓存（varSet 不变时避免每次 O(n) 线性扫描）。
    // 仅当 equSys.varSet 内容变化才重建，单次求解内 O(log n) 查表。
    [[nodiscard]] int getIndex(const Variable &thisVar) const {
        const auto it = m_varIndex.find(thisVar);
        return it != m_varIndex.end() ? it->second : 0;
    }

    static double getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const Eigen::VectorXd &stateVec);

    VariableSet m_varSetCache;          // 上次求解的变量集合（用于检测变化）
    std::map<Variable, int> m_varIndex;  // 变量→列索引（O(log n) 查表）

    const std::vector<std::vector<int>> *mWeightBlocks = nullptr;  // 满阵权块结构（块对角），见 setWeightBlocks
    Eigen::LDLT<Eigen::MatrixXd> mLastDecomp;  // 最近一次 solve 的正规方程分解（供 computeCovariance 复用）
    bool mHasDecomp = false;

};
