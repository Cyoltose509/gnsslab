#pragma once

#include "GnssStruct.h"
#include <map>

using namespace Eigen;

class SolverLSQ {
public:
    SolverLSQ() = default;

    virtual void solve(EquSys &equSys);

    // 满阵观测权(广义最小二乘)：fullWeight 为 m×m 观测权阵 P = R⁻¹，用于观测误差相关的场合
    // (双差共享参考星，见 RTK)。nullptr = 退回 weights.asDiagonal() 的对角近似。
    void solve(EquSys &equSys, const MatrixXd *fullWeight);

    double getSolution(const Parameter &type) {
        return getSolution(type, currentUnkSet, state);
    }

    /// Destructor.
    virtual ~SolverLSQ() = default;

    MatrixXd covMatrix;
    double sigma0{};

    VectorXd state;
    VectorXd prefit;
    MatrixXd hMatrix;
    VectorXd weights;
    VectorXd v;
    VariableSet currentUnkSet;

private:
    void solveImpl(EquSys &equSys, const MatrixXd *fullWeight);

    // 变量→列索引缓存（varSet 不变时避免每次 O(n) 线性扫描）。
    // 仅当 equSys.varSet 内容变化才重建，单次求解内 O(log n) 查表。
    int getIndex(const Variable &thisVar) const {
        auto it = m_varIndex.find(thisVar);
        return it != m_varIndex.end() ? it->second : 0;
    }

    static double getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const VectorXd &stateVec);

    VariableSet m_varSetCache;          // 上次求解的变量集合（用于检测变化）
    std::map<Variable, int> m_varIndex;  // 变量→列索引（O(log n) 查表）

}; // End of class 'SolverLSQ'
