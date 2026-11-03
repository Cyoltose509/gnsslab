#pragma once

#include "GnssStruct.h"
#include <map>
#include <Eigen/Eigen>


class SolverLSQ {
public:
    SolverLSQ() = default;

    void solve(EquSys &equSys,
               const Eigen::MatrixXd *fullWeight = nullptr,
               bool computeCov = true);

    void computeCovariance();

    double getSolution(const Parameter &type) {
        return getSolution(type, currentUnkSet, state);
    }

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
    [[nodiscard]] int getIndex(const Variable &thisVar) const {
        const auto it = m_varIndex.find(thisVar);
        return it != m_varIndex.end() ? it->second : 0;
    }

    static double getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const Eigen::VectorXd &stateVec);

    VariableSet m_varSetCache;          // 上次求解的变量集合（用于检测变化）
    std::map<Variable, int> m_varIndex;  // 变量→列索引（O(log n) 查表）

    Eigen::LDLT<Eigen::MatrixXd> mLastDecomp;  // 最近一次 solve 的正规方程分解（供 computeCovariance 复用）
    bool mHasDecomp = false;

};
