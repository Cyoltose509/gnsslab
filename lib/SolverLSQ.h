#pragma once

#include <Eigen/Eigen>
#include "GnssStruct.h"

using namespace Eigen;

/*
 * 这个类通过定义通用方程系统，可以被用单点定位，rtk，精密单点定位或者其他最小二乘任务
 */
class SolverLSQ {
public:

    SolverLSQ() = default;

    virtual void solve(EquSys &equSys);

    static int getIndex(const VariableSet &varSet, const Variable &thisVar);

    static double getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const VectorXd &stateVec);

    double getSolution(const Parameter &type) {
        return getSolution(type, currentUnkSet, state);
    }

    Vector3d getdxyz() const{
        return dxyz;
    };

    /// Destructor.
    virtual ~SolverLSQ() = default;

private:

    VectorXd state;
    MatrixXd covMatrix;
    Vector3d dxyz;
    VariableSet currentUnkSet;
}; // End of class 'SolverLSQ'



