#pragma once

#include "GnssStruct.h"

using namespace Eigen;

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

}; // End of class 'SolverLSQ'
