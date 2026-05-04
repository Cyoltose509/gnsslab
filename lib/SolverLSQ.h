#pragma once

#include <Eigen/Eigen>
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

    Vector3d getDXYZ() const{
        return dXYZ;
    };

    /// Destructor.
    virtual ~SolverLSQ() = default;

private:

    VectorXd state;
    MatrixXd covMatrix;
    Vector3d dXYZ;
    VariableSet currentUnkSet;
}; // End of class 'SolverLSQ'



