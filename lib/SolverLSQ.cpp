

#include "SolverLSQ.h"

using namespace std;


void SolverLSQ::solve(EquSys &equSys) {

    currentUnkSet = equSys.varSet;
    const auto numUnk = static_cast<int>(currentUnkSet.size());
    const auto numObs = static_cast<int>(equSys.obsEquData.size());

    VectorXd prefit = VectorXd::Zero(numObs);
    MatrixXd hMatrix = MatrixXd::Zero(numObs, numUnk);
    MatrixXd wMatrix = MatrixXd::Zero(numObs, numObs);

    int iobs = 0;
    for (const auto &[id, data]: equSys.obsEquData) {
        prefit(iobs) = data.prefit;

        for (const auto &[var, value]: data.varCoeffData) {
            // 从整体的X中搜索当前未知参数的位置
            const int indexUnk = getIndex(currentUnkSet, var);
            // 把偏导数插入到对应的h矩阵中
            hMatrix(iobs, indexUnk) = value;
        }
        wMatrix(iobs, iobs) = data.weight;

        iobs++;
    }

    const MatrixXd hT = hMatrix.transpose();

    if (prefit.size() != hMatrix.rows()) {
        throw InvalidSolver("prefit size don't equal with rows of hMatrix");
    }

    try {
        covMatrix = hT * wMatrix * hMatrix;
        covMatrix = covMatrix.inverse();
    } catch (...) {
        throw  InvalidSolver("Unable to invert matrix covMatrix");
    }

    state = covMatrix * hT * wMatrix * prefit;
    try {
        dXYZ[0] = getSolution(Parameter::dX, currentUnkSet, state);
        dXYZ[1] = getSolution(Parameter::dY, currentUnkSet, state);
        dXYZ[2] = getSolution(Parameter::dZ, currentUnkSet, state);
    } catch (...) {
        dXYZ.setZero();
        // If solving for velocity, we might use dVX, dVY, dVZ
        try {
            dXYZ[0] = getSolution(Parameter::dVX, currentUnkSet, state);
            dXYZ[1] = getSolution(Parameter::dVY, currentUnkSet, state);
            dXYZ[2] = getSolution(Parameter::dVZ, currentUnkSet, state);
        } catch (...) {
            // No position or velocity parameters found
        }
    }
}

int SolverLSQ::getIndex(const VariableSet &varSet, const Variable &thisVar) {
    int index(0);
    for (const auto& var: varSet) {
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
    // Declare an varIterator for 'stateMap' and go to the first element
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
