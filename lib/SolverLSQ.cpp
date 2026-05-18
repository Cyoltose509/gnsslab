#include "SolverLSQ.h"

using namespace std;


void SolverLSQ::solve(EquSys &equSys) {
    currentUnkSet = equSys.varSet;
    const auto numUnk = static_cast<int>(currentUnkSet.size());
    const auto numObs = static_cast<int>(equSys.obsEquData.size());

    prefit = VectorXd::Zero(numObs);
    hMatrix = MatrixXd::Zero(numObs, numUnk);
    weights = VectorXd::Zero(numObs);
    MatrixXd wMatrix = MatrixXd::Zero(numObs, numObs);

    int iobs = 0;
    for (const auto &[id, data]: equSys.obsEquData) {
        prefit(iobs) = data.prefit;

        for (const auto &[var, value]: data.varCoeffData) {
            const int indexUnk = getIndex(currentUnkSet, var);
            hMatrix(iobs, indexUnk) = value;
        }
        wMatrix(iobs, iobs) = data.weight;
        weights(iobs) = data.weight;

        iobs++;
    }

    const MatrixXd hT = hMatrix.transpose();

    if (prefit.size() != hMatrix.rows()) {
        throw InvalidSolver("prefit size don't equal with rows of hMatrix");
    }

    // 正规方程 N = H^T * W * H
    MatrixXd N = hT * wMatrix * hMatrix;
    // 右端项 b = H^T * W * prefit
    VectorXd b = hT * wMatrix * prefit;

    try {
        LDLT<MatrixXd> ldlt(N);
        state = ldlt.solve(b);
        covMatrix = ldlt.solve(MatrixXd::Identity(N.rows(), N.cols())); // 协方差
    } catch (...) {
        throw InvalidSolver("LDLT failed, matrix singular or ill-conditioned");
    }

    v = prefit - hMatrix * state;
    const int dof = numObs - numUnk;
    if (dof <= 0) {
        throw InvalidSolver("Degree of freedom <= 0, cannot compute sigma0");
    }
    const double sigma0_sq = (v.transpose() * wMatrix * v)(0,0) / dof;
    sigma0 = sqrt(sigma0_sq);
}

int SolverLSQ::getIndex(const VariableSet &varSet, const Variable &thisVar) {
    int index(0);
    for (const auto &var: varSet) {
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
