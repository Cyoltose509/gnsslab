#include "Exception.h"
#include "SolverKalman.h"

using namespace Eigen;
using namespace std;

#define debug 0

void SolverKalman::reset(const VectorXd &initialState, const MatrixXd &initialErrorCovariance) {
    xhat = initialState;
    P = initialErrorCovariance;

    xhatminus = VectorXd::Zero(initialState.size());
    Pminus = MatrixXd::Zero(initialErrorCovariance.rows(), initialErrorCovariance.cols());
}


int SolverKalman::compute(const MatrixXd &phiMatrix, const MatrixXd &qMatrix,
                          const VectorXd &mVector, const MatrixXd &hMatrix, const MatrixXd &wMatrix) {
    predict(phiMatrix, xhat, qMatrix);
    if (const int rc = correct(mVector, hMatrix, wMatrix); rc != 0) return -1;
    return 0;
}

int SolverKalman::predict(const MatrixXd &phiMatrix, const VectorXd &previousState, const MatrixXd &qMatrix) {
    const int stateRow(previousState.size());

    const MatrixXd dummyControMatrix = MatrixXd::Zero(stateRow, 1);
    const VectorXd dummyControlInput = VectorXd::Zero(1);

    return predict(phiMatrix, previousState, dummyControMatrix, dummyControlInput, qMatrix);}

int SolverKalman::predict(const MatrixXd &phiMatrix, const VectorXd &previousState, const MatrixXd &controlMatrix,
                          const VectorXd &controlInput, const MatrixXd &qMatrix) {
    try {
        xhatminus = phiMatrix * xhat + controlMatrix * controlInput;
        const MatrixXd phiT = phiMatrix.transpose();
        Pminus = phiMatrix * P * phiT + qMatrix;
    } catch (...) {
        throw InvalidSolver("Predict(): Unable to predict next state.");
    }

    return 0;
}

int SolverKalman::correct(const VectorXd &mVector, const MatrixXd &hMatrix, const MatrixXd &wMatrix) {
    const int m = static_cast<int>(hMatrix.cols()); // number of unknowns

    // Full inverse of the a-priori covariance (handles correlated P correctly).
    const LDLT<MatrixXd> pInvLdlt(Pminus);
    if (pInvLdlt.info() != Success) {
        return -1;
    }
    const MatrixXd Pinv = pInvLdlt.solve(MatrixXd::Identity(m, m)); // Pminus^{-1}

    const MatrixXd HtW = hMatrix.transpose() * wMatrix; // m × numObs
    const MatrixXd HtWH = HtW * hMatrix; // m × m
    const MatrixXd Mmat = HtWH + Pinv;
    const LDLT<MatrixXd> mldlt(Mmat);
    if (mldlt.info() != Success) {
        return -1;
    }

    const VectorXd dx = mldlt.solve(HtW * mVector); // m × 1 correction
    xhat = xhatminus + dx;
    P = mldlt.solve(MatrixXd::Identity(m, m)); // M⁻¹
    postfitResidual = mVector - hMatrix * dx; // O−C − H·dx = post-fit residual
    if (!xhat.allFinite() || !P.allFinite()) return -1;

    return 0;
}
