#include "Exception.h"
#include "SolverKalman.h"

using namespace std;

#define debug 0

void SolverKalman::reset(const Eigen::VectorXd &initialState, const Eigen::MatrixXd &initialErrorCovariance) {
    xhat = initialState;
    P = initialErrorCovariance;

    xhatminus = Eigen::VectorXd::Zero(initialState.size());
    Pminus = Eigen::MatrixXd::Zero(initialErrorCovariance.rows(), initialErrorCovariance.cols());
}


int SolverKalman::compute(const Eigen::MatrixXd &phiMatrix, const Eigen::MatrixXd &qMatrix,
                          const Eigen::VectorXd &mVector, const Eigen::MatrixXd &hMatrix, const Eigen::MatrixXd &wMatrix) {
    predict(phiMatrix, xhat, qMatrix);
    if (const int rc = correct(mVector, hMatrix, wMatrix); rc != 0) return -1;
    return 0;
}

int SolverKalman::predict(const Eigen::MatrixXd &phiMatrix, const Eigen::VectorXd &previousState, const Eigen::MatrixXd &qMatrix) {
    const int stateRow(previousState.size());

    const Eigen::MatrixXd dummyControMatrix = Eigen::MatrixXd::Zero(stateRow, 1);
    const Eigen::VectorXd dummyControlInput = Eigen::VectorXd::Zero(1);

    return predict(phiMatrix, previousState, dummyControMatrix, dummyControlInput, qMatrix);}

int SolverKalman::predict(const Eigen::MatrixXd &phiMatrix, const Eigen::VectorXd &previousState, const Eigen::MatrixXd &controlMatrix,
                          const Eigen::VectorXd &controlInput, const Eigen::MatrixXd &qMatrix) {
    try {
        xhatminus = phiMatrix * xhat + controlMatrix * controlInput;
        const Eigen::MatrixXd phiT = phiMatrix.transpose();
        Pminus = phiMatrix * P * phiT + qMatrix;
    } catch (...) {
        throw InvalidSolver("Predict(): Unable to predict next state.");
    }

    return 0;
}

int SolverKalman::correct(const Eigen::VectorXd &mVector, const Eigen::MatrixXd &hMatrix, const Eigen::MatrixXd &wMatrix) {
    const int n = static_cast<int>(hMatrix.cols()); // 状态维数
    const int q = static_cast<int>(hMatrix.rows()); // 观测维数

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(q, q);
    for (int i = 0; i < q; ++i) {
        const double w = wMatrix(i, i);
        R(i, i) = w > 0.0 ? 1.0 / w : 1.0; // R = W⁻¹（W 为对角权阵）
    }

    const Eigen::MatrixXd PHt = Pminus * hMatrix.transpose();
    const Eigen::MatrixXd S = hMatrix * PHt + R;
    const Eigen::LDLT<Eigen::MatrixXd> sldlt(S);
    if (sldlt.info() != Eigen::Success) return -1;

    const Eigen::MatrixXd K = sldlt.solve(PHt.transpose()).transpose();
    const Eigen::VectorXd dx = K * mVector;
    xhat = xhatminus + dx;
    const Eigen::MatrixXd Pnew = (Eigen::MatrixXd::Identity(n, n) - K * hMatrix) * Pminus;
    P = 0.5 * (Pnew + Pnew.transpose());
    postfitResidual = mVector - hMatrix * dx;

    if (!xhat.allFinite() || !P.allFinite()) return -1;
    return 0;
}
