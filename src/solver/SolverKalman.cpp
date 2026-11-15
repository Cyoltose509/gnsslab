#include "Exception.h"
#include "SolverKalman.h"

void SolverKalman::reset(const Eigen::VectorXd &X0, const Eigen::MatrixXd &Dx0) {
    Xhat = X0;
    Dx = Dx0;

    Xminus = Eigen::VectorXd::Zero(X0.size());
    Dp = Eigen::MatrixXd::Zero(Dx0.rows(), Dx0.cols());
}

int SolverKalman::predict(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Dw) {
    const int stateRow = static_cast<int>(Xhat.size());
    const Eigen::MatrixXd dummyB = Eigen::MatrixXd::Zero(stateRow, 1);
    const Eigen::VectorXd dummyU = Eigen::VectorXd::Zero(1);
    return predict(Phi, dummyB, dummyU, Dw);
}

int SolverKalman::predict(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &B,
                          const Eigen::VectorXd &U, const Eigen::MatrixXd &Dw) {
    try {
        Xminus = Phi * Xhat + B * U;
        const Eigen::MatrixXd PhiT = Phi.transpose();
        Dp = Phi * Dx * PhiT + Dw;
    } catch (...) {
        throw InvalidSolver("Predict(): Unable to predict next state.");
    }

    return 0;
}

int SolverKalman::correct(const Eigen::VectorXd &Z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &W) {
    const int n = static_cast<int>(H.cols()); // 状态维数
    const int q = static_cast<int>(H.rows()); // 观测维数

    if (W.rows() != q || W.cols() != q) return -1;
    const Eigen::MatrixXd Dv = W.inverse();
    if (!Dv.allFinite()) return -1;

    const Eigen::MatrixXd DpHt = Dp * H.transpose();
    const Eigen::MatrixXd S = H * DpHt + Dv;
    const Eigen::LDLT<Eigen::MatrixXd> sldlt(S);
    if (sldlt.info() != Eigen::Success) return -1;

    const Eigen::MatrixXd K = sldlt.solve(DpHt.transpose()).transpose();
    const Eigen::VectorXd dx = K * Z;
    Xhat = Xminus + dx;
    const Eigen::MatrixXd DxNew = (Eigen::MatrixXd::Identity(n, n) - K * H) * Dp;
    Dx = 0.5 * (DxNew + DxNew.transpose());
    postfitResidual = Z - H * dx;

    if (!Xhat.allFinite() || !Dx.allFinite()) return -1;
    return 0;
}
