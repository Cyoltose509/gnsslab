#pragma once


#include <Eigen/Eigen>

class SolverKalman {
public:
    SolverKalman() = default;

    SolverKalman(const Eigen::VectorXd &X0,
                 const Eigen::MatrixXd &Dx0)
            : Xhat(X0), Dx(Dx0) {
        Xminus = Eigen::VectorXd::Zero(X0.size());
        Dp = Eigen::MatrixXd::Zero(Dx0.rows(), Dx0.cols());
    }

    void reset(const Eigen::VectorXd &X0, const Eigen::MatrixXd &Dx0);

    int predict(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Dw);

    int predict(const Eigen::MatrixXd &Phi,
                const Eigen::MatrixXd &B,
                const Eigen::VectorXd &U,
                const Eigen::MatrixXd &Dw);

    int correct(const Eigen::VectorXd &Z,
                const Eigen::MatrixXd &H,
                const Eigen::MatrixXd &W);

    Eigen::VectorXd Xhat;
    Eigen::MatrixXd Dx;
    Eigen::VectorXd Xminus;
    Eigen::MatrixXd Dp;
    Eigen::VectorXd postfitResidual;
};
