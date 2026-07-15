#pragma once


#include <Eigen/Eigen>
#include <cmath>

class ARLambda {
public:
    ARLambda() : squaredRatio(0) {}
    virtual ~ARLambda() {}

    Eigen::VectorXd resolve(Eigen::VectorXd &ambFloat, Eigen::MatrixXd &ambCov);

    [[nodiscard]] bool isFixed(const double threshold = 3.0) const { return squaredRatio > threshold; }

    double squaredRatio;

protected:
    int lambda(Eigen::VectorXd &a, Eigen::MatrixXd &Q,
               Eigen::MatrixXd &F, Eigen::VectorXd &s, const int &m = 2);

    static int factorize(const Eigen::MatrixXd &Q, Eigen::MatrixXd &L, Eigen::VectorXd &D);

    static void gauss(Eigen::MatrixXd &L, Eigen::MatrixXd &Z, int i, int j);

    static void permute(Eigen::MatrixXd &L, Eigen::VectorXd &D, int j, double del, Eigen::MatrixXd &Z);

    static void reduction(Eigen::MatrixXd &L, Eigen::VectorXd &D, Eigen::MatrixXd &Z);

    virtual int search(Eigen::MatrixXd &L, Eigen::VectorXd &D,
                       Eigen::VectorXd &zs, Eigen::MatrixXd &zn,
                       Eigen::VectorXd &s, const int &m);
};

