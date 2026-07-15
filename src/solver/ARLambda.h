/**
 * LAMBDA method for integer ambiguity resolution.
 * Ref: Teunissen, P.J.G. (1995). The least-squares ambiguity decorrelation adjustment.
 */
#ifndef ARLAMBDA_H
#define ARLAMBDA_H

#include <Eigen/Eigen>
#include <cmath>

class ARLambda {
public:
    ARLambda() : squaredRatio(0) {}
    ~ARLambda() {}

    Eigen::VectorXd resolve(Eigen::VectorXd &ambFloat, Eigen::MatrixXd &ambCov);

    bool isFixed(double threshold = 3.0) { return squaredRatio > threshold; }

    double squaredRatio;

protected:
    int lambda(Eigen::VectorXd &a, Eigen::MatrixXd &Q,
               Eigen::MatrixXd &F, Eigen::VectorXd &s, const int &m = 2);

    int factorize(Eigen::MatrixXd &Q, Eigen::MatrixXd &L, Eigen::VectorXd &D);
    void gauss(Eigen::MatrixXd &L, Eigen::MatrixXd &Z, int i, int j);
    void permute(Eigen::MatrixXd &L, Eigen::VectorXd &D, int j, double del, Eigen::MatrixXd &Z);
    void reduction(Eigen::MatrixXd &L, Eigen::VectorXd &D, Eigen::MatrixXd &Z);

    virtual int search(Eigen::MatrixXd &L, Eigen::VectorXd &D,
                       Eigen::VectorXd &zs, Eigen::MatrixXd &zn,
                       Eigen::VectorXd &s, const int &m = 2);
};

#endif
