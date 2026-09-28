#pragma once


#include <Eigen/Eigen>

class ARLambda {
public:
    ARLambda() : squaredRatio(0) {}
    virtual ~ARLambda() {}

    Eigen::VectorXd resolve(Eigen::VectorXd &ambFloat, const Eigen::MatrixXd &ambCov);

    [[nodiscard]] bool isFixed(const double threshold = 3.0) const { return squaredRatio > threshold; }

    double squaredRatio;

protected:
    int lambda(const Eigen::VectorXd &a, const Eigen::MatrixXd &Q,
               Eigen::MatrixXd &F, Eigen::VectorXd &s, const int &m = 2);

    static int factorize(const Eigen::MatrixXd &Q, Eigen::MatrixXd &L, Eigen::VectorXd &D);

    static void gauss(Eigen::MatrixXd &L, Eigen::MatrixXd &Z, int i, int j);

    static void permute(Eigen::MatrixXd &L, Eigen::VectorXd &D, int j, double del, Eigen::MatrixXd &Z);

    static void reduction(Eigen::MatrixXd &L, Eigen::VectorXd &D, Eigen::MatrixXd &Z);

    virtual int search(Eigen::MatrixXd &L, Eigen::VectorXd &D,
                       Eigen::VectorXd &zs, Eigen::MatrixXd &zn,
                       Eigen::VectorXd &s, const int &m);
};

struct ArFixResult {
    Eigen::VectorXd z;   // 整数解（判定失败时为浮点原值）
    double ratio = 0.0;  // = ARLambda::squaredRatio
    bool fixed = false;
};

inline ArFixResult resolveWithRatio(const Eigen::VectorXd &ambFloat, const Eigen::MatrixXd &ambCov,
                                    const double ratioThreshold) {
    ARLambda ar;
    Eigen::VectorXd a = ambFloat; // resolve 取非 const 引用，此处给副本
    ArFixResult r;
    r.z = ar.resolve(a, ambCov);
    r.ratio = ar.squaredRatio;
    r.fixed = ar.isFixed(ratioThreshold);
    return r;
}

