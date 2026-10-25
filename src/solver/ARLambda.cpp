
#include <iostream>
#include "ARLambda.h"
#include "MathUtils.h"



Eigen::VectorXd ARLambda::resolve(Eigen::VectorXd &ambFloat,
                           const Eigen::MatrixXd &ambCov) {
    // Check input
    if (ambFloat.size() != ambCov.rows() || ambFloat.size() != ambCov.cols()) {
        std::cout << "The dimension of input does not match." << std::endl;
        std::cout << "Cannot perform Ambiguity Resolution!" << std::endl;
        return ambFloat;
    }
    Eigen::MatrixXd F; //NOLINT
    Eigen::VectorXd S; //NOLINT
    if (lambda(ambFloat, ambCov, F, S, 2) == 0 && F.rows() == ambFloat.size()) {
        Eigen::VectorXd ambFixed = Eigen::VectorXd::Zero(ambFloat.size());
        for (int i = 0; i < ambFloat.size(); i++) {
            ambFixed(i) = F(i, 0);
        }
        squaredRatio = S(0) < 1e-12 ? 9999.9 : S(1) / S(0);
        return ambFixed;
    }
    return ambFloat; // 固定失败：返回浮点解
}



int ARLambda::factorize(const Eigen::MatrixXd &Q, Eigen::MatrixXd &L, Eigen::VectorXd &D) {
    const int n = static_cast<int>(Q.rows());
    Eigen::MatrixXd QC = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
    QC = Q;
    L = Eigen::MatrixXd::Zero(n, n);
    D = Eigen::VectorXd::Zero(n);

    for (int i = n - 1; i >= 0; i--) {
        D(i) = QC(i, i);
        if (D(i) <= 0.0) return -1;
        const double temp = std::sqrt(D(i));
        for (int j = 0; j <= i; j++) L(i, j) = QC(i, j) / temp;
        for (int j = 0; j <= i - 1; j++) {
            for (int k = 0; k <= j; k++) QC(j, k) -= L(i, k) * L(i, j);
        }
        for (int j = 0; j <= i; j++) L(i, j) /= L(i, i);
    }

    return 0;
}

void ARLambda::gauss(Eigen::MatrixXd &L, Eigen::MatrixXd &Z, const int i, const int j) {
    const auto n = static_cast<int>(L.rows());
    if (const int mu = static_cast<int>(round(L(i, j))); mu != 0) {
        for (int k = i; k < n; k++) L(k, j) -= static_cast<double>(mu) * L(k, i);
        for (int k = 0; k < n; k++) Z(k, j) -= static_cast<double>(mu) * Z(k, i);
    }
}

void ARLambda::permute(Eigen::MatrixXd &L, Eigen::VectorXd &D, const int j, const double del, Eigen::MatrixXd &Z) {
    const auto n = static_cast<int>(L.rows());
    const double eta = D(j) / del;
    const double lam = D(j + 1) * L(j + 1, j) / del;

    // *Changed D[j] to D(j) : [] may refer to row slice

    D(j) = eta * D(j + 1);
    D(j + 1) = del;
    for (int k = 0; k <= j - 1; k++) {
        const double a0 = L(j, k);
        const double a1 = L(j + 1, k);
        L(j, k) = -L(j + 1, j) * a0 + a1;
        L(j + 1, k) = eta * a0 + lam * a1;
    }
    L(j + 1, j) = lam;
    for (int k = j + 2; k < n; k++) std::swap(L(k, j), L(k, j + 1));
    for (int k = 0; k < n; k++) std::swap(Z(k, j), Z(k, j + 1));
}

void ARLambda::reduction(Eigen::MatrixXd &L, Eigen::VectorXd &D, Eigen::MatrixXd &Z) {
    const auto n = static_cast<int>(L.rows());
    int j(n - 2), k(n - 2);

    while (j >= 0) {
        if (j <= k) {
            for (int i = j + 1; i < n; i++) {
                gauss(L, Z, i, j);
            }
        }

        if (const double del = D(j) + L(j + 1, j) * L(j + 1, j) * D(j + 1); del + 1E-6 < D(j + 1)) {
            permute(L, D, j, del, Z);
            k = j;
            j = n - 2;
        } else {
            j--;
        }
    }
}

int ARLambda::search(Eigen::MatrixXd &L, Eigen::VectorXd &D, Eigen::VectorXd &zs, Eigen::MatrixXd &zn, Eigen::VectorXd &s, const int &m) {
    // TODO: CHECK UNEXPECTED INPUT
    // n - number of float parameters
    // m - number of fixed solutions
    // L - nxn
    // D - nx1
    // zs - nxn
    // zn - nxm
    // s  - m

    const int n = L.rows();

    zn = Eigen::MatrixXd::Zero(n, m);
    s = Eigen::VectorXd::Zero(m);

    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd dist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd zb = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd step = Eigen::VectorXd::Zero(n);

    int k = n - 1;
    dist[k] = 0.0;
    zb(k) = zs(k);
    z(k) = round(zb(k));
    double y = zb(k) - z(k);
    step(k) = sign(y);

    int nn(0), imax(0);
    double maxDist = 1E99;
    // 迭代上限：LAMBDA 整数搜索不保证在有限步内收敛（病态协方差/多维时可能发散）。
    // 超限必须判失败并返回 -1，让上层退回浮点解；若无此保护，search 会带着未填满的
    // zn/s 数组返回 0（"成功"），上层据此取到垃圾整数值——AR 静默给出错误固定解。
    // （2026-10-24 一次"去 using namespace Eigen"的清理误删了此检查，导致
    //  test_ar 的 ar_oneDimTight_fixed / ar_oneDimAmbiguous_notFixed / ar_twoDimTight_fixed
    //  三项回归失败：模糊度被固定成 -10 而非 10、且不该固定的场景被判为已固定。）
    // 正常结束只有一条路：else 分支里 k==n-1 的 break（搜索收敛）。循环自然跑完 = 耗尽 = 失败。
    constexpr int LOOPMAX = 10000;
    bool converged = false;
    for (int c(0); c < LOOPMAX; c++) {
        if (const double newDist = dist(k) + y * y / D(k); newDist < maxDist) {
            if (k != 0) {
                dist(--k) = newDist;
                for (int i = 0; i <= k; i++) {
                    S(k, i) = S(k + 1, i) + (z(k + 1) - zb(k + 1)) * L(k + 1, i);
                }
                zb(k) = zs(k) + S(k, k);
                z(k) = round(zb(k));
                y = zb(k) - z(k);
                step(k) = sign(y);
            } else {
                if (nn < m) {
                    if (nn == 0 || newDist > s(imax)) imax = nn;
                    for (int i = 0; i < n; i++) zn(i, nn) = z(i);
                    s(nn++) = newDist;
                } else {
                    if (newDist < s(imax)) {
                        for (int i = 0; i < n; i++) zn(i, imax) = z(i);
                        s(imax) = newDist;
                        for (int i = imax = 0; i < m; i++) if (s(imax) < s(i)) imax = i;
                    }
                    maxDist = s(imax);
                }
                z(0) += step(0);
                y = zb(0) - z(0);
                step(0) = -step(0) - sign(step(0));
            }
        } else {
            if (k == n - 1) {
                converged = true; // 搜索正常收敛：候选已全部枚举完
                break;
            }
            k++;
            z(k) += step(k);
            y = zb(k) - z(k);
            step(k) = -step(k) - sign(step(k));
        }
    }
    // 迭代耗尽 ⇒ 候选集不完整，zn/s 不可信，判失败（上层退回浮点解，不做固定）
    if (!converged) return -1;

    for (int i = 0; i < m - 1; i++) {
        for (int j = i + 1; j < m; j++) {
            if (s(i) < s(j)) continue;
            std::swap(s(i), s(j));
            for (k = 0; k < n; k++) std::swap(zn(k, i), zn(k, j));
        }
    }

    return 0;
}

int ARLambda::lambda(const Eigen::VectorXd &a, const Eigen::MatrixXd &Q, Eigen::MatrixXd &F, Eigen::VectorXd &s, const int &m) {
    if (a.size() != Q.rows() || Q.rows() != Q.cols()) return -1;
    if (m < 1) return -1;

    const int n = static_cast<int>(a.size());

    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(n, m);

    Eigen::VectorXd D = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd Z = Eigen::MatrixXd::Identity(n, n);

    // factorize 失败（协方差非正定 / D(i)<=0）必须判失败返回：否则 D 全零会让 search 里
    // y*y/D(k) 除零得 NaN，搜索路径崩坏并"固定"出错误整数值（如 -10 而非 10）。
    // （2026-10-24 一次"去 using namespace Eigen"的清理误删了此检查，导致
    //  test_ar 的 ar_oneDimTight_fixed / ar_oneDimAmbiguous_notFixed / ar_twoDimTight_fixed
    //  三项回归失败。）
    if (factorize(Q, L, D) != 0) return -1;
    reduction(L, D, Z);
    z = Z.transpose() * a;

    if (search(L, D, z, E, s, m) != 0) return -1;
    F = Z.transpose().inverse() * E;
    return 0;
}
