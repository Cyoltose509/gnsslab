#include "MathUtils.h"

namespace Math {
    vector<double> polyFit(const vector<double> &x, const vector<double> &y, const int deg) {
        const int n = static_cast<int>(x.size());
        Eigen::MatrixXd A(n, deg + 1);
        Eigen::VectorXd Y(n);
        for (int i = 0; i < n; i++) {
            double p = 1;
            for (int j = 0; j <= deg; j++) {
                A(i, j) = p;
                p *= x[i];
            }
            Y(i) = y[i];
        }
        Eigen::VectorXd c = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
        std::vector<double> r(deg + 1);
        for (int j = 0; j <= deg; j++) r[j] = c(j);
        return r;
    }

    double polyVal(const std::vector<double> &c, const double x) {
        double r = 0, p = 1;
        for (const double cc: c) {
            r += cc * p;
            p *= x;
        }
        return r;
    }

    Eigen::MatrixXd rotationMatrix(const double angle, const int axis) {
        if (axis < 1 || axis > 3) {
            throw InvalidRequest("Invalid axis (must be 1,2, or 3)");
        }

        Eigen::MatrixXd toReturn(3, 3);
        toReturn.setZero();

        const int i1 = axis - 1;
        const int i2 = (i1 + 1) % 3;
        const int i3 = (i2 + 1) % 3;

        toReturn(i1, i1) = 1.0;
        toReturn(i2, i2) = toReturn(i3, i3) = ::cos(angle);
        toReturn(i3, i2) = -(toReturn(i2, i3) = ::sin(angle));
        return toReturn;
    }

    Eigen::Matrix3d rotationX(const double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }

    Eigen::Matrix3d rotationY(const double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }

    Eigen::Matrix3d rotationZ(const double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    double frobenius(const Eigen::MatrixXd &m) {
        double sum(0);
        for (auto i = 0; i < m.rows(); i++)
            for (auto j = 0; j < m.cols(); j++)
                sum += m(i, j) * m(i, j);
        return sqrt(sum);
    }

    double magnitude(const Eigen::VectorXd &v) {
        double mag(0);
        if (v.size() == 0) return mag;
        mag = abs(v(0));

        for (auto i = 1; i < v.size(); i++) {
            if (mag > abs(v(i)))
                mag *= sqrt(1.0 + v(i) / mag * (v(i) / mag));
            else if (abs(v(i)) > mag)
                mag = abs(v(i)) * sqrt(1.0 + mag / v(i) * (mag / v(i)));
            else
                mag *= sqrt(2.0);
        }
        return mag;
    }

    double mean(const std::vector<double> &x) {
        if (x.empty()) return 0.0;
        double s = 0.0;
        for (const double v: x) s += v;
        return s / static_cast<double>(x.size());
    }

    double rms(const std::vector<double> &x) {
        if (x.size() < 2) return 0.0;
        const double m = mean(x);
        double s = 0.0;
        for (const double v: x) s += (v - m) * (v - m);
        return std::sqrt(s / static_cast<double>(x.size() - 1));
    }

    double median(std::vector<double> v) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    }

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) {
        Eigen::Matrix3d S;
        S << 0.0, -v.z(), v.y(),
                v.z(), 0.0, -v.x(),
                -v.y(), v.x(), 0.0;
        return S;
    }

    double angleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }
}
