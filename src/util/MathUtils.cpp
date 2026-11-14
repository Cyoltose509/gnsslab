#include "MathUtils.h"

namespace Math {
    Eigen::Matrix3d rotationX(const double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }

    Eigen::Matrix3d rotationY(const double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }

    Eigen::Matrix3d rotationZ(const double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    double angleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }
}
