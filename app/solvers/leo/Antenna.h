#pragma once

#include <Eigen/Eigen>

#include <string>


namespace Solvers::LEO::Antenna {
    struct Segment {
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        bool known = false;
        std::string from;
    };

    struct Correction {
        Segment comToArp;
        Segment pco;

        [[nodiscard]] Eigen::Vector3d total() const { return comToArp.xyz + pco.xyz; }

        [[nodiscard]] bool any() const { return comToArp.known || pco.known; }
    };

    Correction fromHeader(const std::string &obsPath);

    Correction fromHeaderFile(const std::string &path);

    Correction defaults(const std::string &obsPath);


    Correction merge(const Correction &user, const Correction &def);

    std::string missionKey(const std::string &obsPath);
}
