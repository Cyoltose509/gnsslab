#pragma once

#include <Eigen/Eigen>

#include <optional>
#include <string>


namespace Solvers::LEO::AntennaTable {
    struct Entry {
        Eigen::Vector3d comToArp; // 质心→天线参考点 ARP
        Eigen::Vector3d pco;      // ARP→相位中心
        bool hasPco = false;
        std::string antenna; // 天线分支
    };

    std::optional<Entry> find(const std::string &key);
}
