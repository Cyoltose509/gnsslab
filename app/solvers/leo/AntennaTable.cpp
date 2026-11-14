#include "AntennaTable.h"

#include <string>

namespace Solvers::LEO::AntennaTable {
    namespace {
        struct Row {
            const char *mark;
            const char *antenna;
            double arpX, arpY, arpZ; // 质心→ARP
            double pcoX, pcoY, pcoZ; // ARP→相位中心
            bool hasPco;
        };

        constexpr Row kRows[] = {
            // Swarm 的正牌来源是产品自带的 `.HDR`，这两条只在头文件丢了时兜底。
            // PCO 官方从未公开
            {"SWARM A", "GPSA", 0.326, 0.0, -0.471, 0.0, 0.0, 0.0, false},
            {"SWARM C", "GPSC", 0.323, 0.0, -0.471, 0.0, 0.0, 0.0, false},
            // Sentinel-6A 的产品头给 ARP 相对 SRF 原点的位置，质心要另减一段（IDS POD context §2.2/§2.4.6）：
            // r_GP1(2474.83, 0.12, -1080.31)mm − CoM_BoS(1538, -8, 38)mm = (936.83, 8.12, -1118.31)mm。
            // PCO 在 ARF 下公布（Montenbruck 等），按 A_GP1 转到 SRF 后主要落在 −Z（天底）方向：
            // IF(L1/L2) (0.6, 0.5, 88.4)mm → (0.58, −0.50, −88.40)mm。
            {"S6A|SEN-6A|SENTINEL-6|SENTINEL6", "POD A（PODRIX）",
                0.93683, 0.00812, -1.11831,
             0.00058, -0.00050, -0.08840, true},
            // COSMIC-2 官方至今没定稿
        };

        bool matches(const char *marks, const std::string &key) {
            std::string rest(marks);
            while (!rest.empty()) {
                const std::size_t bar = rest.find('|');
                if (const std::string one = rest.substr(0, bar); !one.empty() && key.find(one) != std::string::npos) return true;
                if (bar == std::string::npos) break;
                rest.erase(0, bar + 1);
            }
            return false;
        }
    }

    std::optional<Entry> find(const std::string &key) {
        for (const auto &[mark, antenna, arpX, arpY, arpZ,
            pcoX, pcoY, pcoZ, hasPco]: kRows) {
            if (!matches(mark, key)) continue;
            return Entry{Eigen::Vector3d(arpX, arpY, arpZ),
                         Eigen::Vector3d(pcoX, pcoY, pcoZ),
                         hasPco, antenna};
        }
        return std::nullopt;
    }
}
