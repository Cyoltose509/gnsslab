#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Eigen>

class SinexReader {
public:
    /// 一组坐标对应一个参考历元；同一站在缓存里可能有多组不同时段的解。
    struct Station {
        double x = 0, y = 0, z = 0;
        double vx = 0, vy = 0, vz = 0;
        bool hasVel = false;
        int refYear = 0, refDoy = 0;      // 坐标参考历元，0 表示文件未标注
        int velRefYear = 0, velRefDoy = 0; // 速度参考历元
    };
    bool readFile(const std::string &path, std::string &err);

    bool readString(const std::string &text, std::string &err);

    [[nodiscard]] bool hasStation(const std::string &name) const;

    /// 该站是否已有这一期（参考历元）的解：缓存追加前用它去重。
    [[nodiscard]] bool hasSolution(const std::string &name, int refYear, int refDoy) const;

    /// 取目标日期所在时段的站坐标：按参考历元就近选组，超出有效期返回 false。
    bool getStation(const std::string &name, int targetYear, int targetDoy,
                    Eigen::Vector3d &out, int *usedYear = nullptr, int *usedDoy = nullptr) const;

    static bool obsStationAndDOY(const std::string &obsPath, std::string &station,
                                 int &year, int &doy);

private:
    bool fail(const std::string &msg, std::string &err);
    void parseEstimate(const std::string &text);

    std::map<std::string, std::vector<Station>> stations;
};
