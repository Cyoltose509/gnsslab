#pragma once

#include <map>
#include <string>

#include <Eigen/Eigen>

class SinexReader {
public:
    struct Station {
        double x = 0, y = 0, z = 0;
        double vx = 0, vy = 0, vz = 0;
        bool hasVel = false;
        int velRefYear = 0, velRefDoy = 0;
    };
    bool readFile(const std::string &path, std::string &err);

    bool readString(const std::string &text, std::string &err);

    [[nodiscard]] bool hasStation(const std::string &name) const;

    bool getStation(const std::string &name, int targetYear, int targetDoy,
                    Eigen::Vector3d &out) const;

    static bool obsStationAndDoy(const std::string &obsPath, std::string &station,
                                 int &year, int &doy);

private:
    bool fail(const std::string &msg);
    static std::string gunzipToString(const std::string &path);
    static std::string readFileToString(const std::string &path);
    void parseEstimate(const std::string &text);

    std::map<std::string, Station> stations;
    std::string error;
};
