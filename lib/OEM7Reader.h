#pragma once
#include <string>
#include <vector>
#include <map>
#include "GnssStruct.h"
#include "Ephemeris.h"
#include "CoordConvert.h"

class OEM7Reader {
public:
    OEM7Reader() = default;

    virtual ~OEM7Reader() { OEM7Reader::close(); }

    bool open(const std::string &filename);

    virtual void close();

    bool getNextEpoch(ObsData &obs);

    std::map<int, GPSEphem> latestGps;
    std::map<int, BDSEphem> latestBds;

    // 获取解析出的天线位置
    [[nodiscard]] const Eigen::Vector3d& getAntennaPosition() const { return antennaPosition; }

protected:
    struct Header {
        int type;
        int week;
        int ms;
        int length;
        int hLen;
    } currentHeader = {};

    std::vector<unsigned char> buffer;
    size_t bufferIndex = 0;  // 当前读取位置，替代 buffer.erase 避免 O(N²) 搬迁
    ObsData currentObs;
    Eigen::Vector3d antennaPosition{0, 0, 0};  // 默认值

    virtual bool getNextMessage(std::vector<uint8_t> &message);

    bool parseMessage(const std::vector<uint8_t> &message);

    bool parseRange(const std::vector<uint8_t> &message);

    bool parseGpsEphem(const std::vector<uint8_t> &message);

    bool parseBdsEphem(const std::vector<uint8_t> &message);

    bool parseBestPos(const std::vector<uint8_t> &message);

    static bool crcExam(const std::vector<uint8_t> &message);

    // Helpers
    static uint16_t U2(const uint8_t *p) {
        uint16_t u;
        memcpy(&u, p, 2);
        return u;
    }

    static uint32_t U4(const uint8_t *p) {
        uint32_t u;
        memcpy(&u, p, 4);
        return u;
    }

    static int32_t I4(const uint8_t *p) {
        int32_t i;
        memcpy(&i, p, 4);
        return i;
    }

    static float R4(const uint8_t *p) {
        float r;
        memcpy(&r, p, 4);
        return r;
    }

    static double R8(const uint8_t *p) {
        double r;
        memcpy(&r, p, 8);
        return r;
    }
};
