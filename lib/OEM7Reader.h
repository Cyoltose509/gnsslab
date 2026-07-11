#pragma once
#include <string>
#include <vector>
#include <map>
#include "GnssStruct.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include "CoordConvert.h"

class OEM7Reader {
public:
    OEM7Reader() = default;

    virtual ~OEM7Reader() { OEM7Reader::close(); }

    bool open(const std::string &filename);

    virtual void close();

    /// 流式读取（逐历元，realtime 路径使用）
    bool getNextEpoch(ObsData &obs);

    /// 批量读取：一次性消费所有消息，产出历元列表 + 逐历元星历快照
    /// @param progress 可选进度输出（0.0~1.0），由调用方轮询
    void readAll(std::vector<ObsData> &epochs,
                 std::vector<EphemerisTable> &ephSnapshots,
                 std::atomic<float> *progress = nullptr);

    /// 文件读取进度（0.0 ~ 1.0）
    [[nodiscard]] double progress() const {
        return fileSize_ > 0 ? static_cast<double>(bufferIndex) / static_cast<double>(fileSize_) : 0.0;
    }

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
    size_t fileSize_ = 0;    // 文件总字节数（用于进度显示）
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
