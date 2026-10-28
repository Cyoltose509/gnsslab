#pragma once
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include "GnssStruct.h"
#include "EphemerisTable.h"
#include "CoordConvert.h"
#include "core/InsStruct.h"

class OEM7Reader {
public:
    OEM7Reader() = default;

    virtual ~OEM7Reader() { OEM7Reader::close(); }

    bool open(const std::string &filename);

    virtual void close();

    bool getNextEpoch(ObsData &obs, bool block = true);

    bool getNextImu(ImuSample &s) {
        std::vector<uint8_t> message;
        while (getNextMessage(message)) {
            if (const size_t before = mImuLog.size(); parseImu(message) && mImuLog.size() > before) {
                s = mImuLog.back();
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] std::shared_ptr<EphemerisTable> ephSnapshot() {
        if (!mEphSnap) mEphSnap = std::make_shared<EphemerisTable>();
        return mEphSnap;
    }

    [[nodiscard]] const ImuLog &getImuLog() const { return mImuLog; }

    std::shared_ptr<EphemerisTable> mEphSnap;

    /// 收到一条（系统,prn）星历时：TOE 未变则跳过
    template<typename E>
    void storeEph(std::map<int, std::vector<std::shared_ptr<E> > > EphemerisTable::*member,
                  int prn, E &&e) {
        if (!mEphSnap) mEphSnap = std::make_shared<EphemerisTable>();
        const auto &curMap = mEphSnap.get()->*member;
        const auto it = curMap.find(prn);
        if (it != curMap.end() && !it->second.empty()) {
            if (std::fabs(it->second.back()->getCommonTime() - e.getCommonTime()) < 1e-9) return;
        }
        auto nxt = std::make_shared<EphemerisTable>(*mEphSnap);
        (nxt.get()->*member)[prn] = {std::make_shared<E>(std::forward<E>(e))};
        mEphSnap = std::move(nxt);
    }

    std::map<char, std::vector<std::string> > availableTypes;

protected:
    struct Header {
        int type;
        int week;
        int ms;
        int length;
        int hLen;
    } currentHeader = {};

    std::vector<unsigned char> buffer;
    size_t bufferIndex = 0; // 当前读取位置，替代 buffer.erase 避免 O(N²) 搬迁
    size_t fileSize = 0; // 文件总字节数
    ObsData currentObs;
    ImuLog mImuLog; // IMU 累积缓存（与 currentObs 同构：parseImu 写入、getImuLog 读出）
    Eigen::Vector3d antennaPosition{0, 0, 0}; // 默认值

    virtual bool getNextMessage(std::vector<uint8_t> &message, bool block = true);

    bool parseGNSSMessage(const std::vector<uint8_t> &message);

    bool parseImu(const std::vector<uint8_t> &message);

    bool parseRange(const std::vector<uint8_t> &message);

    bool parseGpsEphem(const std::vector<uint8_t> &message);

    bool parseBdsEphem(const std::vector<uint8_t> &message);

    bool parseGloEphem(const std::vector<uint8_t> &message);

    bool parseGalEph(const std::vector<uint8_t> &message);

    bool parseBestPos(const std::vector<uint8_t> &message);

    static bool isCompactImuRecord(const std::vector<uint8_t> &buf, size_t start);

    static bool crcExam(const std::vector<uint8_t> &message);
};
