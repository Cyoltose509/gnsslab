#pragma once
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <cmath>
#include "GnssStruct.h"
#include "EphemerisTable.h"
#include "core/InsStruct.h"

class OEM7Reader {
public:
    struct InsRecordLayout {
        bool valid = false;     // 探测是否成功
        int recLen = 56;        // 单条记录字节数
        int weekOffset = 14;    // GPS 周（u16）
        int imuTypeOffset = 13; // IMU Type（u8）
        int towMsOffset = 8;    // 整毫秒（u32）
        int secOffset = 16;     // 周内秒（double）
        int statusOffset = 24;  // IMU status（u32，低 16 位状态、高 16 位温度）
        int countsOffset = 28;  // 6 × int32 计数
        const char *name = "未识别";
    };

    OEM7Reader() = default;

    virtual ~OEM7Reader() { OEM7Reader::close(); }

    bool open(const std::string &filename);

    virtual void close();

    bool getNextEpoch(ObsData &obs, bool block = true);

    [[nodiscard]] const InsRecordLayout &insLayout() const { return mInsLayout; }

    /// 记录里读到的 IMU Type 码；-1 表示该文件没提供
    [[nodiscard]] int detectedImuType() const { return mImuType; }

    /// 按型号查当量表；返回 nullptr = 型号未知
    [[nodiscard]] const ImuModel *detectedImuModel() const { return findImuModel(mImuType); }

    /// 布局/型号识别失败的诊断信息
    [[nodiscard]] const std::string &insDiagnostic() const { return mInsDiag; }

    bool getNextImu(ImuSample &s) {
        std::vector<uint8_t> message;
        while (getNextMessage(message, true)) { // 阻塞取：文件读到尾返回 false，socket 等下一帧
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
    size_t bufferIndex = 0; // 当前读取位置
    size_t fileSize = 0; // 文件总字节数
    ObsData currentObs;
    ImuLog mImuLog; // IMU 累积缓存
    Eigen::Vector3d antennaPosition{0, 0, 0}; // 默认值

    InsRecordLayout mInsLayout;
    int mImuType = -1;
    std::string mInsDiag;
    std::atomic<bool> mInsDiagLogged{false};

    bool probeInsLayout();

    [[nodiscard]] bool acceptInsRecord(size_t start, const InsRecordLayout &layout) const;

    void probeImuTypeFromShortHeader();

    virtual bool getNextMessage(std::vector<uint8_t> &message, bool block);

    bool parseGNSSMessage(const std::vector<uint8_t> &message);

    bool parseImu(const std::vector<uint8_t> &message);

    bool parseRange(const std::vector<uint8_t> &message);

    bool parseGpsEphem(const std::vector<uint8_t> &message);

    bool parseBdsEphem(const std::vector<uint8_t> &message);

    bool parseGloEphem(const std::vector<uint8_t> &message);

    bool parseGalEph(const std::vector<uint8_t> &message);

    bool parseBestPos(const std::vector<uint8_t> &message);

    static bool crcExam(const std::vector<uint8_t> &message);
};
