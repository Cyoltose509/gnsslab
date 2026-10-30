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
        bool valid = false;
        int recLen = 56;
        int weekOffset = 14;
        int imuTypeOffset = 13;
        int towMsOffset = 8;
        int secOffset = 16;
        int statusOffset = 24;
        int countsOffset = 28;
        const char *name = "未识别";
    };

    OEM7Reader() = default;

    virtual ~OEM7Reader() { OEM7Reader::close(); }

    bool open(const std::string &filename);

    virtual void close();

    bool getNextEpoch(ObsData &obs, bool block = true);

    [[nodiscard]] const InsRecordLayout &insLayout() const { return mInsLayout; }


    [[nodiscard]] int detectedImuType() const { return mImuType; }


    [[nodiscard]] const ImuModel *detectedImuModel() const { return findImuModel(mImuType); }


    [[nodiscard]] const std::string &insDiagnostic() const { return mInsDiag; }

    bool getNextImu(ImuSample &s) {
        std::vector<uint8_t> message;
        while (getNextMessage(message, true)) {
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
    size_t bufferIndex = 0;
    size_t fileSize = 0;
    ObsData currentObs;
    ImuLog mImuLog;
    Eigen::Vector3d antennaPosition{0, 0, 0};

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
