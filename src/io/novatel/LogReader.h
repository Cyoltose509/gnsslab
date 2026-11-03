#pragma once
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <cmath>
#include "GnssStruct.h"
#include "EphemerisTable.h"
#include "core/InsStruct.h"

namespace NovAtel {
    namespace LogID {
        constexpr std::uint16_t RANGE = 43;
        constexpr std::uint16_t RANGECMP = 140;
        constexpr std::uint16_t GPSEPHEM = 7;
        constexpr std::uint16_t BDSEPHEMERIS = 1696;
        constexpr std::uint16_t GLOEPHEM = 723;
        constexpr std::uint16_t GALEPHEM = 1122;
        constexpr std::uint16_t GALINAVEPHEM = 1309;
        constexpr std::uint16_t GALFNAVEPHEM = 1310;
        constexpr std::uint16_t BESTPOS = 42;
        constexpr std::uint16_t RAWIMU = 268;
        constexpr std::uint16_t RAWIMUS = 325;
        constexpr std::uint16_t RAWIMUX = 1461;
        constexpr std::uint16_t RAWIMUSX = 1462;
        constexpr std::uint16_t RAWIMUCOMPACT = 0x1313;
    }

    std::uint32_t crc32(const std::uint8_t *data, std::size_t len);

    /// Galileo 星历字段
    struct GalEphFields {
        int prn = 0;
        int svhE1b = 0, svhE5a = 0, svhE5b = 0;
        int dvsE1b = 0, dvsE5a = 0, dvsE5b = 0;
        int sisa = 0;
        int iodNav = 0;
        int toe = 0, toc = 0;
        double m0 = 0, deltaN = 0, ecc = 0, rootA = 0, i0 = 0, idot = 0;
        double omega0 = 0, omega = 0, omegaDot = 0;
        double cuc = 0, cus = 0, crc = 0, crs = 0, cic = 0, cis = 0;
        double af0 = 0, af1 = 0, af2 = 0;
        double bgdE5aE1 = 0, bgdE5bE1 = 0;
        bool inav = true; ///< true=I/NAV 钟差填 I/NAV 槽；false=F/NAV 钟差填 F/NAV 槽
    };


    void assembleGalBody(const GalEphFields &g, std::vector<std::uint8_t> &body);

    class LogReader {
    public:
        enum class RangeKind {
            Unknown,
            Full,
            Compressed
        };

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

        LogReader() = default;

        virtual ~LogReader() = default;

        virtual bool open(const std::string &filename) = 0;

        virtual void close();

        bool getNextEpoch(ObsData &obs, bool block = true);

        [[nodiscard]] const InsRecordLayout &insLayout() const { return mInsLayout; }

        [[nodiscard]] int detectedImuType() const { return mImuType; }

        [[nodiscard]] const ImuModel *detectedImuModel() const { return findImuModel(mImuType); }

        [[nodiscard]] const std::string &insDiagnostic() const { return mInsDiag; }

        bool getNextImu(ImuSample &s) {
            std::vector<std::uint8_t> message;
            while (getNextMessage(message, true)) {
                if (const std::size_t before = mImuLog.size(); parseImu(message) && mImuLog.size() > before) {
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

        ObsData currentObs;
        ImuLog mImuLog;
        Eigen::Vector3d antennaPosition{0, 0, 0};

        InsRecordLayout mInsLayout;
        int mImuType = -1;
        std::string mInsDiag;
        std::atomic<bool> mInsDiagLogged{false};
        RangeKind mRangeKind = RangeKind::Unknown;

        virtual bool getNextMessage(std::vector<std::uint8_t> &message, bool block) = 0;

        virtual void restartStream() = 0;

        void probeRangeKind();

        void beginEpoch();

        void emitObs(int prnRaw, int gfrq, double psr, double adr, double doppler, double cn0, std::uint32_t status);

        bool parseGNSSMessage(const std::vector<std::uint8_t> &message);

        bool parseImu(const std::vector<std::uint8_t> &message);

        bool parseRange(const std::vector<std::uint8_t> &message);

        bool parseRangeCmp(const std::vector<std::uint8_t> &message);

        bool parseGpsEphem(const std::vector<std::uint8_t> &message);

        bool parseBdsEphem(const std::vector<std::uint8_t> &message);

        bool parseGloEphem(const std::vector<std::uint8_t> &message);

        bool parseGalEph(const std::vector<std::uint8_t> &message);

        bool parseGalINavBinary(const std::vector<std::uint8_t> &message);

        bool parseGalFNavBinary(const std::vector<std::uint8_t> &message);

        bool parseBestPos(const std::vector<std::uint8_t> &message);

        static bool crcExam(const std::vector<std::uint8_t> &message);
    };
}
