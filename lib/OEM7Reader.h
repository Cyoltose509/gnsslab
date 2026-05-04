#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <unordered_map>
#include "GnssStruct.h"
#include "NavEphGPS.h"

class OEM7Reader {
public:
    OEM7Reader() = default;
    ~OEM7Reader() { close(); }

    bool open(const std::string &filename);
    void close();

    bool getNextEpoch(ObsData &obs);

    std::map<int, GPSEphem> latestGps;
    std::map<int, BDSEphem> latestBds;

private:
    struct Header {
        int type;
        int week;
        int ms;
        int length;
        int hlen;
    } currentHeader = {};

    std::vector<unsigned char> buffer_;
    size_t buffer_index_ = 0; // 改用索引，避免 $O(N^2)$ 的 erase 操作
    ObsData currentObs_;

    bool getNextMessage(std::vector<uint8_t>& message);
    bool parseMessage(const std::vector<uint8_t>& message);
    
    bool parseRange(const std::vector<uint8_t>& message);
    bool parseGpsEphem(const std::vector<uint8_t>& message);
    bool parseBdsEphem(const std::vector<uint8_t>& message);

    static unsigned int crc32_calc(const unsigned char *buff, int len);
    bool crcExam(const std::vector<uint8_t>& message);

    // Helpers
    static uint16_t U2(const uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
    static uint32_t U4(const uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
    static int32_t  I4(const uint8_t* p) { int32_t  i; memcpy(&i, p, 4); return i; }
    static float    R4(const uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }
    static double   R8(const uint8_t* p) { double   r; memcpy(&r, p, 8); return r; }
};
