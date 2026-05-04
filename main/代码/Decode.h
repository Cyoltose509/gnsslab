#pragma once
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include<unordered_map>
#define POLYCRC32 0xEDB88320u
using namespace std;

//数据结构 
struct SatObservation
{
    int prn;                 // 卫星编号
    int system;              // 0=GPS, 1=BeiDou
    double pseudoRange1;     // 伪距（米），频点1
    double carrierPhase1;   // 载波相位（周）
    double pseudoRange2;     // 伪距（米），频点2
    double carrierPhase2;
    double doppler;          // 多普勒频移（Hz）
    float cn0;               // 载噪比（dB-Hz）
    uint32_t trackingStatus; // 通道跟踪状态字
};

struct EpochData
{
    double tow;                     // 周内秒
    int week;                       // GPS周
    vector<SatObservation> sats;
};

// 广播星历
struct GpsEphemeris {
    int prn;
    int week;
    double tow;//时间戳
    double toe;// 星历参考时刻
    double toc;//钟差参考时刻
    // 开普勒参数和钟差参数
    double A, dN, M0, ecc, omega, cuc, cus, crc, crs, cic, cis, i0, omega0, omegaDot, idot, tgd;
    double af0, af1, af2;
    int health;
};
// 北斗广播星历（BDSEPHEMERIS）
struct BdsEphemeris
{
    int prn;            // 卫星 PRN 号
    int week;           // BDS 周数
    double ura;         // 用户测距精度（米）
    int health;         // 卫星健康标志（0=健康，1=不健康）
    double tgd1;        // B1 信号群延迟偏差（秒）
    double tgd2;        // B2 信号群延迟偏差（秒）
    int aodc;           // 钟差参数龄期
    int toc;            // 钟差参数参考时刻（秒）
    double a0;          // 钟差常数项（秒）
    double a1;          // 钟差一次项系数（秒/秒）
    double a2;          // 钟差二次项系数（秒/秒²）
    int aode;           // 星历参数龄期
    int toe;            // 星历参考时刻（秒）
    double rootA;       // 长半轴平方根（sqrt(m)）
    double e;           // 偏心率
    double omega;       // 近地点角距（弧度）
    double dn;          // 平均角速度改正量（弧度/秒）
    double M0;          // 参考时刻平近点角（弧度）
    double Omega0;      // 升交点经度（弧度）
    double OmegaDot;    // 升交点变化率（弧度/秒）
    double i0;          // 参考时刻轨道倾角（弧度）
    double idot;        // 轨道倾角变化率（弧度/秒）
    double cuc;         // 纬度幅角余弦改正项（弧度）
    double cus;         // 纬度幅角正弦改正项（弧度）
    double crc;         // 轨道半径余弦改正项（米）
    double crs;         // 轨道半径正弦改正项（米）
    double cic;         // 轨道倾角余弦改正项（弧度）
    double cis;         // 轨道倾角正弦改正项（弧度）
};
// 抽象数据源类
class DataSource {
public:
    virtual ~DataSource() = default;
    virtual bool open(const string& source) = 0;
    virtual size_t read(uint8_t* buffer, size_t size) = 0;
    virtual bool isOpen() const = 0;
    virtual void close() = 0;
};

class FileSource : public DataSource {
public:
    bool open(const string& filename) override {
        file_.open(filename, ios::binary);
        return file_.is_open();
    }
    size_t read(uint8_t* buffer, size_t size) override {
        file_.read(reinterpret_cast<char*>(buffer), size);
        return file_.gcount();
    }
    bool isOpen() const override { return file_.is_open(); }
    void close() override { file_.close(); }
private:
    ifstream file_;
};

//解码器类
class EpochReader {
public:
    EpochReader() = default;
    ~EpochReader()
    {
        close();
    }
    //打开文件并读取到内存缓冲区
    bool open(const string& filename)
    {
        ifstream file(filename, ios::binary);
        if (!file) return false;
        buffer_.clear();
        uint8_t tmp[4096];
        while (!file.eof()) {
            file.read(reinterpret_cast<char*>(tmp), sizeof(tmp));
            auto readcnt = file.gcount();
            if (readcnt > 0)
                buffer_.insert(buffer_.end(), tmp, tmp + readcnt);
        }
        file.close();
        return !buffer_.empty();
    }


    void close()
    {
        buffer_.clear();
    }
    // 获取下一历元数据，成功返回 true 并填充 epoch，失败返回 false
    bool getNextEpoch(EpochData& epoch)
    {
        while (true) {
            vector<uint8_t> message;
            if (!getNextMessage(message)) {
                //cout << "getNextEpoch: 无法获取下一条消息，退出" << endl;
                return false;
            }
            if (parseMessage(message)) {
                if (!currentEpoch_.sats.empty()) {
                    epoch = move(currentEpoch_);
                    currentEpoch_.sats.clear();
                    return true;
                }
                /*else {
                    cout << "getNextEpoch: 解析成功但历元无卫星" << endl;
                }*/
            }
            /*else
            {
                cout << "getNextEpoch: 解析失败" << endl;

            }*/
        }
    }

    vector<GpsEphemeris> gpsEphem_;
    vector<BdsEphemeris> bdsEphem_;
    // 获取 GPS 星历映射
    vector<GpsEphemeris>getGpsEphem()
    {
        return gpsEphem_;
    }

    // 获取 BDS 星历映射
    vector<BdsEphemeris> getBdsEphem()
    {
        return bdsEphem_;
    }

    unordered_map<int, GpsEphemeris> latestGps;
    unordered_map<int, BdsEphemeris> latestBds;
    unordered_map<int, GpsEphemeris> getlatestGps()
    {
        return latestGps;
    }
    unordered_map<int, BdsEphemeris> getlatestBds()
    {
        return latestBds;
    }

    //辅助函数：从字节数组解析不同类型数据（小端序）
    static unsigned short U2(const unsigned char* p) { unsigned short u; memcpy(&u, p, 2); return u; }
    static unsigned int   U4(const unsigned char* p) { unsigned int   u; memcpy(&u, p, 4); return u; }
    static float          R4(const unsigned char* p) { float          r; memcpy(&r, p, 4); return r; }
    static double         R8(const unsigned char* p) { double         r; memcpy(&r, p, 8); return r; }

    //缓冲区和当前历元数据
    vector<uint8_t> buffer_;
    EpochData currentEpoch_;

    static constexpr uint8_t SYNC_BYTE1 = 0xAA;
    static constexpr uint8_t SYNC_BYTE2 = 0x44;
    static constexpr uint8_t SYNC_BYTE3 = 0x12;

    bool getNextMessage(std::vector<uint8_t>& message)
    {
        const size_t headerLen = 28;
        while (true) {
            if (buffer_.size() < 3) return false;

            //查找同步字
            size_t start = 0;
            for (; start <= buffer_.size() - 3; ++start)
            {
                if (buffer_[start] == SYNC_BYTE1 && buffer_[start + 1] == SYNC_BYTE2 && buffer_[start + 2] == SYNC_BYTE3)
                    break;
            }
            if (start > buffer_.size() - 3)
            {
                buffer_.clear();
                return false;
            }

            if (buffer_.size() < start + headerLen)
            {
                buffer_.erase(buffer_.begin(), buffer_.begin() + start);
                continue;
            }

            const uint8_t* p = buffer_.data() + start;
            uint16_t msgLen = U2(p + 8);
            size_t totalLen = 28 + msgLen + 4;

            if (buffer_.size() < start + totalLen)
            {
                buffer_.erase(buffer_.begin(), buffer_.begin() + start + headerLen);
                continue;
            }

            message.assign(p, p + totalLen);
            buffer_.erase(buffer_.begin(), buffer_.begin() + start + totalLen);
            return true;
        }
    }

    bool parseMessage(const vector<uint8_t>& message)
    {
        const uint8_t* p = message.data();
        uint16_t msgLen = U2(p + 8);       // 消息体长度
        size_t crcCalcLen = 28 + msgLen;   // CRC计算：28字节头部 + 消息体 
        uint32_t calcCrc = crc32(p, crcCalcLen);
        uint32_t messcrc = U4(p + crcCalcLen);
        if (calcCrc != messcrc)
        {
            //cout << "crc false"<< endl;
            return false;
        }

        uint16_t msgId = U2(message.data() + 4);   // 消息ID偏移4
        size_t headerLen = message[3];
        //cout << "parseMessage: 消息ID=" << msgId << " headerLen=" << (int)headerLen << endl;

        if (msgId == 43)
        {
            return parseRange(message, headerLen);
        }
        if (msgId == 7)
        {
            return parseGpsEphem(message, headerLen);
        }
        if (msgId == 1696)
        {
            return parseBdsEphem(message, headerLen);
        }
        return false;
    }
    //解析伪距观测数据
    bool parseRange(const vector<uint8_t>& data, size_t headerLen)
    {

        const uint8_t* p = data.data() + headerLen;
        uint32_t numObs = U4(p); p += 4;
        int week = U2(data.data() + 14);
        uint32_t ms = U4(data.data() + 16);
        double tow = static_cast<double>(ms) / 1000.0;

        currentEpoch_.tow = tow;
        currentEpoch_.week = week;
        currentEpoch_.sats.clear();
        currentEpoch_.sats.reserve(numObs);

        for (uint32_t i = 0; i < numObs; ++i)
        {
            SatObservation sat;
            sat.prn = U2(p); p += 2;
            p += 2;                           // GLONASS 频率跳过
            sat.pseudoRange1 = R8(p); p += 8;
            p += 4;                           // 伪距标准差跳过
            sat.carrierPhase1 = R8(p); p += 8;
            p += 4;                           // 载波相位标准差跳过
            sat.doppler = R4(p); p += 4;
            sat.cn0 = R4(p); p += 4;
            p += 4;                           // 跟踪时间跳过
            sat.trackingStatus = U4(p); p += 4;
            sat.system = (sat.trackingStatus >> 16) & 0x7;
            sat.pseudoRange2 = 0.0;
            sat.carrierPhase2 = 0.0;
            currentEpoch_.sats.push_back(sat);
        }
        return true;
    }
    //解析 GPS 星历
    bool parseGpsEphem(const std::vector<uint8_t>& data, size_t headerLen)
    {
        const uint8_t* p = data.data() + headerLen;

        GpsEphemeris eph;
        eph.prn = U4(p) & 0xFF; p += 4;   // PRN
        eph.tow = R8(p); p += 8;       // 时间戳
        eph.health = U4(p); p += 4;       // 健康状态
        uint32_t iode1 = U4(p); p += 4;   // IODE
        uint32_t iode2 = U4(p); p += 4;   // IODE
        eph.week = U4(p); p += 4;
        p += 4;                           // zWeek跳过
        eph.toe = R8(p); p += 8;
        eph.A = R8(p); p += 8;
        eph.dN = R8(p); p += 8;
        eph.M0 = R8(p); p += 8;
        eph.ecc = R8(p); p += 8;
        eph.omega = R8(p); p += 8;
        eph.cuc = R8(p); p += 8;
        eph.cus = R8(p); p += 8;
        eph.crc = R8(p); p += 8;
        eph.crs = R8(p); p += 8;
        eph.cic = R8(p); p += 8;
        eph.cis = R8(p); p += 8;
        eph.i0 = R8(p); p += 8;
        eph.idot = R8(p); p += 8;
        eph.omega0 = R8(p); p += 8;
        eph.omegaDot = R8(p); p += 12;

        eph.toc = R8(p); p += 8;
        eph.tgd = R8(p);p += 8;
        eph.af0 = R8(p); p += 8;
        eph.af1 = R8(p); p += 8;
        eph.af2 = R8(p); p += 8;

        gpsEphem_.push_back(eph);
        latestGps[eph.prn]=eph;
        return true;
    }
    //解析北斗星历
    bool parseBdsEphem(const std::vector<uint8_t>& data, size_t headerLen)
    {
        const uint8_t* p = data.data() + headerLen;

        BdsEphemeris eph;
        eph.prn = U4(p) & 0xFF; p += 4;   // PRN
        eph.week = U4(p);        p += 4;   // week
        eph.ura = R8(p);        p += 8;   // URA
        eph.health = U4(p);        p += 4;   // health
        eph.tgd1 = R8(p);        p += 8;   // tgd1
        eph.tgd2 = R8(p);        p += 8;   // tgd2
        eph.aodc = U4(p);        p += 4;   // AODC

        eph.toc = U4(p);        p += 4;   // toc
        eph.a0 = R8(p);        p += 8;   // a0
        eph.a1 = R8(p);        p += 8;   // a1
        eph.a2 = R8(p);        p += 8;   // a2
        eph.aode = U4(p);        p += 4;   // AODE
        eph.toe = U4(p);        p += 4;   // toe
        eph.rootA = R8(p);        p += 8;   // RootA
        eph.e = R8(p);        p += 8;   // e
        eph.omega = R8(p);        p += 8;   // ω
        eph.dn = R8(p);        p += 8;   // dn
        eph.M0 = R8(p);        p += 8;   // M0
        eph.Omega0 = R8(p);        p += 8;   // Ω0
        eph.OmegaDot = R8(p);      p += 8;   // Ωdot
        eph.i0 = R8(p);        p += 8;   // i0
        eph.idot = R8(p);        p += 8;   // idot
        eph.cuc = R8(p);        p += 8;   // cuc
        eph.cus = R8(p);        p += 8;   // cus
        eph.crc = R8(p);        p += 8;   // crc
        eph.crs = R8(p);        p += 8;   // crs
        eph.cic = R8(p);        p += 8;   // cic
        eph.cis = R8(p);        p += 8;   // cis

        latestBds[eph.prn]=eph;
        bdsEphem_.push_back(eph);
        return true;

    }
    //crc校验函数
    unsigned int crc32(const unsigned char* buff, int len)
    {
        int i, j;
        unsigned int crc = 0;
        for (i = 0; i < len; i++)
        {
            crc ^= buff[i];
            for (j = 0; j < 8; j++)
            {
                if (crc & 1)
                {
                    crc = (crc >> 1) ^ POLYCRC32;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }
};

class EpochReader_double
{ 
public:
    EpochReader_double() = default;
    ~EpochReader_double()
    {
        close();
    }
    //打开文件并读取到内存缓冲区
    bool open(const string& filename)
    {
        ifstream file(filename, ios::binary);
        if (!file) return false;
        buffer_.clear();
        uint8_t tmp[4096];
        while (!file.eof()) {
            file.read(reinterpret_cast<char*>(tmp), sizeof(tmp));
            auto readcnt = file.gcount();
            if (readcnt > 0)
                buffer_.insert(buffer_.end(), tmp, tmp + readcnt);
        }
        file.close();
        return !buffer_.empty();
    }


    void close()
    {
        buffer_.clear();
    }
    // 获取下一历元数据，成功返回 true 并填充 epoch，失败返回 false
    bool getNextEpoch(EpochData& epoch)
    {
        while (true) {
            vector<uint8_t> message;
            if (!getNextMessage(message)) {
                //cout << "getNextEpoch: 无法获取下一条消息，退出" << endl;
                return false;
            }
            if (parseMessage(message)) {
                if (!currentEpoch_.sats.empty()) {
                    epoch = move(currentEpoch_);
                    currentEpoch_.sats.clear();
                    return true;
                }
                /*else {
                    cout << "getNextEpoch: 解析成功但历元无卫星" << endl;
                }*/
            }
            /*else
            {
                cout << "getNextEpoch: 解析失败" << endl;

            }*/
        }
    }

    vector<GpsEphemeris> gpsEphem_;
    vector<BdsEphemeris> bdsEphem_;
    // 获取 GPS 星历映射
    vector<GpsEphemeris>getGpsEphem()
    {
        return gpsEphem_;
    }

    // 获取 BDS 星历映射
    vector<BdsEphemeris> getBdsEphem()
    {
        return bdsEphem_;
    }

    unordered_map<int, GpsEphemeris> latestGps;
    unordered_map<int, BdsEphemeris> latestBds;
    unordered_map<int, GpsEphemeris> getlatestGps()
    {
        return latestGps;
    }
    unordered_map<int, BdsEphemeris> getlatestBds()
    {
        return latestBds;
    }

    //辅助函数：从字节数组解析不同类型数据（小端序）
    static unsigned short U2(const unsigned char* p) { unsigned short u; memcpy(&u, p, 2); return u; }
    static unsigned int   U4(const unsigned char* p) { unsigned int   u; memcpy(&u, p, 4); return u; }
    static float          R4(const unsigned char* p) { float          r; memcpy(&r, p, 4); return r; }
    static double         R8(const unsigned char* p) { double         r; memcpy(&r, p, 8); return r; }

    //缓冲区和当前历元数据
    vector<uint8_t> buffer_;
    EpochData currentEpoch_;

    static constexpr uint8_t SYNC_BYTE1 = 0xAA;
    static constexpr uint8_t SYNC_BYTE2 = 0x44;
    static constexpr uint8_t SYNC_BYTE3 = 0x12;

    bool getNextMessage(std::vector<uint8_t>& message)
    {
        const size_t headerLen = 28;
        while (true) {
            if (buffer_.size() < 3) return false;

            //查找同步字
            size_t start = 0;
            for (; start <= buffer_.size() - 3; ++start)
            {
                if (buffer_[start] == SYNC_BYTE1 && buffer_[start + 1] == SYNC_BYTE2 && buffer_[start + 2] == SYNC_BYTE3)
                    break;
            }
            if (start > buffer_.size() - 3)
            {
                buffer_.clear();
                return false;
            }

            if (buffer_.size() < start + headerLen)
            {
                buffer_.erase(buffer_.begin(), buffer_.begin() + start);
                continue;
            }

            const uint8_t* p = buffer_.data() + start;
            uint16_t msgLen = U2(p + 8);
            size_t totalLen = 28 + msgLen + 4;

            if (buffer_.size() < start + totalLen)
            {
                buffer_.erase(buffer_.begin(), buffer_.begin() + start + headerLen);
                continue;
            }

            message.assign(p, p + totalLen);
            buffer_.erase(buffer_.begin(), buffer_.begin() + start + totalLen);
            return true;
        }
    }

    bool parseMessage(const vector<uint8_t>& message)
    {
        const uint8_t* p = message.data();
        uint16_t msgLen = U2(p + 8);       // 消息体长度
        size_t crcCalcLen = 28 + msgLen;   // CRC计算：28字节头部 + 消息体 
        uint32_t calcCrc = crc32(p, crcCalcLen);
        uint32_t messcrc = U4(p + crcCalcLen);
        if (calcCrc != messcrc)
        {
            //cout << "crc false"<< endl;
            return false;
        }

        uint16_t msgId = U2(message.data() + 4);   // 消息ID偏移4
        size_t headerLen = message[3];
        //cout << "parseMessage: 消息ID=" << msgId << " headerLen=" << (int)headerLen << endl;

        if (msgId == 43)
        {
            return parseRange(message, headerLen);
        }
        if (msgId == 7)
        {
            return parseGpsEphem(message, headerLen);
        }
        if (msgId == 1696)
        {
            return parseBdsEphem(message, headerLen);
        }
        return false;
    }
    //解析伪距观测数据
    bool parseRange(const vector<uint8_t>& data, size_t headerLen)
    {
        const uint8_t* p = data.data() + headerLen;
        uint32_t numObs = U4(p); p += 4;
        int week = U2(data.data() + 14);
        uint32_t ms = U4(data.data() + 16);
        double tow = static_cast<double>(ms) / 1000.0;

        currentEpoch_.tow = tow;
        currentEpoch_.week = week;
        currentEpoch_.sats.clear();

        std::unordered_map<int, SatObservation> satMap; // 局部化，避免历元间残留

        for (uint32_t i = 0; i < numObs; ++i) 
        {
            uint16_t prn = U2(p); p += 2;
            p += 2; // GLONASS 频率跳过

            double psr = R8(p); p += 8;
            p += 4; // 伪距标准差跳过
            double adr = R8(p); p += 8;
            p += 4; // 载波相位标准差跳过
            float doppler = R4(p); p += 4;
            float cn0 = R4(p); p += 4;
            p += 4; // 跟踪时间跳过
            uint32_t trackingStatus = U4(p); p += 4;

            int system = (trackingStatus >> 16) & 0x07;
            int sigType = (trackingStatus >> 21) & 0x1F;

           

            if (system != 0 && system != 4) continue; // 仅处理 GPS 和 BDS

            bool isFreq1 = false, isFreq2 = false;
            //cout << system << " " << sigType << endl;
            if (system == 0) 
            { // GPS
                // L1: C/A(0), L2: P(5), P(Y)(9), L2C(17)
                if (sigType == 0||sigType == 16) isFreq1 = true;
                else if (sigType == 1||sigType == 5||sigType == 9||sigType == 17) isFreq2 = true;
            }
            else if (system == 4) 
            { // BDS
                // B1I: 0(D1) / 4(D2)    B2I: 1(D1) / 5(D2)
                if (sigType == 0 || sigType == 4) isFreq1 = true;
                else if (sigType == 2 || sigType == 6) isFreq2 = true;
            }

            if (!isFreq1 && !isFreq2) continue;
            int key = system * 1000 + prn;
            SatObservation& sat = satMap[key];

            sat.prn = prn;
            sat.system = system;
            sat.trackingStatus = trackingStatus;

            if (isFreq1) 
            {
                sat.pseudoRange1 = psr;
                sat.carrierPhase1 = adr;
            }
            else if (isFreq2) 
            {
                sat.pseudoRange2 = psr;
                sat.carrierPhase2 = adr;
            }
            sat.doppler = doppler;
            sat.cn0 = cn0;
        }

        // 仅保留双频完整的卫星
        for (auto& kv : satMap) 
        {
            SatObservation& sat = kv.second;
            if (sat.pseudoRange1 != 0.0 && sat.pseudoRange2 != 0.0) 
            {
                currentEpoch_.sats.push_back(sat);
            }
        }

        return !currentEpoch_.sats.empty();
    }
    //解析 GPS 星历
    bool parseGpsEphem(const std::vector<uint8_t>& data, size_t headerLen)
    {
        const uint8_t* p = data.data() + headerLen;

        GpsEphemeris eph;
        eph.prn = U4(p) & 0xFF; p += 4;   // PRN
        eph.tow = R8(p); p += 8;       // 时间戳
        eph.health = U4(p); p += 4;       // 健康状态
        uint32_t iode1 = U4(p); p += 4;   // IODE
        uint32_t iode2 = U4(p); p += 4;   // IODE
        eph.week = U4(p); p += 4;
        p += 4;                           // zWeek跳过
        eph.toe = R8(p); p += 8;
        eph.A = R8(p); p += 8;
        eph.dN = R8(p); p += 8;
        eph.M0 = R8(p); p += 8;
        eph.ecc = R8(p); p += 8;
        eph.omega = R8(p); p += 8;
        eph.cuc = R8(p); p += 8;
        eph.cus = R8(p); p += 8;
        eph.crc = R8(p); p += 8;
        eph.crs = R8(p); p += 8;
        eph.cic = R8(p); p += 8;
        eph.cis = R8(p); p += 8;
        eph.i0 = R8(p); p += 8;
        eph.idot = R8(p); p += 8;
        eph.omega0 = R8(p); p += 8;
        eph.omegaDot = R8(p); p += 12;

        eph.toc = R8(p); p += 8;
        eph.tgd = R8(p);p += 8;
        eph.af0 = R8(p); p += 8;
        eph.af1 = R8(p); p += 8;
        eph.af2 = R8(p); p += 8;

        gpsEphem_.push_back(eph);
        latestGps[eph.prn] = eph;
        return true;
    }
    //解析北斗星历
    bool parseBdsEphem(const std::vector<uint8_t>& data, size_t headerLen)
    {
        const uint8_t* p = data.data() + headerLen;

        BdsEphemeris eph;
        eph.prn = U4(p) & 0xFF; p += 4;   // PRN
        eph.week = U4(p);        p += 4;   // week
        eph.ura = R8(p);        p += 8;   // URA
        eph.health = U4(p);        p += 4;   // health
        eph.tgd1 = R8(p);        p += 8;   // tgd1
        eph.tgd2 = R8(p);        p += 8;   // tgd2
        eph.aodc = U4(p);        p += 4;   // AODC

        eph.toc = U4(p);        p += 4;   // toc
        eph.a0 = R8(p);        p += 8;   // a0
        eph.a1 = R8(p);        p += 8;   // a1
        eph.a2 = R8(p);        p += 8;   // a2
        eph.aode = U4(p);        p += 4;   // AODE
        eph.toe = U4(p);        p += 4;   // toe
        eph.rootA = R8(p);        p += 8;   // RootA
        eph.e = R8(p);        p += 8;   // e
        eph.omega = R8(p);        p += 8;   // ω
        eph.dn = R8(p);        p += 8;   // dn
        eph.M0 = R8(p);        p += 8;   // M0
        eph.Omega0 = R8(p);        p += 8;   // Ω0
        eph.OmegaDot = R8(p);      p += 8;   // Ωdot
        eph.i0 = R8(p);        p += 8;   // i0
        eph.idot = R8(p);        p += 8;   // idot
        eph.cuc = R8(p);        p += 8;   // cuc
        eph.cus = R8(p);        p += 8;   // cus
        eph.crc = R8(p);        p += 8;   // crc
        eph.crs = R8(p);        p += 8;   // crs
        eph.cic = R8(p);        p += 8;   // cic
        eph.cis = R8(p);        p += 8;   // cis

        latestBds[eph.prn] = eph;
        bdsEphem_.push_back(eph);
        return true;

    }
    //crc校验函数
    unsigned int crc32(const unsigned char* buff, int len)
    {
        int i, j;
        unsigned int crc = 0;
        for (i = 0; i < len; i++)
        {
            crc ^= buff[i];
            for (j = 0; j < 8; j++)
            {
                if (crc & 1)
                {
                    crc = (crc >> 1) ^ POLYCRC32;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

};