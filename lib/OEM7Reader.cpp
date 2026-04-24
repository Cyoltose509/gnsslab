#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cstring>

#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphGPS.h"
#include "OEM7Reader.h"

#define ID_RANGE        43
#define ID_RANGECMP     140
#define ID_GPSEPHEM     7
#define ID_BDSEPHEMRIS  1696
#define POLYCRC32 0xEDB88320u


static unsigned short U2(const unsigned char *p) {
    unsigned short u;
    memcpy(&u, p, 2);
    return u;
}

static unsigned int U4(const unsigned char *p) {
    unsigned int u;
    memcpy(&u, p, 4);
    return u;
}

static int I4(const unsigned char *p) {
    int i;
    memcpy(&i, p, 4);
    return i;
}

static float R4(const unsigned char *p) {
    float r;
    memcpy(&r, p, 4);
    return r;
}

static double R8(const unsigned char *p) {
    double r;
    memcpy(&r, p, 8);
    return r;
}

RangeDataStatus GetStatus(const int stat) {
    RangeDataStatus res{};
    res.track = stat & 0x1F;
    res.plock = (stat >> 10) & 1;
    res.parity = (stat >> 11) & 1;
    res.clock = (stat >> 12) & 1;
    res.sys = (stat >> 16) & 7;
    res.type = (stat >> 21) & 0x1F;
    res.halfc = (stat >> 28) & 1;
    return res;
}

ObsData OEM7Reader::readRange() const {
    ObsData obs;
    const WeekSecond ws(header.week, header.ms * 0.001, TimeSystem::GPS);
    WeekSecond2CommonTime(ws, obs.epoch);

    int off = 28;
    const int num = I4(&buf[off]);
    off += 4;
    for (int i = 0; i < num; i++) {
        const int status = I4(&buf[off + 40]);
        const RangeDataStatus Status = GetStatus(status);

        int freqIdx = -1;
        if (Status.sys == 0) { // GPS
            if (Status.type == 0) freqIdx = 1;      // L1C/A -> Freq 1
            else if (Status.type == 9) freqIdx = 2; // L2P(Y) -> Freq 2
        } else if (Status.sys == 4) { // BDS
            if (Status.type == 0 || Status.type == 4) freqIdx = 2; // B1I -> Freq 2 in Const.h
            else if (Status.type == 2 || Status.type == 6) freqIdx = 6; // B3I -> Freq 6 in Const.h
        }

        if (freqIdx == -1) {
            off += 44;
            continue;
        }

        SatID sat;
        sat.system = (Status.sys == 0) ? "G" : "C";
        sat.id = U2(&buf[off + 0]);

        double psr = R8(&buf[off + 4]);
        double doppler = R4(&buf[off + 28]);

        // Use freqIdx as part of the key to distinguish observations
        string obsType = "C" + to_string(freqIdx);
        string dopType = "D" + to_string(freqIdx);

        obs.satTypeValueData[sat][obsType] = psr;
        obs.satTypeValueData[sat][dopType] = doppler;

        off += 44;
    }
    return obs;
}

void OEM7Reader::readHeaderData() {
    header.type = U2(&buf[4]);
    header.week = U2(&buf[14]);
    header.length = U2(&buf[8]);
}

inline int crc32(const unsigned char *buff, const int len) {
    unsigned int crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= buff[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ POLYCRC32;
            else
                crc >>= 1;
        }
    }
    return static_cast<int>(crc);
}

bool OEM7Reader::crcExam() {
    vector<unsigned char> crc(4);
    ifs.read(reinterpret_cast<char *>(crc.data()), 4);
    if (crc32(&buf[0], static_cast<int>(buf.size())) == I4(&crc[0]))
        return true;
    return false;
}

bool OEM7Reader::open(const std::string &filename) {
    if (ifs.is_open()) ifs.close();
    buf.clear();

    file = filename;
    ifs = ifstream(file, ios::binary);
    if (!ifs) {
        cerr << "Failed to open file: " << file << '\n';
        return true;
    }
    return false;
}

size_t OEM7Reader::readBytes(const size_t n, vector<unsigned char> &chars) {
    chars.resize(n);
    ifs.read(reinterpret_cast<char *>(chars.data()), static_cast<std::streamsize>(n));
    const auto got = static_cast<size_t>(ifs.gcount());
    if (got < n) chars.resize(got);
    return got;
}

GPSEphem OEM7Reader::readGPSEphem() const {
    GPSEphem res{};
    int off = 28;
    res.PRN = U4(&buf[off]);
    off += 4;
    //res.tow = R8(&buf[off]);
    off += 8;
    res.health = U4(&buf[off]);
    off += 4;
    res.IODE = U4(&buf[off]);
    off += 4;
    //res.IODE2 = U4(&buf[off]);
    off += 4;
    res.week = U4(&buf[off]);
    off += 4;
    //    res.z_week = U4(&buf[off]);
    off += 4;
    res.toe = R8(&buf[off]);
    off += 8;
    res.RootA = sqrt(R8(&buf[off]));
    off += 8;
    res.dn = R8(&buf[off]);
    off += 8;
    res.M0 = R8(&buf[off]);
    off += 8;
    res.e = R8(&buf[off]);
    off += 8;
    res.omega = R8(&buf[off]);
    off += 8;
    res.cuc = R8(&buf[off]);
    off += 8;
    res.cus = R8(&buf[off]);
    off += 8;
    res.crc = R8(&buf[off]);
    off += 8;
    res.crs = R8(&buf[off]);
    off += 8;
    res.cic = R8(&buf[off]);
    off += 8;
    res.cis = R8(&buf[off]);
    off += 8;
    res.i0 = R8(&buf[off]);
    off += 8;
    res.idot = R8(&buf[off]);
    off += 8;
    res.Omega0 = R8(&buf[off]);
    off += 8;
    res.Omegadot = R8(&buf[off]);
    off += 8;
    res.IODC = U4(&buf[off]);
    off += 4;
    res.toc = R8(&buf[off]);
    off += 8;
    res.tgd = R8(&buf[off]);
    off += 8;
    res.a0 = R8(&buf[off]);
    off += 8;
    res.a1 = R8(&buf[off]);
    off += 8;
    res.a2 = R8(&buf[off]);
    off += 8;
    // unsigned int asv = U4(&buf[off]);
    off += 4;
    // res.AS = (asv != 0);
    //  res.N = R8(&buf[off]);
    off += 8;
    res.URA = R8(&buf[off]);
    off += 8;
    return res;
}

BDSEphem OEM7Reader::readBDSEphem() const {
    BDSEphem res = {};
    int off = 28;
    res.PRN = U4(&buf[off]);
    off += 4;
    res.week = U4(&buf[off]);
    off += 4;
    res.URA = R8(&buf[off]);
    off += 8;
    res.health = U4(&buf[off]);
    off += 4;
    res.tgd1 = R8(&buf[off]);
    off += 8;
    res.tgd2 = R8(&buf[off]);
    off += 8;
    res.AODC = U4(&buf[off]);
    off += 4;
    res.toc = U4(&buf[off]);
    off += 4;
    res.a0 = R8(&buf[off]);
    off += 8;
    res.a1 = R8(&buf[off]);
    off += 8;
    res.a2 = R8(&buf[off]);
    off += 8;
    res.AODE = U4(&buf[off]);
    off += 4;
    res.toe = U4(&buf[off]);
    off += 4;
    res.RootA = R8(&buf[off]);
    off += 8;
    res.e = R8(&buf[off]);
    off += 8;
    res.omega = R8(&buf[off]);
    off += 8;
    res.dn = R8(&buf[off]);
    off += 8;
    res.M0 = R8(&buf[off]);
    off += 8;
    res.Omega0 = R8(&buf[off]);
    off += 8;
    res.Omegadot = R8(&buf[off]);
    off += 8;
    res.i0 = R8(&buf[off]);
    off += 8;
    res.idot = R8(&buf[off]);
    off += 8;
    res.cuc = R8(&buf[off]);
    off += 8;
    res.cus = R8(&buf[off]);
    off += 8;
    res.crc = R8(&buf[off]);
    off += 8;
    res.crs = R8(&buf[off]);
    off += 8;
    res.cic = R8(&buf[off]);
    off += 8;
    res.cis = R8(&buf[off]);
    off += 8;
    return res;
}

std::unique_ptr<Ephemeris> OEM7Reader::readOne() {
    readBytes(28, buf); //读取消息头
    readHeaderData();
    // 读取消息主体，并将它们放在buf的消息头数据之后
    const auto body_length = readBytes(header.length, body);
    buf.resize(28 + body_length);
    memcpy(buf.data() + 28, body.data(), body_length);

    if (!crcExam()) {
        throw InvalidRequest("CRCExam wrong");
    }
    switch (header.type) {
        case ID_RANGE:
            lastObs = readRange(); // 注意：readRange 内部目前硬编码了 off=28
            hasObs = true;
            break;
        case ID_GPSEPHEM: {
            hasObs = false;
            return std::make_unique<GPSEphem>(readGPSEphem());
        }
        case ID_BDSEPHEMRIS: {
            hasObs = false;
            return std::make_unique<BDSEphem>(readBDSEphem());
        }
        default:
            hasObs = false;
            break;
    }
    return nullptr;
}


void outputNav(ofstream &nav_file, const BDSEphem &BDSData) {
    const WeekSecond ws(BDSData.week, BDSData.toe, TimeSystem::BDT);
    CommonTime ct;
    WeekSecond2CommonTime(ws, ct);
    const auto civil = CommonTime2CivilTime(ct);
    nav_file << "C" << setw(2) << setfill('0') << BDSData.PRN << ' ' << civil.year << ' ' << setw(2) << civil.month << ' '
            << setw(2) << civil.day << ' ' << setw(2) << civil.hour << ' ' << setw(2) << civil.minute << ' ' << setw(2) << civil.second;
    nav_file << setfill(' ');
    cout << ws.week << " " << ws.sow << endl;
    nav_file << scientific << uppercase << setprecision(12);
    constexpr double spare = 0;
    nav_file << ' ' << BDSData.a0 << ' ' << BDSData.a1 << ' ' << BDSData.a2 << '\n';
    nav_file << ' ' << static_cast<double>(BDSData.AODE) << ' ' << BDSData.crs << ' ' << BDSData.dn << ' ' << BDSData.M0 << '\n';
    nav_file << ' ' << BDSData.cuc << ' ' << BDSData.e << ' ' << BDSData.cus << ' ' << BDSData.RootA << '\n';
    nav_file << ' ' << BDSData.toe << ' ' << BDSData.cic << ' ' << BDSData.Omega0 << ' ' << BDSData.cis << '\n';
    nav_file << ' ' << BDSData.i0 << ' ' << BDSData.crc << ' ' << BDSData.omega << ' ' << BDSData.Omegadot << '\n';
    nav_file << ' ' << BDSData.idot << ' ' << spare << ' ' << static_cast<double>(BDSData.week) << ' ' << spare << '\n';
    nav_file << ' ' << BDSData.URA << ' ' << static_cast<double>(BDSData.health) << ' ' << BDSData.tgd1 << ' ' << BDSData.tgd2 << '\n';
    nav_file << ' ' << BDSData.toc << ' ' << static_cast<double>(BDSData.AODC) << '\n';

    nav_file << defaultfloat << nouppercase << setprecision(6);
}
