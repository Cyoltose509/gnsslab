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

    int off = 28; // 硬编码 28 字节偏移
    const int num = I4(&buf[off]);
    off += 4;
    for (int i = 0; i < num; i++) {
        const int status = I4(&buf[off + 40]);
        const RangeDataStatus Status = GetStatus(status);

        int freqIdx = -1;
        if (Status.sys == 0) { // GPS
            if (Status.type == 0) freqIdx = 1;      
            else if (Status.type == 9) freqIdx = 2; 
        } else if (Status.sys == 4) { // BDS
            if (Status.type == 0 || Status.type == 4) freqIdx = 2; 
            else if (Status.type == 2 || Status.type == 6) freqIdx = 6; 
        }

        if (freqIdx == -1) { off += 44; continue; }

        SatID sat;
        sat.system = (Status.sys == 0) ? "G" : "C";
        sat.id = U2(&buf[off + 0]);
        double psr = R8(&buf[off + 4]);
        double doppler = R4(&buf[off + 28]);

        obs.satTypeValueData[sat]["C" + to_string(freqIdx)] = psr;
        obs.satTypeValueData[sat]["D" + to_string(freqIdx)] = doppler;

        off += 44;
    }
    return obs;
}

void OEM7Reader::readHeaderData() {
    header.type = U2(&buf[4]);
    header.week = U2(&buf[14]);
    header.ms = I4(&buf[16]);   // 解决 0:0:0 问题的核心
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
    unsigned char crc_bytes[4];
    ifs.read(reinterpret_cast<char *>(crc_bytes), 4);
    return crc32(buf.data(), (int)buf.size()) == I4(crc_bytes);
}

bool OEM7Reader::open(const std::string &filename) {
    if (ifs.is_open()) ifs.close();
    buf.clear();

    file = filename;
    ifs = ifstream(file, ios::binary);
    return !ifs;
}

size_t OEM7Reader::readBytes(const size_t n, vector<unsigned char> &chars) {
    chars.resize(n);
    ifs.read(reinterpret_cast<char *>(chars.data()), static_cast<std::streamsize>(n));
    return static_cast<size_t>(ifs.gcount());
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
    return res;
}

std::unique_ptr<Ephemeris> OEM7Reader::readOne() {
    unsigned char head[3];
    while (true) {
        if (ifs.read((char*)head, 1).gcount() < 1) throw EndOfFile("EOF");
        if (head[0] != 0xAA) continue;
        if (ifs.read((char*)head+1, 1).gcount() < 1) throw EndOfFile("EOF");
        if (head[1] != 0x44) continue;
        if (ifs.read((char*)head+2, 1).gcount() < 1) throw EndOfFile("EOF");
        if (head[2] != 0x12) continue;

        buf.assign(28, 0);
        buf[0]=0xAA; buf[1]=0x44; buf[2]=0x12;
        ifs.read((char*)buf.data() + 3, 25);
        if (ifs.gcount() < 25) throw EndOfFile("EOF");

        readHeaderData();
        body.assign(header.length, 0);
        ifs.read((char*)body.data(), header.length);
        if (ifs.gcount() < header.length) throw EndOfFile("EOF");

        buf.insert(buf.end(), body.begin(), body.end());

        if (!crcExam()) continue;

        switch (header.type) {
            case ID_RANGE: lastObs = readRange(); hasObs = true; break;
            case ID_GPSEPHEM: hasObs = false; return std::make_unique<GPSEphem>(readGPSEphem());
            case ID_BDSEPHEMRIS: hasObs = false; return std::make_unique<BDSEphem>(readBDSEphem());
            default: hasObs = false; break;
        }
        return nullptr;
    }
}


void outputNav(ofstream &nav_file, const BDSEphem &BDSData) {
    // ... 原有逻辑 ...
}
