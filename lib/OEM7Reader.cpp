#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cstring>

#include "TimeConvert.h"
#include "Ephem.h"
#include "OEM7Reader.h"

#define ID_RANGE        43
#define ID_RANGECMP     140
#define ID_GPSEPHEM     7
#define ID_BDSEPHEMRIS  1696
#define POLYCRC32 0xEDB88320u
using namespace std;

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

//计算卫星时刻与观测时刻的时间差
double TimeDiff_GPS(const CivilTime &ct, const GPSEphem &eph) {
    GPSWeekSecond g{};
    CommonTime2WeekSecond(CivilTime2CommonTime(ct), g);
    double sec = (g.week - eph.week) * 604800.0 + (g.sow - eph.toe);
    // 归一化到最近的周范围（±3.5天）
    const double halfWeek = 604800.0 / 2.0; // 302400
    if (sec > halfWeek) sec -= 604800.0;
    if (sec < -halfWeek) sec += 604800.0;
    return sec;
}

double TimeDiff_BDS(const CivilTime &ct, const BDSEphem &eph) {
    BDTWeekSecond b{};
    CommonTime2WeekSecond(CivilTime2CommonTime(ct), b);
    double sec = (b.week - eph.week) * 604800.0
                 + (b.sow - eph.toe);
    // 归一化到最近的周范围（±3.5天）
    const double halfWeek = 604800.0 / 2.0; // 302400
    if (sec > halfWeek) sec -= 604800.0;
    if (sec < -halfWeek) sec += 604800.0;
    return sec;
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

void PrintData(const RangeData &data) {
    cout << "PRN: " << data.PRN << endl;
    cout << "Pseudorange: " << data.psr << " m" << endl;
    cout << "Pseudorange Std Dev: " << data.psr_std << " m" << endl;
    cout << "Carrier Phase: " << data.adr << " cycles" << endl;
    cout << "Carrier Phase Std Dev: " << data.adr_std << " cycles" << endl;
    cout << "Doppler Shift: " << data.doppler << " Hz" << endl;
}

int OEM7Reader::readRange() const {
    int off = 28;
    const int num = I4(&buf[off]);
    off += 4;
    for (int i = 0; i < num; i++) {
        RangeData data{};
        const int status = I4(&buf[off + 40]);
        const RangeDataStatus Status = GetStatus(status);
        if (!((Status.sys == 0 || Status.sys == 4) && (Status.type == 0 || Status.type == 9))) {
            continue;
        }
        data.status = Status;
        data.PRN = U2(&buf[off + 0]);
        data.psr = R8(&buf[off + 4]);
        data.adr_std = R4(&buf[off + 12]);
        data.adr = R8(&buf[off + 16]);
        data.adr_std = R4(&buf[off + 24]);
        data.doppler = R4(&buf[off + 28]);
        off += 44;
    }
    return num;
}


OEM7Reader::Header &OEM7Reader::readHeaderData() {
    header.type = U2(&buf[4]);
    header.week = U2(&buf[14]);
    header.BodyLength = U2(&buf[8]);
    return header;
}

int crc32(const unsigned char *buff, const int len) {
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
    file = filename;
    ifs = ifstream(file, ios::binary);
    if (!ifs) {
        cerr << "Failed to open file: " << file << '\n';
        return true;
    }
    return false;
}

size_t OEM7Reader::readBytes(const size_t n) {
    buf.resize(n);
    ifs.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(n));
    const auto got = static_cast<size_t>(ifs.gcount());
    if (got < n) buf.resize(got);
    return got;
}

GPSEphem OEM7Reader::readGPSEphem() const {
    GPSEphem res{};
    int off = 28;
    res.PRN = U4(&buf[off]);
    off += 4;
    res.tow = R8(&buf[off]);
    off += 8;
    res.health = U4(&buf[off]);
    off += 4;
    res.IODE1 = U4(&buf[off]);
    off += 4;
    res.IODE2 = U4(&buf[off]);
    off += 4;
    res.week = U4(&buf[off]);
    off += 4;
    res.z_week = U4(&buf[off]);
    off += 4;
    res.toe = R8(&buf[off]);
    off += 8;
    res.A = R8(&buf[off]);
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
    res.iodc = R8(&buf[off]);
    off += 8;
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
    unsigned int asv = U4(&buf[off]);
    off += 4;
    res.AS = (asv != 0);
    res.N = R8(&buf[off]);
    off += 8;
    res.URA = R8(&buf[off]);
    off += 8;
    return res;
}

void OEM7Reader::readOne() {
    GPSWeekSecond GWS{};
    BDTWeekSecond BWS{};
    const auto CommonTime = CivilTime2CommonTime(CivilTime(2021, 11, 14, 7, 25, 0.00001788));
    CommonTime2WeekSecond(CommonTime, GWS);
    CommonTime2WeekSecond(CommonTime, BWS);

    readBytes(28); //读取消息头
    // 读取消息主体，并将它们放在buf的消息头数据之后
    const auto BodyLength = readBytes(header.BodyLength);
    buf.resize(28 + BodyLength);
    memcpy(buf.data() + 28, body.data(), BodyLength);
    if (!crcExam()) {
        cout << "CRCExam wrong" << endl;
        return;
    }
    switch (header.type) {
        case ID_RANGE:
            //readRange();
            break;
        case ID_RANGECMP:
            break;
        case ID_GPSEPHEM:
            auto GPSData = readGPSEphem();
            tk = TimeDiff_GPS(CTime, GPSData);
            if (tk < 10000) {
                cout << "G" << GPSData.PRN << " ";
                vector<double> R = GPS_P(GPSData, tk);
                for (int i = 0; i < R.size(); i++)
                    cout << std::fixed << std::setprecision(3) << R[i] << " ";
                cout << endl;
            }
            break;
        case ID_BDSEPHEMRIS:
            BDSData = BDSEphemRead(buf, header);
            tk = TimeDiff_BDS(CTime, BDSData);
            if (tk < 10000) {
                cout << "C" << BDSData.PRN << " ";
                vector<double> R = BDS_P(BDSData, tk);
                for (int i = 0; i < R.size(); i++)
                    cout << std::fixed << std::setprecision(3) << R[i] << " ";
                cout << endl;
            }
            break;
        default:
            break;
    }
}


GPSEphem GPSEphemRead(vector<unsigned char> buf, header Head) {
    int len = Head.BodyLength;
    GPSEphem res{};
    int off = 28;
    res.PRN = U4(&buf[off]);
    off += 4;
    res.tow = R8(&buf[off]);
    off += 8;
    res.health = U4(&buf[off]);
    off += 4;
    res.IODE1 = U4(&buf[off]);
    off += 4;
    res.IODE2 = U4(&buf[off]);
    off += 4;
    res.week = U4(&buf[off]);
    off += 4;
    res.z_week = U4(&buf[off]);
    off += 4;
    res.toe = R8(&buf[off]);
    off += 8;
    res.A = R8(&buf[off]);
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
    res.iodc = R8(&buf[off]);
    off += 8;
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
    unsigned int asv = U4(&buf[off]);
    off += 4;
    res.AS = (asv != 0);
    res.N = R8(&buf[off]);
    off += 8;
    res.URA = R8(&buf[off]);
    off += 8;
    return res;
}

BDSEphem BDSEphemRead(const vector<unsigned char> &buf, const header Head) {
    int len = Head.BodyLength;
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


void outputNav(ofstream &navfile, BDSEphem BDSData) {
    BDTWeekSecond ws(BDSData.week, BDSData.toe);
    CommonTime ct;
    WeekSecond2CommonTime(ws, ct);
    const auto civil = CommonTime2CivilTime(ct);
    navfile << "C" << setw(2) << setfill('0') << BDSData.PRN << ' ' << civil.year << ' ' << setw(2) << civil.month << ' '
            << setw(2) << civil.day << ' ' << setw(2) << civil.hour << ' ' << setw(2) << civil.minute << ' ' << setw(2) << civil.second;
    navfile << setfill(' ');
    cout << ws.week << " " << ws.sow << endl;
    navfile << scientific << uppercase << setprecision(12);
    double spare = 0;
    navfile << ' ' << BDSData.a0 << ' ' << BDSData.a1 << ' ' << BDSData.a2 <<
            '\n';
    navfile << ' ' << static_cast<double>(BDSData.AODE) << ' ' << BDSData.crs << ' ' << BDSData.dn << ' ' << BDSData.M0 << '\n';
    navfile << ' ' << BDSData.cuc << ' ' << BDSData.e << ' ' << BDSData.cus << ' ' << BDSData.RootA << '\n';
    navfile << ' ' << BDSData.toe << ' ' << BDSData.cic << ' ' << BDSData.Omega0 << ' ' << BDSData.cis << '\n';
    navfile << ' ' << BDSData.i0 << ' ' << BDSData.crc << ' ' << BDSData.omega << ' ' << BDSData.Omegadot << '\n';
    navfile << ' ' << BDSData.idot << ' ' << spare << ' ' << static_cast<double>(BDSData.week) << ' ' << spare << '\n';
    navfile << ' ' << BDSData.URA << ' ' << static_cast<double>(BDSData.health) << ' ' << BDSData.tgd1 << ' ' << BDSData.tgd2 << '\n';
    navfile << ' ' << static_cast<double>(BDSData.toc) << ' ' << static_cast<double>(BDSData.AODC) << '\n';

    navfile << defaultfloat << nouppercase << setprecision(6);
}
