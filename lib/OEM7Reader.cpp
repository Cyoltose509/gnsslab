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

#define A0 26559710//参考轨道长半径(m)
#define u_B 3.986004418e14
#define u_G 3.9860050e14//地球引力常数
#define OMEe_dt_G 7.2921151467e-5
#define OMEe_dt_B 7.2921150e-5//地球自转速率
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
    header.length = U2(&buf[8]);
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
    res.IODC = R8(&buf[off]);
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

// 解 Kepler 方程：求偏近点角 E，使用牛顿迭代
//迭代法求偏近点角Ek
double C_Ek(const double E0, const double e) {
    double E = E0;
    double E_1;
    do {
        E_1 = E;
        E = E0 + e * sin(E_1);
    } while (E_1 != E);
    return E;
}

vector<double> GPS_P(const GPSEphem &data, double tk) {
    double A = data.RootA * data.RootA; //计算轨道长半径
    double n0 = sqrt(u_G / pow(A, 3)); //平均角速度0.000145859rad/s
    double toe = data.toe; //星历参考时刻
    double delta_n = data.dn; //平均角速度修正值
    double nA = n0 + delta_n; //改正平均角速度
    double e = data.e; //偏心率
    double M0 = data.M0; //参考时刻平近点角
    double Mk = M0 + nA * tk;
    double Ek = C_Ek(Mk, e);
    double vk = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(Ek / 2)); //计算真近点角
    double omega = data.omega; //近地点角距
    double phi = vk + omega; //卫星与升交点间地心夹角
    double Cus = data.cus, Crs = data.crs, Cis = data.cis, Cuc = data.cuc, Crc = data.crc, Cic = data.cic; //卫星各周期震动的正弦项振幅
    double s2 = sin(2 * phi), c2 = cos(2 * phi);
    double Qu = Cus * s2 + Cuc * c2, Qr = Crs * s2 + Crc * c2, Qi = Cis * s2 + Cic * c2; //短周期摄动项
    double uk = phi + Qu, rk = A * (1 - e * cos(Ek)) + Qr, i0 = data.i0; //短周期摄动改正，ik的值不确定
    double ik = i0 + Qi + data.idot * tk;
    double x0 = rk * cos(uk), y0 = rk * sin(uk);
    double OME_dt = data.Omegadot, OMEk = data.Omega0 - OMEe_dt_G * toe + (OME_dt - OMEe_dt_G) * tk; //计算升交点经度
    double xk = x0 * cos(OMEk) - y0 * cos(ik) * sin(OMEk);
    double yk = x0 * sin(OMEk) + y0 * cos(ik) * cos(OMEk);
    double zk = y0 * sin(ik);
    vector<double> res{xk, yk, zk};
    double Edot = nA / (1 - e * cos(Ek)); //偏近点角速率
    double phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(Ek)); //升交角距速率
    double rdot = A * e * sin(Ek) * Edot + 2 * (Crs * c2 - Crc * s2) * phidot; //轨道半径速率
    double ukdot = phidot + 2 * phidot * (Cus * c2 - Cuc * s2);
    double OMEkdot = OME_dt - OMEe_dt_G; //升交点速率
    double ikdot = 2 * phidot * (Cis * c2 - Cic * s2) + data.idot;
    double x0dot = rdot * cos(uk) - rk * ukdot * sin(uk);
    double y0dot = rdot * sin(uk) + rk * ukdot * cos(uk); //轨道平面内速度分量
    double cosik = cos(ik), sinik = sin(ik);
    double xkdot = x0dot * cos(OMEk) - x0 * sin(OMEk) * OMEkdot
                   - (y0dot * cosik * sin(OMEk) + y0 * (-sinik * ikdot) * sin(OMEk) + y0 * cosik * cos(OMEk) * OMEkdot);
    double ykdot = x0dot * sin(OMEk) + x0 * cos(OMEk) * OMEkdot
                   + (y0dot * cosik * cos(OMEk) + y0 * (-sinik * ikdot) * cos(OMEk) - y0 * cosik * sin(OMEk) * OMEkdot);
    double zkdot = y0dot * sinik + y0 * cosik * ikdot;
    res.push_back(xkdot);
    res.push_back(ykdot);
    res.push_back(zkdot);
    return res;
}

vector<double> BDS_P(const BDSEphem &data, double tk) {
    double A = data.RootA * data.RootA; //计算轨道长半径
    double n0 = sqrt(u_B / pow(A, 3)); //平均角速度
    double toe = data.toe; //星历参考时刻
    double delta_n = data.dn; //平均角速度修正值
    double nA = n0 + delta_n; //改正平均角速度
    double e = data.e; //偏心率
    double M0 = data.M0; //参考时刻平近点角
    double Mk = M0 + nA * tk;
    double Ek = C_Ek(Mk, e);
    double vk = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(Ek / 2)); //计算真近点角
    double omega = data.omega; //近地点角距
    double phi = vk + omega; //卫星与升交点间地心夹角
    double Cus = data.cus, Crs = data.crs, Cis = data.cis, Cuc = data.cuc, Crc = data.crc, Cic = data.cic; //卫星各周期震动的正弦项振幅
    double s2 = sin(2 * phi), c2 = cos(2 * phi);
    double Qu = Cus * s2 + Cuc * c2, Qr = Crs * s2 + Crc * c2, Qi = Cis * s2 + Cic * c2; //短周期摄动项
    double uk = phi + Qu, rk = A * (1 - e * cos(Ek)) + Qr, i0 = data.i0; //短周期摄动改正，ik的值不确定
    double ik = i0 + Qi + data.idot * tk;
    double x0 = rk * cos(uk), y0 = rk * sin(uk);
    double OME_dt = data.Omegadot, OMEk = data.Omega0 - OMEe_dt_B * toe + (OME_dt - OMEe_dt_B) * tk; //计算升交点经度
    double xk = x0 * cos(OMEk) - y0 * cos(ik) * sin(OMEk);
    double yk = x0 * sin(OMEk) + y0 * cos(ik) * cos(OMEk);
    double zk = y0 * sin(ik);
    vector<double> res{xk, yk, zk};
    double Edot = nA / (1 - e * cos(Ek)); //偏近点角速率
    double phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(Ek)); //升交角距速率
    double rdot = A * e * sin(Ek) * Edot + 2 * (Crs * c2 - Crc * s2) * phidot; //轨道半径速率
    double ukdot = phidot + 2 * phidot * (Cus * c2 - Cuc * s2);
    double OMEkdot = OME_dt - OMEe_dt_G; //升交点速率
    double ikdot = 2 * phidot * (Cis * c2 - Cic * s2) + data.idot;
    double x0dot = rdot * cos(uk) - rk * ukdot * sin(uk);
    double y0dot = rdot * sin(uk) + rk * ukdot * cos(uk); //轨道平面内速度分量
    double cosik = cos(ik), sinik = sin(ik);
    double xkdot = x0dot * cos(OMEk) - x0 * sin(OMEk) * OMEkdot
                   - (y0dot * cosik * sin(OMEk) + y0 * (-sinik * ikdot) * sin(OMEk) + y0 * cosik * cos(OMEk) * OMEkdot);
    double ykdot = x0dot * sin(OMEk) + x0 * cos(OMEk) * OMEkdot
                   + (y0dot * cosik * cos(OMEk) + y0 * (-sinik * ikdot) * cos(OMEk) - y0 * cosik * sin(OMEk) * OMEkdot);
    double zkdot = y0dot * sinik + y0 * cosik * ikdot;
    res.push_back(xkdot);
    res.push_back(ykdot);
    res.push_back(zkdot);
    return res;
}

void OEM7Reader::readOne() {
    GPSWeekSecond GWS{};
    BDTWeekSecond BWS{};
    auto CommonTime = CivilTime2CommonTime(CivilTime(2021, 11, 14, 7, 25, 0.00001788));
    CommonTime2WeekSecond(CommonTime, GWS);
    convertTimeSystem(CommonTime, BWS.timeSystem);
    CommonTime2WeekSecond(CommonTime, BWS);

    readBytes(28, buf); //读取消息头
    readHeaderData();
    // 读取消息主体，并将它们放在buf的消息头数据之后
    const auto body_length = readBytes(header.length, body);
    buf.resize(28 + body_length);
    memcpy(buf.data() + 28, body.data(), body_length);
    double tk;

    if (!crcExam()) {
        throw InvalidRequest("CRCExam wrong");
    }
    switch (header.type) {
        case ID_RANGE:
            //readRange();
            break;
        case ID_RANGECMP:
            break;
        case ID_GPSEPHEM: {
            const auto GPSData = readGPSEphem();
            tk = GWS.diff(GPSData.getWeekSecond());
            if (tk < 10000) {
                cout << "G" << GPSData.PRN << " ";
                vector<double> R = GPS_P(GPSData, tk);
                for (double i: R)
                    cout << std::fixed << std::setprecision(3) << i << " ";
                cout << endl;
            }
            break;
        }
        case ID_BDSEPHEMRIS: {
            const auto BDSData = readBDSEphem();
            tk = BWS.diff(BDSData.getWeekSecond());
            if (tk < 10000) {
                cout << "C" << BDSData.PRN << " ";
                vector<double> R = BDS_P(BDSData, tk);
                for (double i: R)
                    cout << std::fixed << std::setprecision(3) << i << " ";
                cout << endl;
            }
            break;
        }
        default:
            break;
    }
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
