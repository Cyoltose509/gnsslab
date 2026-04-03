#include <sstream>
#include<cmath>

#include"OEM7Reader.cpp"

#define A0 26559710//参考轨道长半径(m)
#define u_B 3.986004418e14
#define u_G 3.9860050e14//地球引力常数
#define OMEe_dt_G 7.2921151467e-5
#define OMEe_dt_B 7.2921150e-5//地球自转速率


// 解 Kepler 方程：求偏近点角 E，使用牛顿迭代
//迭代法求偏近点角Ek
double C_Ek(double E0, double e) {
    double E = E0;
    double E_1;
    do {
        E_1 = E;
        E = E0 + e * sin(E_1);
    } while (E_1 != E);
    return E;
}

// 计算 GPS 卫星速度（ECEF）

//计算卫星位置
vector<double> GPS_P(GPSEphem data, double tk) {
    double n0 = sqrt(u_G / pow(data.A, 3)); //平均角速度0.000145859rad/s
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
    double uk = phi + Qu, rk = data.A * (1 - e * cos(Ek)) + Qr, i0 = data.i0; //短周期摄动改正，ik的值不确定
    double ik = i0 + Qi + data.idot * tk;
    double x0 = rk * cos(uk), y0 = rk * sin(uk);
    double OME_dt = data.Omegadot, OMEk = data.Omega0 - OMEe_dt_G * toe + (OME_dt - OMEe_dt_G) * tk; //计算升交点经度
    double xk = x0 * cos(OMEk) - y0 * cos(ik) * sin(OMEk);
    double yk = x0 * sin(OMEk) + y0 * cos(ik) * cos(OMEk);
    double zk = y0 * sin(ik);
    vector<double> res{xk, yk, zk};
    double Edot = nA / (1 - e * cos(Ek)); //偏近点角速率
    double phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(Ek)); //升交角距速率
    double rdot = data.A * e * sin(Ek) * Edot + 2 * (Crs * c2 - Crc * s2) * phidot; //轨道半径速率
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

vector<double> BDS_P(BDSEphem data, double tk) {
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

//输出坐标
void show_position(vector<double> p) {
    cout << "x:" << std::fixed << std::setprecision(3) << p[0] << " ";
    cout << "y:" << std::fixed << std::setprecision(3) << p[1] << " ";
    cout << "z:" << std::fixed << std::setprecision(3) << p[2] << endl;
}


int main() {
    string filename = "NovatelOEM20211114-01.log";

    // 以二进制方式打开文件用于读取
    ifstream datafile(filename, ios::binary);
    if (!datafile) {
        cerr << "Failed to open file: " << filename << '\n';
        return 1;
    }
    ofstream navfile("GPSdata.txt");


    header Head{};
    GPSEphem GPSData{};
    BDSEphem BDSData{};
    CivilTime CTime;
    CTime.year = 2021;
    CTime.month = 11;
    CTime.day = 14;
    CTime.hour = 7;
    CTime.minute = 25;
    CTime.second = 0.00001788;
    size_t BodyLength;
    vector<unsigned char> buf;
    vector<unsigned char> body;
    vector<double> vals;
    double tk;
    const int colw = 23;
    //while (datafile.peek() != std::char_traits<char>::eof()) {
    for (int i = 0; i < 100; i++) {
        ReadBytes(datafile, buf, 28); //读取消息头
        Head = HeaderData(buf);
        // 读取消息主体，并将它们放在buf的消息头数据之后
        BodyLength = ReadBytes(datafile, body, Head.BodyLength);
        buf.resize(28 + BodyLength);
        memcpy(buf.data() + 28, body.data(), BodyLength);
        if (!crcExam(datafile, buf)) {
            cout << "CRCExam wrong" << endl;
            continue;
        }
        switch (Head.type) {
            case ID_RANGE:
                //cout << "range ";
                RangeRead(buf, Head);
                break;
            case ID_RANGECMP:
                break;
            case ID_GPSEPHEM:
                GPSData = GPSEphemRead(buf, Head);
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
                BDSData = BDSEphemRead(buf, Head);
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
    return 0;
}
