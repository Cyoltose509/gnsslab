/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */
#include "TimeStruct.h"
#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphGPS.hpp"
#include "StringUtils.h"
#include "SP3Store.hpp"

int main(int argc,char* argv[]) {

    CivilTime civilTimePrediced;
    civilTimePrediced = CivilTime(2025, 1, 1, 0, 5, 0.0);

    CommonTime predictedTime;
    predictedTime = CivilTime2CommonTime(civilTimePrediced);

    YDSTime ydsPrediced;
    ydsPrediced = CommonTime2YDSTime(predictedTime);

    cout << "epoch:" << ydsPrediced << endl;


    string line0=
"G01 2025 01 01 00 00 00 8.645467460160E-06 3.649347490860E-11 0.000000000000E+00";
    string line1=
"     3.900000000000E+01 9.565625000000E+01 4.723053877010E-09 3.125812576130E+00";
    string line2=
"     5.071982741360E-06 2.085076412190E-04 1.234933733940E-06 5.153755249020E+03";
    string line3=
"     2.592000000000E+05 7.636845111850E-08-1.782896012340E+00 8.195638656620E-08";
    string line4=
"     9.596454288100E-01 3.528437500000E+02-1.317975728420E+00-8.442851678660E-09";
    string line5=
"     1.582208762490E-10 1.000000000000E+00 2.347000000000E+03 0.000000000000E+00";
    string line6=
"     2.000000000000E+00 6.300000000000E+01-1.396983861920E-09 3.900000000000E+01";
    string line7=
"     2.520060000000E+05 4.000000000000E+00 0.000000000000E+00 0.000000000000E+00";

    // 将上述rinex 3.05格式导航电文存储到navGPS中
    NavEphGPS gpsEph;
    
    SatID sat(line0.substr(0,3));
    
    int yr = safeStoi(line0.substr(4, 4));
    int mo = safeStoi(line0.substr(9, 2));
    int day = safeStoi(line0.substr(12, 2));
    int hr = safeStoi(line0.substr(15, 2));
    int min = safeStoi(line0.substr(18, 2));
    double sec = safeStod(line0.substr(21, 2));
    
    CivilTime cvt(yr, mo, day, hr, min, sec);
    gpsEph.CivilToc = cvt;
    gpsEph.ctToe = CivilTime2CommonTime(cvt);;
    gpsEph.ctToe.setTimeSystem(TimeSystem::GPS);

    GPSWeekSecond gws;
    CommonTime2WeekSecond(gpsEph.ctToe, gws);     // sow is system-independent
    gpsEph.Toc = gws.sow;
    gpsEph.af0 = safeStod(line0.substr(23, 19));
    gpsEph.af1 = safeStod(line0.substr(42, 19));
    gpsEph.af2 = safeStod(line0.substr(61, 19));

    ///orbit-1
    int n = 4;

    replace(line1.begin(), line1.end(), 'D', 'e');
    gpsEph.IODE = safeStod(line1.substr(n, 19));
    n += 19;
    gpsEph.Crs = safeStod(line1.substr(n, 19));
    n += 19;
    gpsEph.Delta_n = safeStod(line1.substr(n, 19));
    n += 19;
    gpsEph.M0 = safeStod(line1.substr(n, 19));
    ///orbit-2
    n = 4;
 
    replace(line2.begin(), line2.end(), 'D', 'e');
    gpsEph.Cuc = safeStod(line2.substr(n, 19));
    n += 19;
    gpsEph.ecc = safeStod(line2.substr(n, 19));
    n += 19;
    gpsEph.Cus = safeStod(line2.substr(n, 19));
    n += 19;
    gpsEph.sqrt_A = safeStod(line2.substr(n, 19));
    
    ///orbit-3
    n = 4;
    replace(line3.begin(), line3.end(), 'D', 'e');
    gpsEph.Toe = safeStod(line3.substr(n, 19));
    n += 19;
    gpsEph.Cic = safeStod(line3.substr(n, 19));
    n += 19;
    gpsEph.OMEGA_0 = safeStod(line3.substr(n, 19));
    n += 19;
    gpsEph.Cis = safeStod(line3.substr(n, 19));
    
    ///orbit-4
    n = 4;
    replace(line4.begin(), line4.end(), 'D', 'e');
    gpsEph.i0 = safeStod(line4.substr(n, 19));
    n += 19;
    gpsEph.Crc = safeStod(line4.substr(n, 19));
    n += 19;
    gpsEph.omega = safeStod(line4.substr(n, 19));
    n += 19;
    gpsEph.OMEGA_DOT = safeStod(line4.substr(n, 19));
    
    ///orbit-5
    n = 4;
    replace(line5.begin(), line5.end(), 'D', 'e');
    gpsEph.IDOT = safeStod(line5.substr(n, 19));
    n += 19;
    gpsEph.L2Codes = safeStod(line5.substr(n, 19));
    n += 19;
    gpsEph.GPSWeek = safeStod(line5.substr(n, 19));
    n += 19;
    gpsEph.L2Pflag = safeStod(line5.substr(n, 19));
    
    ///orbit-6
    n = 4;
    replace(line6.begin(), line6.end(), 'D', 'e');
    gpsEph.URA = safeStod(line6.substr(n, 19));
    n += 19;
    gpsEph.SV_health = safeStod(line6.substr(n, 19));
    n += 19;
    gpsEph.TGD = safeStod(line6.substr(n, 19));
    n += 19;
    gpsEph.IODC = safeStod(line6.substr(n, 19));
    
    ///orbit-7
    n = 4;
    replace(line7.begin(), line7.end(), 'D', 'e');
    gpsEph.HOWtime = safeStod(line7.substr(n, 19));
    n += 19;
    gpsEph.fitInterval = safeStod(line7.substr(n, 19));

    GPSWeekSecond gws2 = GPSWeekSecond(gpsEph.GPSWeek, gpsEph.Toc, TimeSystem::GPS);
    WeekSecond2CommonTime(gws2, gpsEph.ctToc);
    gpsEph.ctToc.setTimeSystem(TimeSystem::GPS);

    Xvt xvtNav = gpsEph.svXvt(predictedTime);
    cout << "nav:" << xvtNav << endl;

    // 读取精密星历，内插出位置和速度以及钟差等数值

    string sp3File = "D:\\documents\\Source\\gnssLab-2.3\\data\\COD0MGXFIN_20250010000_01D_05M_ORB.SP3";
    SP3Store sp3Store;
    sp3Store.loadSP3File(sp3File);

    Xvt xvtSP3 = sp3Store.getXvt(sat, predictedTime);
    cout << "sp3:" << xvtSP3 << endl;
    Vector3d xSP3 = xvtSP3.getPos();

    Vector3d diffXYZ = xvtNav.getPos() - xSP3;
    Vector3d diffVel = xvtNav.getVel() - xvtSP3.getVel();
    double diffClockBias = xvtNav.getClockBias() - xvtSP3.getClockBias();
    double diffRelCorr = xvtNav.getRelativityCorr() - xvtSP3.getRelativityCorr();

    cout << ydsPrediced << " \n"
         << " sat:" << sat << " \n"
         << " nav:\n" << xvtNav << " \n"
         << " sp3:\n" << xvtSP3 << " \n"
         << " diffXYZ:\n" << diffXYZ << " \n"
         << " diffVel:\n" << diffVel << " \n"
         << " diffClockBias:\n" << diffClockBias << " \n"
         << " diffRelCorr:\n" << diffRelCorr << " \n"
         << endl;

}