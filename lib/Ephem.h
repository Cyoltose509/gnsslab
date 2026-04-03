
#pragma  once
#include <TimeStruct.h>

struct GPSEphem {
    unsigned int PRN;
    double tow;
    unsigned int health;
    unsigned int IODE1;
    unsigned int IODE2;
    unsigned int week;
    unsigned int z_week;
    double toe;
    double A;
    double dn;
    double M0;
    double e;
    double omega;
    double cuc;
    double cus;
    double crc;
    double crs;
    double cic;
    double cis;
    double i0;
    double idot;
    double Omega0;
    double Omegadot;
    double iodc;
    double toc;
    double tgd;
    double a0;
    double a1;
    double a2;
    bool AS;
    double N;
    double URA;
    unsigned int crc32;
    GPSWeekSecond getWeekSecond() const {
        return {week, toe};
    };
};

struct BDSEphem {
    unsigned int PRN;
    unsigned int week; // BDS week
    double URA;
    unsigned int health;
    double tgd1; // B1
    double tgd2; // B2
    unsigned int AODC;
    unsigned int toc;
    double a0;
    double a1;
    double a2;
    unsigned int AODE;
    double toe;
    double RootA;
    double e;
    double omega;
    double dn;
    double M0;
    double Omega0;
    double Omegadot;
    double i0;
    double idot;
    double cuc;
    double cus;
    double crc;
    double crs;
    double cic;
    double cis;
    unsigned int crc32;

    BDTWeekSecond getWeekSecond() const {
        return {week, toe};
    };
};