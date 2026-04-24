#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <NavEphGPS.h>

struct RangeDataStatus {
    int track, plock, parity, clock, sys, type, halfc;
};


struct RangeData {
    int PRN;
    double psr;
    double psr_std;
    double adr;
    double adr_std;
    double doppler;
    RangeDataStatus status;
};


class OEM7Reader {
    std::string file;
    std::ifstream ifs;
    vector<unsigned char> buf;
    vector<unsigned char> body;
    vector<double> vals;

public:
    struct Header {
        int type;
        int week;
        int ms;
        int length;
        int hlen;
    } header = {};

    size_t readBytes(size_t n, vector<unsigned char> &chars);

    GPSEphem readGPSEphem() const;

    BDSEphem readBDSEphem() const;

    bool crcExam();

    bool open(const std::string &filename);

    ObsData readRange() const;

    void readHeaderData();

    std::unique_ptr<Ephemeris> readOne();

    ObsData lastObs;
    bool hasObs = false;
};
