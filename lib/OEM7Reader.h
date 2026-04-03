#pragma once
#include <fstream>
#include <string>

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
private:
    std::string file;
    std::ifstream ifs;
    vector<unsigned char> buf;
    vector<unsigned char> body;
    vector<double> vals;
public:
    struct Header {
        int type;
        int week;
        int BodyLength;
    } header = {};
    size_t readBytes(size_t n);

    GPSEphem readGPSEphem() const;

    bool crcExam();

    bool open(const std::string &filename);

    int readRange() const;

    Header& readHeaderData();
    void readOne();



};
