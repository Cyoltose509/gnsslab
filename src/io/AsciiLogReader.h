#pragma once
#include <string>
#include <vector>
#include "GnssStruct.h"
#include "Ephemeris.h"
#include "core/InsStruct.h"

class AsciiLogReader {
public:
    AsciiLogReader() = default;

    [[nodiscard]] bool open(const std::string &path);

    bool getNextImu(ImuSample &s);

    const ImuLog &getImuLog() const { return mImuLog; }

private:
    static bool parseLine(const std::string &line, ImuSample &s);

    std::vector<std::string> mLines;
    size_t mLineIdx = 0;
    ImuLog mImuLog;
};
