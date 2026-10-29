#pragma once
#include <string>
#include <vector>
#include "GnssStruct.h"
#include "core/InsStruct.h"

class AsciiLogReader {
public:
    AsciiLogReader() = default;

    [[nodiscard]] bool open(const std::string &path);

    bool getNextImu(ImuSample &s);

    const ImuLog &getImuLog() const { return mImuLog; }

    [[nodiscard]] int detectedImuType() const { return mImuType; }

    [[nodiscard]] const ImuModel *detectedImuModel() const { return findImuModel(mImuType); }

private:
    static bool parseLine(const std::string &line, ImuSample &s, int &imuType);

    std::vector<std::string> mLines;
    size_t mLineIdx = 0;
    ImuLog mImuLog;
    int mImuType = -1;
};
