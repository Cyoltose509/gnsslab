#pragma once
#include <string>
#include <vector>
#include "novatel/LogReader.h"
namespace NovAtel {

class BinaryReader final : public LogReader {
public:
    BinaryReader() = default;

    ~BinaryReader() override { BinaryReader::close(); }

    bool open(const std::string &filename) override;

    void close() override;

    bool getNextMessage(std::vector<std::uint8_t> &message, bool block) override;


private:
    std::size_t frameLengthAt(std::size_t at) const;

    void probeImuType();

    std::vector<unsigned char> buffer;
    std::size_t bufferIndex = 0;
};
}
