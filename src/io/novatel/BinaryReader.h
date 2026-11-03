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

protected:
    bool getNextMessage(std::vector<std::uint8_t> &message, bool block) override;

    void restartStream() override { bufferIndex = 0; }

private:
    bool acceptInsRecord(std::size_t start, const InsRecordLayout &layout) const;

    bool probeInsLayout();

    void probeImuTypeFromShortHeader();

    std::vector<unsigned char> buffer;
    std::size_t bufferIndex = 0;
    std::size_t fileSize = 0;
};
}
