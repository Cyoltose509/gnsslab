#pragma once
#include <fstream>
#include <string>
#include <vector>

#include "novatel/LogReader.h"

namespace NovAtel {

    class AsciiReader final : public LogReader {
    public:
        AsciiReader() = default;

        ~AsciiReader() override { AsciiReader::close(); }

        bool open(const std::string &filename) override;

        void close() override;

    protected:
        bool getNextMessage(std::vector<std::uint8_t> &message, bool block) override;

        void restartStream() override;

    private:

        bool buildMessage(const std::string &line, std::vector<std::uint8_t> &message) const;

        std::ifstream mIn;
    };
}
