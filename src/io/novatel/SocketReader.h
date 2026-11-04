#pragma once
#include <string>
#include <vector>
#include "SocketClient.h"
#include "novatel/LogReader.h"

namespace NovAtel {
#define MAXRAWLEN 8192

    class SocketReader final : public LogReader {
    public:
        SocketReader() = default;

        ~SocketReader() override { SocketReader::close(); }

        bool connect(const std::string &ip, unsigned short port);

        void close() override;

        bool isConnected() const { return socketClient.isConnected(); }
        bool isStale(const std::chrono::milliseconds ms) const { return socketClient.idleTooLong(ms); }

        bool setReceiveTimeout(const int timeoutMs) { return socketClient.setReceiveTimeout(timeoutMs); }

        bool open(const std::string &) override { return false; }

    protected:
        bool getNextMessage(std::vector<std::uint8_t> &message, bool block) override;

    private:
        SocketClient socketClient;
        std::vector<unsigned char> buffer;
        std::size_t bufferIndex = 0;

        static constexpr std::size_t kMaxFrameBytes = 16384;
    };
}
