#pragma once
#include <string>
#include "SocketClient.h"
#include "OEM7Reader.h"

#define MAXRAWLEN 8192

class OEM7SocketReader : public OEM7Reader {
public:
    OEM7SocketReader() = default;
    ~OEM7SocketReader() override { OEM7SocketReader::close(); }

    bool connect(const std::string& ip, unsigned short port);
    void close() override;

    bool isConnected() const { return socketClient.isConnected(); }
    bool isStale(const std::chrono::milliseconds ms) const { return socketClient.idleTooLong(ms); }

    bool setReceiveTimeout(const int timeoutMs) { return socketClient.setReceiveTimeout(timeoutMs); }

    bool getNextMessage(std::vector<uint8_t> &message, bool block = true) override;

private:
    SocketClient socketClient;

    static constexpr size_t kMaxFrameBytes = 16384;
};
