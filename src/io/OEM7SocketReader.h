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

    // 半开连接检测：超过 ms 未收到任何数据即视为对端已失联，应触发重连
    bool isStale(std::chrono::milliseconds ms) const { return socketClient.idleTooLong(ms); }

    bool setReceiveTimeout(int timeoutMs) { return socketClient.setReceiveTimeout(timeoutMs); }

    bool getNextMessage(std::vector<uint8_t> &message) override;

private:
    SocketClient socketClient;
};
