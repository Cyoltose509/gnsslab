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

    bool setReceiveTimeout(int timeoutMs) { return socketClient.setReceiveTimeout(timeoutMs); }

    bool getNextMessage(std::vector<uint8_t> &message) override;

private:
    SocketClient socketClient;
};
