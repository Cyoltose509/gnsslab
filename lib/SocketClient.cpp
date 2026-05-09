#include "SocketClient.h"

bool SocketClient::connect(const std::string& ip, const unsigned short port) {
    init_winsock();

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == SOCKET_INVALID) {
        return false;
    }

    sockaddr_in addrSrv = {};
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &addrSrv.sin_addr) <= 0) {
        socket_close(sock);
        sock = SOCKET_INVALID;
        return false;
    }

    if (socket_connect(sock, reinterpret_cast<sockaddr*>(&addrSrv), sizeof(addrSrv)) < 0) {
        socket_close(sock);
        sock = SOCKET_INVALID;
        return false;
    }

    connected = true;
    return true;
}

void SocketClient::close() {
    if (sock != SOCKET_INVALID) {
        socket_close(sock);
        sock = SOCKET_INVALID;
    }
    connected = false;
}

int SocketClient::receive(unsigned char* buffer, const int maxLen) const {
    if (!connected || sock == SOCKET_INVALID) {
        return -1;
    }
    return socket_receive(sock, reinterpret_cast<char*>(buffer), maxLen);
}

bool SocketClient::isConnected() const {
    return connected;
}
