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

bool SocketClient::setReceiveTimeout(int timeoutMs) {
    if (sock == SOCKET_INVALID) return false;

#ifdef _WIN32
    int tv = timeoutMs;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv)) == 0;
#else
    struct timeval tv;
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
#endif
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
