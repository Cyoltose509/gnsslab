#include "SocketClient.h"
#include <cstring>

#ifdef _WIN32
#pragma warning(disable:4996)
#endif

SocketClient::SocketClient() = default;

SocketClient::~SocketClient() {
    close();
}

bool SocketClient::connect(const std::string& ip, unsigned short port) {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        return false;
    }
#endif

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
#ifdef _WIN32
        WSACleanup();
#endif
        return false;
    }

    SOCKADDR_IN addrSrv;
    memset(&addrSrv, 0, sizeof(addrSrv));
    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &addrSrv.sin_addr) <= 0) {
        close();
        return false;
    }

#ifdef _WIN32
    if (::connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        closesocket(sock);
        WSACleanup();
        return false;
    }
#else
    if (::connect(sock, (sockaddr*)&addrSrv, sizeof(addrSrv)) < 0) {
        close();
        return false;
    }
#endif

    connected = true;
    return true;
}

void SocketClient::close() {
#ifdef _WIN32
    if (sock != INVALID_SOCKET) {
        closesocket(sock);
        sock = INVALID_SOCKET;
    }
    WSACleanup();
#else
    if (sock != INVALID_SOCKET) {
        ::close(sock);
        sock = INVALID_SOCKET;
    }
#endif
    connected = false;
}

int SocketClient::receive(unsigned char* buffer, int maxLen) const {
    if (!connected || sock == INVALID_SOCKET) {
        return -1;
    }
#ifdef _WIN32
    return recv(sock, reinterpret_cast<char*>(buffer), maxLen, 0);
#else
    return recv(sock, buffer, maxLen, 0);
#endif
}

bool SocketClient::isConnected() const {
    return connected;
}
