#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "WS2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#endif

#include "SocketClient.h"
#include "core/Const.h"

namespace {
    void init_winsock() {
#ifdef _WIN32
        static WSADATA wsaData{};
        static std::once_flag flag;
        std::call_once(flag, [] { WSAStartup(MAKEWORD(2, 2), &wsaData); });
#endif
    }

    void socket_close(const SocketType s) {
#ifdef _WIN32
        closesocket(s);
#else
        ::close(s);
#endif
    }

    int socket_connect(const SocketType s, const sockaddr *addr, const socklen_t len) {
#ifdef _WIN32
        return connect(s, addr, len);
#else
        return ::connect(s, addr, len);
#endif
    }

    int socket_receive(const SocketType s, char *buf, const int len) {
        return recv(s, buf, len, 0);
    }
}

bool SocketClient::connect(const std::string &ip, const unsigned short port) {
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

#ifdef _WIN32
    u_long nonBlock = 1;
    ioctlsocket(sock, FIONBIO, &nonBlock);
#else
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif

    if (socket_connect(sock, reinterpret_cast<sockaddr *>(&addrSrv), sizeof(addrSrv)) < 0) {
#ifdef _WIN32
        const int err = WSAGetLastError();
#else
        const int err = errno;
#endif
        if (err != WSAEWOULDBLOCK && err != EINPROGRESS && err != EAGAIN) {
            socket_close(sock);
            sock = SOCKET_INVALID;
            return false;
        }
    }

#ifdef _WIN32
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    timeval tv{};
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    if (const int sel = select(0, nullptr, &wfds, nullptr, &tv); sel <= 0) {
        socket_close(sock);
        sock = SOCKET_INVALID;
        return false;
    }
    int errCode = 0;
    int len = sizeof(errCode);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&errCode), &len);
    if (errCode != 0) {
        socket_close(sock);
        sock = SOCKET_INVALID;
        return false;
    }

    nonBlock = 0;
    ioctlsocket(sock, FIONBIO, &nonBlock);
#else
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    const int sel = select(sock + 1, nullptr, &wfds, nullptr, &tv);
    if (sel <= 0) {
        socket_close(sock);
        sock = SOCKET_INVALID;
        return false;
    }
    int errCode = 0;
    socklen_t len = sizeof(errCode);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &errCode, &len);
    if (errCode != 0) {
        socket_close(sock);
        sock = SOCKET_INVALID;
        return false;
    }
    fcntl(sock, F_SETFL, flags);
#endif

    setReceiveTimeout(200);
    connected = true;
    lastRecvTime = Clock::now();
    return true;
}

void SocketClient::close() {
    if (sock != SOCKET_INVALID) {
        socket_close(sock);
        sock = SOCKET_INVALID;
    }
    connected = false;
}

bool SocketClient::setReceiveTimeout(const int timeoutMs) {
    if (sock == SOCKET_INVALID) return false;
    recvTimeoutMs = timeoutMs;



#ifdef _WIN32
    const int tv = timeoutMs;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&tv), sizeof(tv)) == 0;
#else
    struct timeval tv;
    tv.tv_sec = timeoutMs / MS_PER_SEC;
    tv.tv_usec = (timeoutMs % MS_PER_SEC) * MS_PER_SEC;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
#endif
}

int SocketClient::receive(unsigned char *buffer, const int maxLen, const int timeoutMs) {
    if (!connected || sock == SOCKET_INVALID) {
        return -1;
    }

    if (const int effTimeout = timeoutMs >= 0 ? timeoutMs : recvTimeoutMs; effTimeout >= 0) {
#ifdef _WIN32
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);
        timeval tv{};
        tv.tv_sec = effTimeout / 1000;
        tv.tv_usec = effTimeout % 1000 * 1000;
        if (const int sel = select(0, &rfds, nullptr, nullptr, &tv); sel <= 0) {
            return -1;
        }
#else
        pollfd pfd{};
        pfd.fd = sock;
        pfd.events = POLLIN;
        const int sel = poll(&pfd, 1, effTimeout);
        if (sel <= 0) {
            return -1;
        }
#endif
    }

    const int ret = socket_receive(sock, reinterpret_cast<char *>(buffer), maxLen);
    if (ret == 0) {
        connected = false;
    } else if (ret < 0) {
#ifdef _WIN32
        if (const int err = WSAGetLastError(); err != WSAEWOULDBLOCK && err != WSAETIMEDOUT) {
            connected = false;
        }
#else
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            connected = false;
        }
#endif
    } else {
        lastRecvTime = Clock::now();
    }
    return ret;
}

bool SocketClient::isConnected() const {
    return connected;
}
