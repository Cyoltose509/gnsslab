#include "SocketClient.h"
#include "Const.h"

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
    timeval tv;
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
    // 恢复阻塞模式，并设置接收超时（实时流按 200ms 轮询 stop）
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
    lastRecvTime = std::chrono::steady_clock::now();
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

    // 仍尽量设置 SO_RCVTIMEO 作为次级保护；但实测在某些 Windows 环境 recv() 不会按此超时返回，
    // 因此 receive() 主动用 select/poll 做有界等待，保证最长阻塞不超过 recvTimeoutMs。
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
        timeval tv;
        tv.tv_sec = effTimeout / 1000;
        tv.tv_usec = effTimeout % 1000 * 1000;
        if (const int sel = select(0, &rfds, nullptr, nullptr, &tv); sel <= 0) {
            return -1; // 超时或异常：上层当作"本帧未就绪"，由泵决定重试
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
        lastRecvTime = std::chrono::steady_clock::now();
    }
    return ret;
}

bool SocketClient::isConnected() const {
    return connected;
}
