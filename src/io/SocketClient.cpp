#include "SocketClient.h"
#include "Const.h"

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

    // 非阻塞连接 + select 超时：避免关闭程序时实时线程卡在 connect()（默认 TCP 重传超时可达数十秒），
    // 否则 stop 已置位也无法退出，进程残留后台。
#ifdef _WIN32
    u_long nonBlock = 1;
    ioctlsocket(sock, FIONBIO, &nonBlock);
#else
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif

    if (socket_connect(sock, reinterpret_cast<sockaddr*>(&addrSrv), sizeof(addrSrv)) < 0) {
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
    getsockopt(sock, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&errCode), &len);
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

#ifdef _WIN32
    const int tv = timeoutMs;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv)) == 0;
#else
    struct timeval tv;
    tv.tv_sec = timeoutMs / MS_PER_SEC;
    tv.tv_usec = (timeoutMs % MS_PER_SEC) * MS_PER_SEC;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
#endif
}

int SocketClient::receive(unsigned char* buffer, const int maxLen) {
    if (!connected || sock == SOCKET_INVALID) {
        return -1;
    }
    const int ret = socket_receive(sock, reinterpret_cast<char*>(buffer), maxLen);
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
