#pragma once
#include <string>
#include <mutex>

// ============================================================================
// 平台适配层
// ============================================================================
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)
// Winsock 惰性初始化（线程安全）
inline void init_winsock() {
#ifdef _WIN32
    static WSADATA wsaData{};
    static std::once_flag flag;
    std::call_once(flag, [] { WSAStartup(MAKEWORD(2, 2), &wsaData); });
#endif
}

using SocketType = SOCKET;
#define SOCKET_INVALID  INVALID_SOCKET

#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using SocketType = int;
#define SOCKET_INVALID  (-1)
#endif

// 统一封装的 socket 操作
inline void socket_close(SocketType s) {
#ifdef _WIN32
    closesocket(s);
#else
    ::close(s);
#endif
}

inline int socket_connect(const SocketType s, const sockaddr *addr, const socklen_t len) {
#ifdef _WIN32
    return connect(s, addr, len);
#else
    return ::connect(s, addr, len);
#endif
}

inline int socket_receive(const SocketType s, char *buf, const int len) {
    return recv(s, buf, len, 0);
}

// ============================================================================
// SocketClient
// ============================================================================
class SocketClient {
public:
    SocketClient() = default;

    ~SocketClient() {
        close();
    }

    bool connect(const std::string &ip, unsigned short port);

    void close();

    bool setReceiveTimeout(int timeoutMs);

    int receive(unsigned char *buffer, int maxLen);

    [[nodiscard]] bool isConnected() const;

private:
    SocketType sock = SOCKET_INVALID;
    bool connected = false;
};
