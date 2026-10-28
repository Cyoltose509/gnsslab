#pragma once
#include <string>
#include <mutex>
#include <chrono>

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

    // 半开连接检测：TCP 对端可能悄无声息地消失（无 FIN/RST，云服务器常见），
    // 此时 recv 仅超时返回、isConnected() 仍为真，上层会永远轮询却收不到数也不会重连。
    // 因此以"距上次真正收到数据的时间"判定连接是否仍有效。
    [[nodiscard]] bool idleTooLong(std::chrono::milliseconds ms) const {
        return std::chrono::steady_clock::now() - lastRecvTime > ms;
    }

private:
    SocketType sock = SOCKET_INVALID;
    bool connected = false;
    std::chrono::steady_clock::time_point lastRecvTime = std::chrono::steady_clock::now();
};
