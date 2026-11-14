#pragma once
#include <cstdint>
#include <mutex>
#include <string>

#include "util/Clock.h"

/// Socket 句柄类型：Windows 上是 UINT_PTR，其他平台是 int。
/// winsock2.h / windows.h 只允许出现在 SocketClient.cpp——放头文件里会把 Windows 头
/// 拖进每个 include 方，MSVC 下引发 std::byte 歧义等冲突（此前逼得调用点在
/// Realtime.cpp 顶上堆 COM_NO_WINDOWS_H / _HAS_STD_BYTE 创可贴）。
#ifdef _WIN32
using SocketType = std::uintptr_t;
inline constexpr SocketType SOCKET_INVALID = static_cast<SocketType>(~0ull); // 即 INVALID_SOCKET
#else
using SocketType = int;
inline constexpr SocketType SOCKET_INVALID = -1;
#endif

class SocketClient {
public:
    SocketClient() = default;

    ~SocketClient() {
        close();
    }

    bool connect(const std::string &ip, unsigned short port);

    void close();

    bool setReceiveTimeout(int timeoutMs);

    int receive(unsigned char *buffer, int maxLen, int timeoutMs = -1);

    [[nodiscard]] bool isConnected() const;

    [[nodiscard]] bool idleTooLong(const Clock::milliseconds ms) const {
        return Clock::over(lastRecvTime, ms);
    }

private:
    SocketType sock = SOCKET_INVALID;
    bool connected = false;
    int recvTimeoutMs = 0;
    Clock::Point lastRecvTime = Clock::now();
};
