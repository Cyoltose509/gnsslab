#pragma once
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib,"WS2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int SOCKET;
#define INVALID_SOCKET (-1)
#endif

class SocketClient {
public:
    SocketClient();
    ~SocketClient();

    bool connect(const std::string& ip, unsigned short port);
    void close();
    int receive(unsigned char* buffer, int maxLen) const;
    [[nodiscard]] bool isConnected() const;

private:
    SOCKET sock = INVALID_SOCKET;
    bool connected = false;
};
