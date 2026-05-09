#include "OEM7SocketReader.h"

bool OEM7SocketReader::connect(const std::string& ip, unsigned short port) {
    buffer.clear();
    return socketClient.connect(ip, port);
}

void OEM7SocketReader::close() {
    socketClient.close();
    buffer.clear();
}

bool OEM7SocketReader::getNextMessage(std::vector<uint8_t>& message) {
    // 从网络接收数据到缓冲区
    unsigned char recvBuf[MAXRAWLEN];
    int lenR = socketClient.receive(recvBuf, MAXRAWLEN - 1);
    if (lenR > 0) {
        buffer.insert(buffer.end(), recvBuf, recvBuf + lenR);
    }

    while (buffer.size() >= 3) {
        int start = 0;
        bool found = false;
        for (; start <= static_cast<int>(buffer.size()) - 3; ++start) {
            if (buffer[start] == 0xAA && buffer[start + 1] == 0x44 && buffer[start + 2] == 0x12) {
                found = true;
                break;
            }
        }

        if (!found) {
            buffer.clear();
            return false;
        }

        if (start > 0) {
            buffer.erase(buffer.begin(), buffer.begin() + start);
        }

        if (buffer.size() < 28) {
            return false;
        }

        const auto hlen = buffer[3];
        const auto msgLen = U2(&buffer[8]);
        const auto totalLen = hlen + msgLen + 4;

        if (buffer.size() < totalLen) {
            return false;
        }

        message.assign(buffer.begin(), buffer.begin() + totalLen);
        buffer.erase(buffer.begin(), buffer.begin() + totalLen);
        return true;
    }
    return false;
}
