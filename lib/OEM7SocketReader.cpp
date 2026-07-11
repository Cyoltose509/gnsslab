#include "OEM7SocketReader.h"

bool OEM7SocketReader::connect(const std::string& ip, const unsigned short port) {
    buffer.clear();
    bufferIndex = 0;
    return socketClient.connect(ip, port);
}

void OEM7SocketReader::close() {
    socketClient.close();
    buffer.clear();
    bufferIndex = 0;
}

bool OEM7SocketReader::getNextMessage(std::vector<uint8_t>& message) {
    // 从网络接收数据追加到缓冲区末尾
    unsigned char recvBuf[MAXRAWLEN];
    if (const int lenR = socketClient.receive(recvBuf, MAXRAWLEN - 1); lenR > 0) {
        buffer.insert(buffer.end(), recvBuf, recvBuf + lenR);
    }

    while (buffer.size() - bufferIndex >= 3) {
        // 从 bufferIndex 开始扫描同步头
        size_t start = bufferIndex;
        bool found = false;
        for (; start + 2 < buffer.size(); ++start) {
            if (buffer[start] == 0xAA && buffer[start + 1] == 0x44 && buffer[start + 2] == 0x12) {
                found = true;
                break;
            }
        }

        if (!found) {
            buffer.clear();
            bufferIndex = 0;
            return false;
        }

        bufferIndex = start;

        if (buffer.size() - bufferIndex < 28) {
            return false;
        }

        const auto hlen = buffer[bufferIndex + 3];
        const auto msgLen = U2(&buffer[bufferIndex + 8]);
        const auto totalLen = hlen + msgLen + 4;

        if (buffer.size() - bufferIndex < totalLen) {
            return false;
        }

        message.assign(buffer.begin() + bufferIndex, buffer.begin() + bufferIndex + totalLen);
        bufferIndex += totalLen;

        // 定期清理已消费的前半段，防止 buffer 无限增长
        if (bufferIndex > buffer.size() / 2) {
            buffer.erase(buffer.begin(), buffer.begin() + bufferIndex);
            bufferIndex = 0;
        }

        return true;
    }
    return false;
}
