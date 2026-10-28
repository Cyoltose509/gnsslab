#include "OEM7SocketReader.h"

#include "MathUtils.h"

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
            // 没有同步头：仅丢弃不可能成为同步头前缀的尾部（最多保留最后 2 字节，
            // 即可能存在的 0xAA / 0xAA 0x44 前缀），避免把真实同步头的前缀误删导致后续丢帧。
            const size_t remaining = buffer.size() - bufferIndex;
            const size_t keep = remaining > 2 ? 2 : remaining;
            buffer.erase(buffer.begin() + static_cast<long long>(bufferIndex),
                         buffer.end() - static_cast<long long>(keep));
            bufferIndex = 0;
            return false;
        }

        bufferIndex = start;

        if (buffer.size() - bufferIndex < 28) {
            return false;   // 头部未齐，等下一轮接收
        }

        const auto hlen = buffer[bufferIndex + 3];
        const auto msgLen = Math::U2(&buffer[bufferIndex + 8]);
        const size_t totalLen = static_cast<size_t>(hlen) + static_cast<size_t>(msgLen) + 4;

        // 长度合理性下限校验：OEM7 帧头至少 28 字节，totalLen 不可能小于此值。
        // 若不满足，说明 0xAA 0x44 0x12 出现在消息体内（假同步头），前进 1 字节重新扫描。
        // 注意：msgLen 为 16 位（≤65535），totalLen 上限天然受限，不会出现"等待一个永远到不齐的超大帧"
        // 而永久卡死；故此处只做下界保护，不限制上界（否则会把 60KB+ 的真实大帧误判为假同步头而丢弃）。
        // 真正的"对端静默消失"由 SocketClient 的 idleTooLong 半开连接检测负责。
        if (totalLen < 28) {
            bufferIndex = start + 1;
            continue;
        }

        if (buffer.size() - bufferIndex < totalLen) {
            return false;   // 整帧未齐，等下一轮接收
        }

        message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                       buffer.begin() + static_cast<long long>(bufferIndex + totalLen));
        bufferIndex += totalLen;

        // 定期清理已消费的前半段，防止 buffer 无限增长
        if (bufferIndex > buffer.size() / 2) {
            buffer.erase(buffer.begin(), buffer.begin() + static_cast<long long>(bufferIndex));
            bufferIndex = 0;
        }

        return true;
    }
    return false;
}
