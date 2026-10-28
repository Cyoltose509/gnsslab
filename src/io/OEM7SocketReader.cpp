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

bool OEM7SocketReader::getNextMessage(std::vector<uint8_t>& message, const bool block) {
    // 实时流：block=true 时最多等一次接收；false 时只取内核已缓冲数据（0 超时 poll 即返回）。
    unsigned char recvBuf[MAXRAWLEN];
    if (const int lenR = socketClient.receive(recvBuf, MAXRAWLEN - 1, block ? -1 : 0); lenR > 0) {
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
            // 仅保留末尾 2 字节（可能是跨接收边界的同步头前缀），避免误删真实同步头
            const size_t keep = buffer.size() > 2 ? 2 : buffer.size();
            buffer.erase(buffer.begin(), buffer.end() - static_cast<long long>(keep));
            bufferIndex = 0;
            return false;
        }

        bufferIndex = start;

        if (buffer.size() - bufferIndex < 28) {
            return false; // 头部未齐，等下一轮接收
        }

        const auto hlen = buffer[bufferIndex + 3];
        const auto msgLen = Math::U2(&buffer[bufferIndex + 8]);
        const size_t totalLen = static_cast<size_t>(hlen) + static_cast<size_t>(msgLen) + 4;

        // 长度越界 = 数据体内偶发的假同步头，前进 1 字节重新扫描（不能按该长度跳过，
        // 否则会把落在其"长度范围"内的真实帧整段吞掉）。
        if (totalLen < 28 || totalLen > kMaxFrameBytes) {
            bufferIndex = start + 1;
            continue;
        }

        if (buffer.size() - bufferIndex < totalLen) {
            return false; // 整帧未齐，等下一轮接收补齐
        }

        // CRC 门禁：只有校验通过的帧才抽取并推进 bufferIndex，抽取失败仅前进 1 字节重扫，
        // 保证真实帧不会被假帧"吞掉"（假同步头在观测体二进制中无法完全避免）。
        message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                       buffer.begin() + static_cast<long long>(bufferIndex + totalLen));
        if (!crcExam(message)) {
            bufferIndex = start + 1;
            continue;
        }

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
