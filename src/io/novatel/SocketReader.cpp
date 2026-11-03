#include "novatel/SocketReader.h"

#include "MathUtils.h"
namespace NovAtel {

bool SocketReader::connect(const std::string& ip, const unsigned short port) {
    buffer.clear();
    bufferIndex = 0;
    return socketClient.connect(ip, port);
}

void SocketReader::close() {
    socketClient.close();
    buffer.clear();
    bufferIndex = 0;
}

bool SocketReader::getNextMessage(std::vector<std::uint8_t>& message, const bool block) {

    unsigned char recvBuf[MAXRAWLEN];
    if (const int lenR = socketClient.receive(recvBuf, MAXRAWLEN - 1, block ? -1 : 0); lenR > 0) {
        buffer.insert(buffer.end(), recvBuf, recvBuf + lenR);
    }

    while (buffer.size() - bufferIndex >= 3) {

        std::size_t start = bufferIndex;
        bool found = false;
        for (; start + 2 < buffer.size(); ++start) {
            if (buffer[start] == 0xAA && buffer[start + 1] == 0x44 && buffer[start + 2] == 0x12) {
                found = true;
                break;
            }
        }

        if (!found) {

            const std::size_t keep = buffer.size() > 2 ? 2 : buffer.size();
            buffer.erase(buffer.begin(), buffer.end() - static_cast<long long>(keep));
            bufferIndex = 0;
            return false;
        }

        bufferIndex = start;

        if (buffer.size() - bufferIndex < 28) {
            return false;
        }

        const auto hlen = buffer[bufferIndex + 3];
        const auto msgLen = Math::U2(&buffer[bufferIndex + 8]);
        const std::size_t totalLen = static_cast<std::size_t>(hlen) + static_cast<std::size_t>(msgLen) + 4;



        if (totalLen < 28 || totalLen > kMaxFrameBytes) {
            bufferIndex = start + 1;
            continue;
        }

        if (buffer.size() - bufferIndex < totalLen) {
            return false;
        }



        message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                       buffer.begin() + static_cast<long long>(bufferIndex + totalLen));
        if (!crcExam(message)) {
            bufferIndex = start + 1;
            continue;
        }

        bufferIndex += totalLen;

        if (bufferIndex > buffer.size() / 2) {
            buffer.erase(buffer.begin(), buffer.begin() + static_cast<long long>(bufferIndex));
            bufferIndex = 0;
        }
        return true;
    }
    return false;
}
}
