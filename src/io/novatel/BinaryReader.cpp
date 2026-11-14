#include <fstream>
#include <vector>
#include <algorithm>
#include <cstring>

#include "novatel/BinaryReader.h"
#include "MathUtils.h"
#include "StringUtils.h"

namespace NovAtel {

    namespace {
        constexpr std::size_t kProbeWindow = 64 * 1024;

        bool isImuId(const int id) {
            return id == LogID::RAWIMU || id == LogID::RAWIMUS ||
                   id == LogID::RAWIMUX || id == LogID::RAWIMUSX;
        }
    }

    bool BinaryReader::open(const std::string &filename) {
        close();
        std::ifstream ifs(Str::ACPToWide(filename), std::ios::binary | std::ios::ate);
        if (!ifs) return false;

        const auto fileSize = static_cast<std::size_t>(ifs.tellg());
        ifs.seekg(0, std::ios::beg);
        buffer.resize(fileSize);
        if (fileSize > 0) {
            ifs.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(fileSize));
            buffer.resize(static_cast<std::size_t>(ifs.gcount()));
        }
        ifs.close();
        mEphSnap = std::make_shared<EphemerisTable>();

        probeImuType();
        return !buffer.empty();
    }

    void BinaryReader::close() {
        buffer.clear();
        bufferIndex = 0;
        LogReader::close();
    }

    // 0x13 短头的 +3 是消息体长度；0x12 长头的 +3 是头长、体长在 +8。
    std::size_t BinaryReader::frameLengthAt(const std::size_t at) const {
        if (at + 12 > buffer.size() || buffer[at] != 0xAA || buffer[at + 1] != 0x44) return 0;
        const bool shortHeader = buffer[at + 2] == 0x13;
        if (!shortHeader && buffer[at + 2] != 0x12) return 0;
        const int hlen = shortHeader ? 12 : buffer[at + 3] == 28 ? 28 : 12;
        const int body = shortHeader ? static_cast<int>(buffer[at + 3])
                        : hlen == 28 ? static_cast<int>(Math::U2(&buffer[at + 8]))
                              : static_cast<int>(buffer[at + 3]);
        const std::size_t total = static_cast<std::size_t>(hlen + body + 4);
        if (at + total > buffer.size()) return 0;
        return total;
    }

    void BinaryReader::probeImuType() {
        const std::size_t end = std::min(buffer.size(), kProbeWindow);
        std::size_t at = 0;
        while (at + 12 <= end) {
            const auto *hit = static_cast<const unsigned char *>(
                std::memchr(buffer.data() + at, 0xAA, end - at));
            if (!hit) break;
            at = static_cast<std::size_t>(hit - buffer.data());
            const std::size_t total = frameLengthAt(at);
            if (total == 0) { ++at; continue; }

            if (const int id = Math::U2(&buffer[at + 4]); isImuId(id) && crc32(&buffer[at], total - 4) == Math::U4(&buffer[at + total - 4])) {
                if (id == LogID::RAWIMUX || id == LogID::RAWIMUSX) {
                    const int hlen = buffer[at + 2] == 0x13 ? 12 : buffer[at + 3] == 28 ? 28 : 12;
                    if (const int type = buffer[at + hlen + 1]; type > 0) mImuType = type;
                }
                return;
            }
            at += total;
        }
    }

    bool BinaryReader::getNextMessage(std::vector<std::uint8_t> &message, [[maybe_unused]] const bool block) {
        while (buffer.size() - bufferIndex >= 12) {
            std::size_t start = bufferIndex;
            while (start + 2 < buffer.size() &&
                   !(buffer[start] == 0xAA && buffer[start + 1] == 0x44 &&
                     (buffer[start + 2] == 0x12 || buffer[start + 2] == 0x13))) ++start;
            if (start + 2 >= buffer.size()) {
                buffer.clear();
                bufferIndex = 0;
                return false;
            }
            bufferIndex = start;

            const bool shortHeader = buffer[start + 2] == 0x13;
            const std::size_t total = frameLengthAt(start);
            if (total == 0) { bufferIndex += 3; continue; }
            // 短头帧按 CRC 定界。
            if (shortHeader &&
                crc32(&buffer[start], total - 4) != Math::U4(&buffer[start + total - 4])) {
                bufferIndex += 3;
                continue;
            }

            message.assign(buffer.begin() + static_cast<long long>(start),
                           buffer.begin() + static_cast<long long>(start + total));
            bufferIndex += total;
            return true;
        }
        return false;
    }
}
