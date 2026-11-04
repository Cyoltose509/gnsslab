#include <fstream>
#include <vector>
#include <algorithm>

#include "novatel/BinaryReader.h"
#include "Const.h"

#include "Log.h"
#include "MathUtils.h"
#include "StringUtils.h"

#include <windows.h>

namespace NovAtel {

    bool BinaryReader::open(const std::string &filename) {
        std::ifstream ifs(Str::ACPToWide(filename), std::ios::binary | std::ios::ate);
        if (!ifs) return false;

        fileSize = static_cast<std::size_t>(ifs.tellg());
        ifs.seekg(0, std::ios::beg);

        buffer.clear();
        bufferIndex = 0;
        mImuLog.clear();
        mInsLayout = InsRecordLayout();
        mImuType = -1;
        mInsDiag.clear();
        mInsDiagLogged.store(false);
        std::uint8_t tmp[4096];
        while (ifs.read(reinterpret_cast<char *>(tmp), sizeof(tmp)) || ifs.gcount() > 0) {
            buffer.insert(buffer.end(), tmp, tmp + ifs.gcount());
        }
        ifs.close();
        mEphSnap = std::make_shared<EphemerisTable>();

        probeInsLayout();
        if (mImuType < 0) probeImuTypeFromShortHeader();
        if (mImuType >= 0) mInsDiag.clear();
        return !buffer.empty();
    }

    void BinaryReader::close() {
        buffer.clear();
        bufferIndex = 0;
        LogReader::close();
    }

    static constexpr BinaryReader::InsRecordLayout kInsLayoutCandidates[] = {
        {true, 56, 14, 13, 8, 16, 24, 28, "RAWIMUSX 字段集（含 IMU Type）"},
        {true, 56, 12, -1, 8, 16, 24, 28, "RAWIMUS 字段集（无 IMU Type）"},
    };

    bool BinaryReader::acceptInsRecord(const std::size_t start, const InsRecordLayout &layout) const {
        if (start + static_cast<std::size_t>(layout.recLen) > buffer.size()) return false;

        if (const auto body = static_cast<std::size_t>(layout.recLen) - 4;
            crc32(&buffer[start], body) != Math::U4(&buffer[start + body])) return false;

        if (const int week = Math::U2(&buffer[start + layout.weekOffset]); week < 1000 || week > 3000) return false;

        double sec = 0.0;
        std::memcpy(&sec, &buffer[start + layout.secOffset], sizeof(double));
        if (!(sec >= 0.0 && sec <= FULL_WEEK)) return false;
        if (const double msSec = Math::U4(&buffer[start + layout.towMsOffset]) * 0.001; std::fabs(msSec - sec) > 2e-3) return false;

        if (layout.imuTypeOffset >= 0 && buffer[start + layout.imuTypeOffset] == 0) return false;

        return true;
    }

    bool BinaryReader::probeInsLayout() {
        constexpr std::size_t kMaxProbe = 2000;
        constexpr int kCandidates = std::size(kInsLayoutCandidates);
        std::size_t probes[kCandidates] = {};
        int pass[kCandidates] = {};
        int firstType = -1;

        std::size_t i = 0;
        std::size_t considered = 0;
        while (i + 56 <= buffer.size() && considered < kMaxProbe) {
            if (buffer[i] == 0xAA && buffer[i + 1] == 0x44 && buffer[i + 2] == 0x13 && buffer[i + 3] == 0x28) {
                ++considered;
                for (int c = 0; c < kCandidates; ++c) {
                    ++probes[c];
                    if (acceptInsRecord(i, kInsLayoutCandidates[c])) {
                        ++pass[c];
                        if (firstType < 0 && kInsLayoutCandidates[c].imuTypeOffset >= 0)
                            firstType = buffer[i + kInsLayoutCandidates[c].imuTypeOffset];
                    }
                }
                i += 56;
            } else {
                ++i;
            }
        }

        if (considered == 0) {
            mInsDiag = "文件中没有 0xAA4413 紧凑 IMU 记录（可能只有标准 OEM7 日志）";
            return false;
        }

        int best = -1;
        for (int c = 0; c < kCandidates; ++c) {
            if (probes[c] > 0 && pass[c] * 100 >= probes[c] * 95) {
                if (best < 0 || pass[c] > pass[best]) best = c;
            }
        }

        if (best < 0) {
            mInsDiag = "发现 " + std::to_string(considered) +
                       " 条 0xAA4413 候选记录，但没有一种已知布局能通过 CRC/时间一致性校验"
                       "（已试 " + std::to_string(kCandidates) + " 种）";
            return false;
        }

        mInsLayout = kInsLayoutCandidates[best];
        mInsLayout.valid = true;
        mImuType = firstType;
        return true;
    }

    void BinaryReader::probeImuTypeFromShortHeader() {
        std::size_t scanned = 0;
        for (std::size_t i = 0; i + 12 <= buffer.size() && scanned < 2000; ++i) {
            if (!(buffer[i] == 0xAA && buffer[i + 1] == 0x44 && buffer[i + 2] == 0x12)) continue;
            if (const int msgId = Math::U2(&buffer[i + 4]); msgId != LogID::RAWIMUX && msgId != LogID::RAWIMUSX) continue;
            const int hLen = buffer[i + 3] == 28 ? 28 : 12;
            if (i + static_cast<std::size_t>(hLen) + 2 > buffer.size()) continue;
            ++scanned;
            if (const std::uint8_t type = buffer[i + hLen + 1]; type != 0) {
                mImuType = type;
                return;
            }
        }
    }

    bool BinaryReader::getNextMessage(std::vector<std::uint8_t> &message, [[maybe_unused]] const bool block) {
        while (buffer.size() - bufferIndex >= 3) {
            std::size_t start = bufferIndex;
            bool found = false;
            std::uint8_t sync = 0;
            for (; start + 2 < buffer.size(); ++start) {
                if (buffer[start] == 0xAA && buffer[start + 1] == 0x44 &&
                    (buffer[start + 2] == 0x12 || buffer[start + 2] == 0x13)) {
                    found = true;
                    sync = buffer[start + 2];
                    break;
                }
            }

            if (!found) {
                buffer.clear();
                bufferIndex = 0;
                return false;
            }

            bufferIndex = start;

            if (buffer.size() - bufferIndex < 12)
                return false;

            if (sync == 0x13) {
                if (mInsLayout.valid && acceptInsRecord(bufferIndex, mInsLayout)) {
                    message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                                   buffer.begin() + static_cast<long long>(bufferIndex) + mInsLayout.recLen);
                    message[4] = static_cast<std::uint8_t>(LogID::RAWIMUCOMPACT & 0xFF);
                    message[5] = static_cast<std::uint8_t>(LogID::RAWIMUCOMPACT >> 8 & 0xFF);
                    bufferIndex += mInsLayout.recLen;
                    return true;
                }
                if (!mInsLayout.valid && !mInsDiagLogged.exchange(true) && !mInsDiag.empty())
                    LOG_ERROR << "BinaryReader: " << mInsDiag;
                bufferIndex += 3;
                continue;
            }

            const auto b3 = static_cast<int>(buffer[bufferIndex + 3]);
            const bool longHeader = b3 == 28;
            const int hlen = longHeader ? 28 : 12;
            const int msgLen = longHeader ? static_cast<int>(Math::U2(&buffer[bufferIndex + 8])) : b3;
            const int totalLen = hlen + msgLen + 4;

            if (buffer.size() - bufferIndex < static_cast<std::size_t>(totalLen) || totalLen < hlen + 4) {
                bufferIndex += 3;
                continue;
            }

            message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                           buffer.begin() + static_cast<long long>(bufferIndex) + totalLen);
            bufferIndex += totalLen;
            return true;
        }
        return false;
    }
}
