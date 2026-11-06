#include "CRXDecoder.h"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <string_view>

namespace {
    constexpr std::string_view kVersionLabel = "CRINEX VERS   / TYPE";
    constexpr std::string_view kEndOfHeader = "END OF HEADER";
    constexpr std::string_view kTypes2 = "# / TYPES OF OBSERV";
    constexpr std::string_view kTypes3 = "SYS / # / OBS TYPES";

    long long prefixInt(const std::string_view s) {
        std::size_t i = 0;
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
        long long v = 0;
        if (auto [p, ec] = std::from_chars(s.data() + i, s.data() + s.size(), v); ec == std::errc{})
            return v;
        return 0;
    }

    std::string rtrim(const std::string &s) {
        std::size_t n = s.size();
        while (n > 0 && s[n - 1] == ' ') --n;
        return s.substr(0, n);
    }

    std::string padLeft(const std::string &s, const std::size_t w) {
        return s.size() >= w ? s : std::string(w - s.size(), ' ') + s;
    }

    std::string padRight(std::string s, const std::size_t w) {
        if (s.size() < w) s += std::string(w - s.size(), ' ');
        return s;
    }

    std::string zeroPad(const long long v, const int prec) {
        std::string d = std::to_string(std::llabs(v));
        if (d.size() < static_cast<std::size_t>(prec)) d = std::string(prec - d.size(), '0') + d;
        return (v < 0 ? "-" : "") + d;
    }

    std::string labelAt(const std::string &s) {
        return s.size() < 60 ? std::string() : s.substr(60);
    }

    void repairText(std::string &base, const std::string &diff) {
        const std::size_t n = std::min(base.size(), diff.size());
        for (std::size_t i = 0; i < n; ++i) {
            if (diff[i] == ' ') continue;
            base[i] = diff[i] == '&' ? ' ' : diff[i];
        }
        if (diff.size() > base.size()) {
            std::string tail = diff.substr(base.size());
            std::replace(tail.begin(), tail.end(), '&', ' ');
            base += tail;
        }
    }
}

bool CRXDecoder::isCompactRINEX(const std::string &path) {
    std::ifstream f(path);
    std::string first;
    if (!f || !std::getline(f, first)) return false;
    if (!first.empty() && first.back() == '\r') first.pop_back();
    return first.size() >= 79 && first.substr(60, 19) == kVersionLabel.substr(0, 19);
}

bool CRXDecoder::fail(const std::string &msg) {
    error = msg + "（第 " + std::to_string(lineNo) + " 行）";
    return false;
}

bool CRXDecoder::readLine(std::string &s) {
    if (hasPending) {
        s = std::move(pending);
        pending.clear();
        hasPending = false;
        return true;
    }
    if (!std::getline(in, s)) return false;
    ++lineNo;
    if (!s.empty() && s.back() == '\r') s.pop_back();
    return true;
}

void CRXDecoder::parseHeaderTypes(const std::string &s) {
    if (const std::string label = labelAt(s); label.compare(0, kTypes2.size(), kTypes2) == 0 && s.size() > 5 && s[5] != ' ') {
        ntype = static_cast<int>(prefixInt(std::string_view(s).substr(0, 6)));
    } else if (label.compare(0, kTypes3.size(), kTypes3) == 0 && !s.empty() && s[0] != ' ') {
        ntypeGnss[static_cast<unsigned char>(s[0])] =
                static_cast<int>(prefixInt(std::string_view(s).substr(3, 3)));
    }
}

bool CRXDecoder::readHeader() {
    std::string s;
    if (!readLine(s) || s.size() < 79 || s.substr(60, 19) != kVersionLabel.substr(0, 19))
        return fail("不是 Compact RINEX 文件");
    crinexVersion = static_cast<int>(prefixInt(s));
    crinexFull = s.size() > 2 && s[1] == '.' && std::isdigit(static_cast<unsigned char>(s[2]))
                     ? (s[0] - '0') * 10 + (s[2] - '0')
                     : -1;
    if (crinexVersion != 1 && crinexVersion != 3) return fail("不支持的 CRINEX 版本");

    if (!readLine(s)) return fail("头部被截断");
    if (!readLine(s) || s.size() < 61 ||
        labelAt(s).compare(0, 20, "RINEX VERSION / TYPE") != 0)
        return fail("缺少 RINEX VERSION / TYPE");
    rinexVersion = static_cast<int>(prefixInt(s));
    if (s[5] != '2' && s[5] != '3' && s[5] != '4') return fail("不支持的 RINEX 版本");

    if (rinexVersion == 2) {
        epTopFrom = '&';
        epTopTo = ' ';
        eventPos = 28;
        nSatPos = 29;
        satLstPos = 32;
        shiftClk = 1;
        offset = 3;
    } else {
        epTopFrom = '>';
        epTopTo = '>';
        eventPos = 31;
        nSatPos = 32;
        satLstPos = 41;
        shiftClk = 4;
        offset = 6;
    }

    for (;;) {
        out << rtrim(s) << '\n';
        parseHeaderTypes(s);
        if (labelAt(s).compare(0, kEndOfHeader.size(), kEndOfHeader) == 0) break;
        if (!readLine(s)) return fail("头部缺少 END OF HEADER");
    }
    return true;
}

bool CRXDecoder::validEpochLine() const {
    if (line.empty() || line[0] != epTopTo) return false;
    if (line.size() < offset + 26) return false;
    return line[offset + 23] == ' ' && line[offset + 24] == ' ' &&
           std::isdigit(static_cast<unsigned char>(line[offset + 25])) != 0;
}

bool CRXDecoder::parseClock(const std::string &s, bool &haveClock) {
    haveClock = !s.empty();
    if (!haveClock) {
        clkOrder = -1;
        return true;
    }
    std::string num = s;
    if (s.size() > 1 && s[1] == '&') {
        clkArcOrder = static_cast<int>(prefixInt(s));
        clkOrder = -1;
        num = s.substr(2);
        if (clkArcOrder > kMaxDiffOrder) return fail("钟差差分阶数超限");
    }
    if (num.empty()) return fail("钟差记录为空");
    if (const std::size_t digits = num.size() - (num[0] == '-' ? 1u : 0u); digits < 9) {
        clkCur.u[0] = 0;
        clkCur.l[0] = prefixInt(num);
    } else {
        clkCur.l[0] = prefixInt(std::string_view(num).substr(num.size() - 8));
        clkCur.u[0] = prefixInt(std::string_view(num).substr(0, num.size() - 8));
        if (clkCur.u[0] < 0) clkCur.l[0] = -clkCur.l[0];
    }
    return true;
}

void CRXDecoder::processClock() {
    if (clkOrder < clkArcOrder) {
        ++clkOrder;
        for (int i = 0, j = 1; i < clkOrder; ++i, ++j) {
            clkCur.u[j] = clkCur.u[i] + clkPrev.u[i];
            clkCur.l[j] = clkCur.l[i] + clkPrev.l[i];
            clkCur.u[j] += clkCur.l[j] / 100000000;
            clkCur.l[j] %= 100000000;
        }
    } else {
        for (int i = 0, j = 1; i < clkOrder; ++i, ++j) {
            clkCur.u[j] = clkCur.u[i] + clkPrev.u[j];
            clkCur.l[j] = clkCur.l[i] + clkPrev.l[j];
            clkCur.u[j] += clkCur.l[j] / 100000000;
            clkCur.l[j] %= 100000000;
        }
    }
}

int CRXDecoder::typesOf(const std::string &satId) const {
    if (rinexVersion == 2 || satId.empty()) return ntype;
    return ntypeGnss[static_cast<unsigned char>(satId[0])];
}

bool CRXDecoder::parseSatDiff(const std::string &s, const int sat, const int prevIdx, const int nt) {
    std::vector<std::string> fields;
    fields.reserve(nt);
    std::size_t pos = 0;
    for (int j = 0; j < nt; ++j) {
        const std::size_t sp = s.find(' ', pos);
        if (sp == std::string::npos) {
            fields.push_back(s.substr(pos));
            pos = s.size();
            while (static_cast<int>(fields.size()) < nt) fields.emplace_back("");
            break;
        }
        fields.push_back(s.substr(pos, sp - pos));
        pos = sp + 1;
    }
    dflag[sat] = s.substr(pos);

    for (int j = 0; j < nt; ++j) {
        auto &[u, l, order, arcOrder] = diffCur[sat][j];
        const std::string &tok = fields[j];
        if (tok.empty()) {
            arcOrder = -1;
            order = -1;
            continue;
        }
        std::string num = tok;
        if (tok.size() > 1 && tok[1] == '&') {
            order = 0;
            arcOrder = static_cast<int>(prefixInt(tok));
            num = tok.substr(2);
            if (arcOrder > kMaxDiffOrder) return fail("观测差分阶数超限");
        } else {
            if (prevIdx < 0 || prevIdx >= static_cast<int>(diffPrev.size()) ||
                j >= static_cast<int>(diffPrev[prevIdx].size()))
                return fail("新卫星未初始化 arc");
            const Diff &prev = diffPrev[prevIdx][j];
            if (prev.arcOrder < 0) return fail("观测量未初始化 arc");
            order = prev.order;
            arcOrder = prev.arcOrder;
            if (order < arcOrder) ++order;
        }
        const int i = order;
        if (i < 0 || i > kMaxDiffOrder) return fail("差分阶数越界");
        if (num.empty()) return fail("观测差分记录为空");
        if (const std::size_t digits = num.size() - (num[0] == '-' ? 1u : 0u); digits < 6) {
            u[i] = 0;
            l[i] = prefixInt(num);
        } else {
            l[i] = prefixInt(std::string_view(num).substr(num.size() - 5));
            u[i] = prefixInt(std::string_view(num).substr(0, num.size() - 5));
            if (u[i] < 0) l[i] = -l[i];
        }
    }
    return true;
}

std::string CRXDecoder::formatField(const Diff &d, const char f0, const char f1) {
    long long u = d.u[0];
    long long l = d.l[0];
    if (u < 0 && l > 0) {
        ++u;
        l -= 100000;
    } else if (u > 0 && l < 0) {
        --u;
        l += 100000;
    }
    std::string s;
    if (u != 0) {
        s = padLeft(std::to_string(u), 8) + " " + zeroPad(std::llabs(l), 5) + f0 + f1;
        s[8] = s[9];
        s[9] = s[10];
    } else {
        s = std::string(9, ' ') + zeroPad(std::llabs(l), 5) + f0 + f1;
        if (s[9] != '0') {
            s[8] = s[9];
            s[9] = s[10];
            if (l < 0) s[7] = '-';
        } else if (s[10] != '0') {
            s[9] = s[10];
            s[8] = l < 0 ? '-' : ' ';
        } else {
            s[9] = l < 0 ? '-' : ' ';
        }
    }
    s[10] = '.';
    return s;
}

std::string CRXDecoder::formatClock(long long u, long long l) const {
    if (u < 0 && l > 0) {
        ++u;
        l -= 100000000;
    } else if (u > 0 && l < 0) {
        --u;
        l += 100000000;
    }
    const int sgn = l < 0 ? -1 : 1;
    const std::string tmp = zeroPad(u * 10 + sgn, shiftClk + 1);
    const int n = static_cast<int>(tmp.size()) - 1;
    if (n < shiftClk) return {};
    std::string res = "  ." + tmp.substr(n - shiftClk, shiftClk);
    if (n > shiftClk) {
        res[1] = tmp[n - shiftClk - 1];
        if (n > shiftClk + 1) res[0] = tmp[n - shiftClk - 2];
    }
    res += zeroPad(std::llabs(l), 8);
    return res;
}

void CRXDecoder::emitEpochLine() {
    if (rinexVersion == 2) {
        if (clkOrder >= 0) {
            buf += padRight(line.substr(0, 68), 68);
            buf += formatClock(clkCur.u[clkOrder], clkCur.l[clkOrder]);
            buf += '\n';
        } else {
            buf += line.substr(0, 68);
            buf += '\n';
        }
        for (int n = nsat - 12, p = 68; n > 0; n -= 12, p += 36)
            buf += std::string(32, ' ') + line.substr(p, 36) + '\n';
        return;
    }
    const std::string head = line.substr(0, 41);
    if (clkOrder >= 0) {
        buf += head + formatClock(clkCur.u[clkOrder], clkCur.l[clkOrder]) + '\n';
    } else {
        buf += rtrim(head) + '\n';
    }
}

void CRXDecoder::emitSatData() {
    flagCur.assign(nsat, std::string());
    for (int i = 0; i < nsat; ++i) {
        const int nt = typesOf(satIds[i]);
        const int prev = satTable[i];
        std::string f = prev < 0
                            ? (rinexVersion >= 3 ? std::string() : padRight(dflag[i], nt * 2))
                            : flagPrev[prev].substr(0, nt * 2);
        repairText(f, dflag[i]);
        if (f.size() < nt * 2)
            f += std::string(nt * 2 - f.size(), ' ');
        flagCur[i] = f;

        std::string row = rinexVersion >= 3 ? satIds[i] : std::string();
        for (int j = 0; j < nt; ++j) {
            Diff &d = diffCur[i][j];
            const char f0 = f[2 * j];
            const char f1 = f[2 * j + 1];
            if (d.arcOrder >= 0) {
                if (d.order > 0 && prev >= 0) {
                    const Diff &prevDiff = diffPrev[prev][j];
                    for (int k = d.order, k1 = k - 1; k > 0; --k, --k1) {
                        d.u[k1] = d.u[k] + prevDiff.u[k1];
                        d.l[k1] = d.l[k] + prevDiff.l[k1];
                        d.u[k1] += d.l[k1] / 100000;
                        d.l[k1] %= 100000;
                    }
                }
                row += formatField(d, f0, f1);
            } else if (crinexVersion == 1) {
                row += std::string(16, ' ');
                flagCur[i][2 * j] = ' ';
                flagCur[i][2 * j + 1] = ' ';
            } else {
                row += std::string(14, ' ') + f0 + f1;
            }
            if (j + 1 == nt || (rinexVersion == 2 && (j + 1) % 5 == 0)) {
                buf += rtrim(row) + '\n';
                row.clear();
            }
        }
    }
}

void CRXDecoder::saveState() {
    nsatPrev = nsat;
    clkPrev = clkCur;
    satPrev = satIds;
    flagPrev = flagCur;
    diffPrev = diffCur;
}

bool CRXDecoder::processEpoch(const std::string &dline) {
    repairText(line, dline);
    if (!validEpochLine()) return fail("历元行格式异常");
    line = rtrim(line);

    nsat = static_cast<int>(prefixInt(std::string_view(line).substr(nSatPos)));
    if (nsat <= 0 || nsat > 100) return fail("卫星数异常");

    satIds.clear();
    satIds.reserve(nsat);
    for (int i = 0; i < nsat; ++i) {
        const std::size_t p = satLstPos + 3 * static_cast<std::size_t>(i);
        satIds.push_back(p + 3 <= line.size() ? line.substr(p, 3) : std::string("   "));
    }
    satTable.assign(nsat, -1);
    for (int i = 0; i < nsat; ++i)
        for (int j = 0; j < nsatPrev; ++j)
            if (satIds[i] == satPrev[j]) {
                satTable[i] = j;
                break;
            }

    std::string clockLine;
    if (!readLine(clockLine)) return fail("缺少钟差记录");
    bool haveClock = false;
    if (!parseClock(clockLine, haveClock)) return false;

    dflag.assign(nsat, std::string());
    diffCur.assign(nsat, std::vector<Diff>());
    for (int i = 0; i < nsat; ++i) {
        const int nt = typesOf(satIds[i]);
        if (nt <= 0 || nt > 100) return fail("观测量类型数异常");
        diffCur[i].resize(nt);
        std::string s;
        if (!readLine(s)) return fail("观测数据被截断");
        if (!parseSatDiff(s, i, satTable[i], nt)) return false;
    }

    if (haveClock) processClock();
    emitEpochLine();
    emitSatData();
    out << buf;
    if (!out) return fail("写入 RINEX 失败");
    buf.clear();
    saveState();
    return true;
}

bool CRXDecoder::handleEvent(std::string &dline) {
    for (;;) {
        if (!dline.empty()) dline[0] = epTopTo;
        dline = rtrim(dline);
        out << dline << '\n';
        if (dline.size() > 29) {
            const int n = static_cast<int>(prefixInt(std::string_view(dline).substr(eventPos + 1)));
            if (n < 0) return fail("事件记录条数异常");
            for (int i = 0; i < n; ++i) {
                std::string s;
                if (!readLine(s)) return fail("事件记录被截断");
                out << rtrim(s) << '\n';
                parseHeaderTypes(s);
            }
        }
        do {
            if (!readLine(dline)) return true;
        } while (crinexVersion >= 3 && !dline.empty() && dline[0] == '&');
        if (dline.empty() || dline[0] != epTopFrom || dline.size() < 29 ||
            eventPos >= dline.size() ||
            !std::isdigit(static_cast<unsigned char>(dline[eventPos])))
            return fail("事件记录后缺少初始化历元");
        if (dline[eventPos] == '0' || dline[eventPos] == '1') break;
    }
    pending = dline;
    hasPending = true;
    return true;
}

bool CRXDecoder::runEpochs() {
    std::string dline;
    while (readLine(dline)) {
        if (crinexVersion == 3)
            while (!dline.empty() && dline[0] == '&')
                if (!readLine(dline)) return true;

        if (!dline.empty() && dline[0] == '\032') return true;
        if (!dline.empty() && dline[0] == epTopFrom) {
            dline[0] = epTopTo;
            if (eventPos < dline.size() && dline[eventPos] != '0' && dline[eventPos] != '1') {
                if (!handleEvent(dline)) return false;
                continue;
            }
            line.clear();
            nsatPrev = 0;
        }
        if (!processEpoch(dline)) return false;
    }
    return true;
}

bool CRXDecoder::decode(const std::string &inPath, const std::string &outPath, std::string &err) {
    in.open(inPath);
    out.open(outPath);
    if (!in || !out) {
        err = "无法打开输入/输出文件";
        return false;
    }
    std::fill(ntypeGnss.begin(), ntypeGnss.end(), -1);

    const bool ok = readHeader() && runEpochs();
    out.close();
    in.close();
    if (!ok) {
        std::error_code ec;
        std::filesystem::remove(outPath, ec);
        err = error.empty() ? "Compact RINEX 解压失败" : error;
        return false;
    }
    err.clear();
    return true;
}
