#pragma once
#include <functional>
#include <fstream>
#include <string>
#include <algorithm>
#include <utility>
#include "StringUtils.h"
#include "Exception.h"

struct RinexVersionType {
    double version = 0.0;
    std::string type;
    std::string system;
};

[[nodiscard]] inline std::string rinexLabel(const std::string &line) {
    return Str::strip(Str::sub(line, 60, 20));
}

[[nodiscard]] inline RinexVersionType parseRinexVersionType(const std::string &line) {
    RinexVersionType v;
    v.version = Str::toDouble(Str::sub(line, 0, 20));
    v.type = Str::strip(Str::sub(line, 20, 20));
    v.system = Str::strip(Str::sub(line, 40, 20));
    return v;
}

class RINEXFileBase {
public:
    RINEXFileBase() = default;

    RINEXFileBase(const RINEXFileBase &) = delete;

    RINEXFileBase &operator=(const RINEXFileBase &) = delete;

    /// 移动：连同已打开的流一起搬走；pStream 原本指着自家的流，搬到新家要重新指过去。
    RINEXFileBase(RINEXFileBase &&other) noexcept
        : pStream(other.pStream == &other.ownedStream ? &ownedStream : other.pStream),
          pendingLine(std::move(other.pendingLine)),
          pushedBack(other.pushedBack) {
        ownedStream = std::move(other.ownedStream);
        other.pStream = nullptr;
    }

    RINEXFileBase &operator=(RINEXFileBase &&other) noexcept {
        if (this == &other) return *this;
        ownedStream = std::move(other.ownedStream);
        pStream = other.pStream == &other.ownedStream ? &ownedStream : other.pStream;
        pendingLine = std::move(other.pendingLine);
        pushedBack = other.pushedBack;
        other.pStream = nullptr;
        return *this;
    }

    virtual ~RINEXFileBase() = default;

    void openFile(const std::string &path) {
        if (ownedStream.is_open()) ownedStream.close();
        ownedStream.open(path, std::ios::in | std::ios::binary);
        if (!ownedStream) throw FFStreamError("can't open file: " + path);
        pStream = &ownedStream;
        pushedBack = false;
    }

    void attach(std::fstream &fs) {
        pStream = &fs;
        pushedBack = false;
    }

protected:
    std::fstream *pStream = nullptr;

    bool nextLine(std::string &line) const {
        if (pushedBack) {
            line = pendingLine;
            pushedBack = false;
            return true;
        }
        if (!pStream || !std::getline(*pStream, line)) return false;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        return true;
    }

    void pushBack(const std::string &line) const {
        pendingLine = line;
        pushedBack = true;
    }

    mutable std::string pendingLine;
    mutable bool pushedBack = false;

    static void fortranExponent(std::string &line) {
        std::replace(line.begin(), line.end(), 'D', 'e');
    }

    void scanLabeledHeader(const std::function<void(const std::string &, const std::string &)> &cb) {
        std::string line;
        while (true) {
            if (!nextLine(line)) throw FFStreamError("Unexpected EOF in RINEX header");
            const std::string label = rinexLabel(line);
            if (label == "END OF HEADER") break;
            cb(label, line);
        }
    }

private:
    std::fstream ownedStream;
};
