#pragma once
#include <fstream>
#include <string>
#include <algorithm>
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

    virtual ~RINEXFileBase() = default;

    void openFile(const std::string &path) {
        if (ownedStream.is_open()) ownedStream.close();
        ownedStream.open(path, std::ios::in);
        if (!ownedStream) throw FFStreamError("can't open file: " + path);
        pStream = &ownedStream;
    }

    void attach(std::fstream &fs) { pStream = &fs; }

protected:
    std::fstream *pStream = nullptr;

    bool nextLine(std::string &line) const {
        if (!pStream || !std::getline(*pStream, line)) return false;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        return true;
    }

    static void fortranExponent(std::string &line) {
        std::replace(line.begin(), line.end(), 'D', 'e');
    }

    template<class Cb>
    void scanLabeledHeader(Cb &&cb) {
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
