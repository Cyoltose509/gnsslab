#pragma once
#include <fstream>
#include <string>
#include <algorithm>
#include "StringUtils.h"
#include "Exception.h"

class RinexFileBase {
public:
    RinexFileBase() = default;

    virtual ~RinexFileBase() = default;

    void openFile(const std::string &path) {
        if (ownedStream.is_open()) ownedStream.close();
        ownedStream.open(path, std::ios::in);
        if (!ownedStream) throw FFStreamError("can't open file: " + path);
        pStream = &ownedStream;
    }

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

    static std::string getLabel(const std::string &line) {
        return strip(safeSubstr(line, 60, 20));
    }

    template<class Cb>
    void scanLabeledHeader(Cb &&cb) {
        std::string line;
        while (true) {
            if (!nextLine(line)) throw FFStreamError("Unexpected EOF in RINEX header");
            const std::string label = getLabel(line);
            if (label == "END OF HEADER") break;
            cb(label, line);
        }
    }

private:
    std::fstream ownedStream;
};
