#pragma once

#include <array>
#include <fstream>
#include <string>
#include <vector>

/// Compact RINEX（Hatanaka）解压：把 .crx / .??d 还原成标准 RINEX 观测文件。
/// 算法依据 Hatanaka, Y. (2008), A Compression Format and Tools for GNSS Observation Data,
/// Bulletin of the Geospatial Information Authority of Japan, 55, 21-30.
class CRXDecoder {
public:
    static bool isCompactRINEX(const std::string &path);

    bool decode(const std::string &inPath, const std::string &outPath, std::string &err);

private:
    static constexpr int kMaxDiffOrder = 5;

    struct Diff {
        std::array<long long, kMaxDiffOrder + 1> u{};
        std::array<long long, kMaxDiffOrder + 1> l{};
        int order = -1;
        int arcOrder = -1;
    };

    struct Clock {
        std::array<long long, kMaxDiffOrder + 1> u{};
        std::array<long long, kMaxDiffOrder + 1> l{};
    };

    bool fail(const std::string &msg);
    bool readLine(std::string &s);
    bool readHeader();
    void parseHeaderTypes(const std::string &s);
    bool runEpochs();
    bool handleEvent(std::string &dline);
    bool processEpoch(const std::string &dline);
    [[nodiscard]] bool validEpochLine() const;
    bool parseClock(const std::string &s, bool &haveClock);
    void processClock();
    bool parseSatDiff(const std::string &s, int sat, int prevIdx, int nt);
    void emitEpochLine();
    void emitSatData();
    void saveState();

    [[nodiscard]] int typesOf(const std::string &satId) const;

    static std::string formatField(const Diff &d, char f0, char f1);
    [[nodiscard]] std::string formatClock(long long u, long long l) const;

    std::ifstream in;
    std::ofstream out;
    std::string error;
    std::string line;
    std::string pending;
    bool hasPending = false;

    int rinexVersion = 0;
    int crinexVersion = 0;
    int crinexFull = 0;
    int ntype = 0;
    std::array<int, 256> ntypeGnss{};
    int nsat = 0;
    int nsatPrev = 0;
    int clkOrder = 0;
    int clkArcOrder = 0;
    char epTopFrom = 0;
    char epTopTo = 0;
    std::size_t eventPos = 0;
    std::size_t nSatPos = 0;
    std::size_t satLstPos = 0;
    std::size_t offset = 0;
    int shiftClk = 0;
    std::size_t lineNo = 0;

    std::vector<std::string> satIds;
    std::vector<std::string> satPrev;
    std::vector<int> satTable;
    std::vector<std::string> dflag;
    std::vector<std::string> flagCur;
    std::vector<std::string> flagPrev;
    std::vector<std::vector<Diff> > diffCur;
    std::vector<std::vector<Diff> > diffPrev;
    Clock clkCur;
    Clock clkPrev;
    std::string buf;
};
