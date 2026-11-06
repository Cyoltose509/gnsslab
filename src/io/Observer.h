#pragma once
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>

#include "EphemerisTable.h"
#include "Exception.h"
#include "GnssStruct.h"

#include "novatel/LogReader.h"
#include "novatel/AsciiReader.h"
#include "novatel/BinaryReader.h"
#include "rinex/RINEXObsReader.h"
#include "StringUtils.h"

namespace Observer {
    enum class Format {
        RINEX,
        NovAtel,
        Unknown
    };

    [[nodiscard]] constexpr bool isRINEX(const Format f) { return f == Format::RINEX; }

    [[nodiscard]] inline bool looksLikeAsciiLogLine(const std::string_view line) {
        if (line.size() < 8) return false;
        if (line.front() != '#' && line.front() != '%') return false;
        const std::size_t comma = line.find(',');
        return comma != std::string_view::npos && comma < 32;
    }

    [[nodiscard]] inline double obsVersionFromHeaderLine(const std::string &line) {
        const RinexVersionType vt = parseRinexVersionType(line);
        if (const std::string &type = vt.type;
            !type.empty() && type != "O" && type != "OBSERVATION DATA" && type != "OBSERVATION") return 0.0;
        return vt.version >= 2.0 ? vt.version : 0.0;
    }

    struct Sniff {
        bool asciiLog = false;
        double rinexVersion = 0.0;
    };

    inline Sniff sniffObsStream(std::istream &in) {
        const std::streampos start = in.tellg();
        char buf[4096];
        in.read(buf, sizeof(buf));
        const std::streamsize got = in.gcount();
        in.clear();
        if (start != std::streampos(-1)) in.seekg(start);

        Sniff snack;
        const std::string_view text(buf, static_cast<std::size_t>(got > 0 ? got : 0));
        std::size_t pos = 0;
        for (int n = 0; n < 8 && pos < text.size(); ++n) {
            std::size_t eol = text.find('\n', pos);
            if (eol == std::string_view::npos) eol = text.size();
            std::string line(text.substr(pos, eol - pos));
            pos = eol + 1;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue; // 容忍前导空行

            if (looksLikeAsciiLogLine(line)) {
                snack.asciiLog = true;
                break;
            }
            if (line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
            if (rinexLabel(line) != "RINEX VERSION / TYPE") continue;

            snack.rinexVersion = obsVersionFromHeaderLine(line);
            break;
        }
        return snack;
    }

    inline Format classifyObsStream(std::istream &in) {
        const auto [asciiLog, rinexVersion] = sniffObsStream(in);
        if (asciiLog) return Format::NovAtel;
        return rinexVersion > 0.0 ? Format::RINEX : Format::Unknown;
    }

    inline Format classifyObsPath(const std::string &path) {
        std::ifstream in(path, std::ios::binary);
        if (!in) return Format::Unknown;

        if (const Format fmt = classifyObsStream(in); fmt != Format::Unknown) return fmt;

        const bool novatelSuffix = Str::endsWithIC(path, ".log") || Str::endsWithIC(path, ".oem7") ||
                                   Str::endsWithIC(path, ".gps") || Str::endsWithIC(path, ".gnss");
        return novatelSuffix ? Format::NovAtel : Format::Unknown;
    }

    inline std::unique_ptr<RINEXObsReader> makeRINEXReader(std::fstream &fs) {
        const double ver = sniffObsStream(fs).rinexVersion;
        std::unique_ptr<RINEXObsReader> reader =
                ver >= 2.0 && ver < 3.0
                    ? std::unique_ptr<RINEXObsReader>(new RINEX2ObsReader())
                    : std::unique_ptr<RINEXObsReader>(new RINEX3ObsReader());
        reader->attach(fs);
        return reader;
    }

    inline std::unique_ptr<NovAtel::LogReader> makeNovAtelReader(const std::string &path) {
        std::ifstream in(path, std::ios::binary);
        char c = 0;
        while (in.get(c) && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) {
        }
        if (c == '#' || c == '%') return std::make_unique<NovAtel::AsciiReader>();
        return std::make_unique<NovAtel::BinaryReader>();
    }

    struct Item {
        ObsData obs;
        std::shared_ptr<EphemerisTable> eph;
    };

    struct Source {
        virtual ~Source() = default;

        virtual bool next(Item &out) = 0;
    };

    struct RINEXFile : Source {
        std::fstream f;
        std::unique_ptr<RINEXObsReader> rdr;

        bool open(const std::string &path) {
            f.open(path, std::ios::in);
            if (!f) return false;
            rdr = makeRINEXReader(f);
            return true;
        }

        bool next(Item &out) override {
            try {
                out.obs = rdr->parseRINEXObs();
            } catch (const EndOfFile &) {
                return false;
            } catch (const std::exception &e) {
                std::cerr << "RINEX 解析错误: " << e.what() << "\n";
                return false;
            } catch (...) {
                return false;
            }
            out.eph = nullptr;
            return true;
        }
    };

    struct NovAtelFile : Source {
        std::unique_ptr<NovAtel::LogReader> reader;

        bool open(const std::string &path) {
            reader = makeNovAtelReader(path);
            return reader && reader->open(path);
        }

        bool next(Item &out) override {
            if (!reader->getNextEpoch(out.obs)) return false;
            out.eph = reader->ephSnapshot();
            return true;
        }
    };
}
