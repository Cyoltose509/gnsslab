#include "Observer.h"

#include <algorithm>
#include <fstream>
#include <istream>
#include <string_view>

#include "StringUtils.h"
#include "novatel/AsciiReader.h"
#include "novatel/BinaryReader.h"
#include "core/TimeStruct.h"
#include "i18n/I18n.h"

namespace {
    using Observer::DateFrom;
    using Observer::Format;
    using Observer::ObsDate;

    /// 1 Hz 超过 55 h：RINEX 解析器在残缺尾部可能不抛 EOF，靠这个上限收手。
    constexpr int kRINEXEpochLimit = 200000;

    bool looksLikeAsciiLogLine(const std::string_view line) {
        if (line.size() < 8) return false;
        if (line.front() != '#' && line.front() != '%') return false;
        const std::size_t comma = line.find(',');
        return comma != std::string_view::npos && comma < 32;
    }

    double obsVersionFromHeaderLine(const std::string &line) {
        const RinexVersionType vt = parseRinexVersionType(line);
        if (const std::string &type = vt.type;
            !type.empty() && type != "O" && type != "OBSERVATION DATA" && type != "OBSERVATION")
            return 0.0;
        return vt.version >= 2.0 ? vt.version : 0.0;
    }

    bool novatelSuffix(const std::string &path) {
        return Str::endsWithIC(path, ".log") || Str::endsWithIC(path, ".oem7") ||
               Str::endsWithIC(path, ".gps") || Str::endsWithIC(path, ".gnss");
    }

    /// 头里的 TIME OF FIRST OBS：只读前 400 行，比解一条历元便宜，文件里一条观测都没有时也还在。
    /// 一行超过 RINEX 的 80 列说明根本不是文本头（二进制日志），立刻收手。
    bool headerDate(const std::string &path, ObsDate &out) {
        std::ifstream in(path, std::ios::binary);
        if (!in) return false;
        char buf[512];
        for (int n = 0; n < 400; ++n) {
            in.getline(buf, sizeof buf);
            if (in.fail()) return false;
            const std::string line(buf);
            if (line.find("END OF HEADER") != std::string::npos) return false;
            if (line.size() < 60) continue;
            if (Str::strip(Str::sub(line, 60, 20)) != "TIME OF FIRST OBS") continue;

            double v[3]{0, 0, 0};
            if (Str::scanDoubles(line.c_str(), v, 3) < 3) return false;
            int year = static_cast<int>(v[0]);
            const int month = static_cast<int>(v[1]);
            const int day = static_cast<int>(v[2]);
            if (year >= 0 && year < 80) year += 2000;
            else if (year < 100) year += 1900;
            if (month < 1 || month > 12 || day < 1 || day > 31) return false;
            out.year = year;
            out.doy = YearMonthDay{year, month, day}.toDOY();
            out.from = DateFrom::Header;
            return out.doy > 0;
        }
        return false;
    }
}

namespace Observer {
    Sniff sniffObsStream(std::istream &in) {
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

    Format classifyObsStream(std::istream &in) {
        const auto [asciiLog, rinexVersion] = sniffObsStream(in);
        if (asciiLog) return Format::NovAtel;
        return rinexVersion > 0.0 ? Format::RINEX : Format::Unknown;
    }

    Format classifyObsPath(const std::string &path) {
        std::ifstream in(path, std::ios::binary);
        if (!in) return Format::Unknown;

        if (const Format fmt = classifyObsStream(in); fmt != Format::Unknown) return fmt;
        return novatelSuffix(path) ? Format::NovAtel : Format::Unknown;
    }

    std::unique_ptr<RINEXObsReader> makeRINEXReader(const double version) {
        if (version >= 2.0 && version < 3.0) return std::make_unique<RINEX2ObsReader>();
        return std::make_unique<RINEX3ObsReader>();
    }

    std::unique_ptr<RINEXObsReader> makeRINEXReader(std::fstream &fs) {
        std::unique_ptr<RINEXObsReader> reader = makeRINEXReader(sniffObsStream(fs).rinexVersion);
        reader->attach(fs);
        return reader;
    }

    std::unique_ptr<NovAtel::LogReader> makeNovAtelReader(const std::string &path) {
        std::ifstream in(path, std::ios::binary);
        char c = 0;
        while (in.get(c) && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) {
        }
        if (c == '#' || c == '%') return std::make_unique<NovAtel::AsciiReader>();
        return std::make_unique<NovAtel::BinaryReader>();
    }

    bool ObsStream::open(const std::string &fpath, std::string *err,
                         const std::map<std::string, int> *gloSlots) {
        const auto fail = [&](const char *msg) {
            if (err) *err = msg;
            fmt = Format::Unknown;
            return false;
        };

        path = fpath;
        // 一次嗅探同时拿到「格式」与「RINEX 版本」，别为同一份 4 KB 头开三遍文件。
        double version = 0.0;
        {
            std::ifstream in(fpath, std::ios::binary);
            if (!in) return fail(I18n::trc("io.obs.open_failed"));
            const Sniff snack = sniffObsStream(in);
            if (snack.asciiLog) {
                fmt = Format::NovAtel;
            } else if (snack.rinexVersion > 0.0) {
                fmt = Format::RINEX;
                version = snack.rinexVersion;
            } else {
                fmt = novatelSuffix(fpath) ? Format::NovAtel : Format::Unknown;
            }
        }
        if (fmt == Format::Unknown) return fail(I18n::trc("io.err.bad_format"));

        if (fmt == Format::RINEX) {
            if (version <= 0.0) return fail(I18n::trc("io.obs.rinex_no_version"));
            rnx = makeRINEXReader(version);
            if (gloSlots) rnx->gloSlots = *gloSlots;
            try {
                rnx->openFile(fpath);
            } catch (const std::exception &e) {
                rnx.reset();
                return fail(e.what());
            }
            return true;
        }

        oem = makeNovAtelReader(fpath);
        if (!oem || !oem->open(fpath)) {
            oem.reset();
            return fail(I18n::trc("io.obs.novatel_open_failed"));
        }
        return true;
    }

    bool ObsStream::pull(Item &out) {
        if (fmt == Format::RINEX) {
            if (++rinexEpochs >= kRINEXEpochLimit) {
                limitHit = true;
                return false;
            }
            try {
                out.obs = rnx->parseRINEXObs();
            } catch (const EndOfFile &) {
                return false;
            }
            out.eph = nullptr;
            return true;
        }
        if (fmt == Format::NovAtel) {
            if (!oem->getNextEpoch(out.obs)) return false;
            out.eph = oem->ephSnapshot();
            return true;
        }
        return false;
    }

    bool ObsStream::next(Item &out) {
        if (hasPending) {
            out = std::move(pending);
            hasPending = false;
            return true;
        }
        return pull(out);
    }

    bool ObsStream::peek(Item &out) {
        if (!hasPending) {
            if (!pull(pending)) return false;
            hasPending = true;
        }
        out = pending;
        return true;
    }

    void ObsStream::fastForward(const CommonTime &t) const {
        if (rnx) rnx->fastForwardToEpoch(t);
    }

    bool ObsStream::firstDate(ObsDate &out) {
        try {
            if (fmt == Format::RINEX && headerDate(path, out)) return true;

            Item it;
            if (!peek(it)) return false;
            const YDSTime yds = it.obs.epoch.toYDS();
            const WeekSecond ws = it.obs.epoch.toWeekSecond();
            if (yds.year < 1980 || yds.doy < 1) return false;
            out = {yds.year, yds.doy, static_cast<int>(ws.week), ws.sow, DateFrom::Epoch};
            return true;
        } catch (...) { return false; }
    }

    bool probeObsDate(const std::string &path, ObsDate &out) {
        ObsStream s;
        if (!s.open(path)) return false;
        return s.firstDate(out);
    }

    bool probeAntenna(const std::string &path, AntennaInfo &out) {
        std::ifstream in(path);
        if (!in) return false;
        out = {};
        std::string line;
        while (std::getline(in, line)) {
            const std::string label = line.size() > 60 ? Str::strip(line.substr(60)) : std::string();
            if (label.rfind("END OF HEADER", 0) == 0) break;
            if (label.rfind("MARKER NAME", 0) == 0) {
                std::string s = Str::strip(Str::sub(line, 0, 60));
                std::replace(s.begin(), s.end(), ' ', '_');
                out.station = Str::strip(s);
            } else if (label.rfind("ANT # / TYPE", 0) == 0) {
                out.serial = Str::strip(Str::sub(line, 0, 20));
                const std::string type = Str::strip(Str::sub(line, 20, 20));
                const std::string radome = Str::strip(Str::sub(line, 40, 20));
                if (!type.empty()) out.type = radome.empty() ? type : type + " " + radome;
            } else if (label.rfind("ANTENNA: DELTA H/E/N", 0) == 0) {
                out.deltaENU = Eigen::Vector3d(Str::toDouble(Str::sub(line, 14, 14)),
                                               Str::toDouble(Str::sub(line, 28, 14)),
                                               Str::toDouble(Str::sub(line, 0, 14)));
            }
        }
        return !out.type.empty();
    }
}
