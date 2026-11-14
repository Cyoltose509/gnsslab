#include "product/sp3/Writer.h"

#include <cmath>
#include <iomanip>
#include <ostream>

#include "TimeStruct.h"

namespace {
    /// SP3 没有钟差数据时的哨兵值
    constexpr double kNoClock = 999999.999999;

    std::string svName(const std::string &id) {
        std::string s = id;
        if (s.size() > 3) s.resize(3);
        if (s.size() < 3) s.append(3 - s.size(), ' ');
        return s;
    }

    void writeEpochLine(std::ostream &out, const CivilTime &c) {
        out << "*  "
                << std::setw(4) << c.year
                << std::setw(3) << c.month
                << std::setw(3) << c.day
                << std::setw(3) << c.hour
                << std::setw(3) << c.minute
                << std::fixed << std::setprecision(8) << std::setw(12) << c.second
                << '\n';
    }

    void writeDataLine(std::ostream &out, const char *tag, const std::string &sv,
                       const Eigen::Vector3d &xyz) {
        out << tag << sv
                << std::fixed << std::setprecision(6)
                << std::setw(14) << xyz.x()
                << std::setw(14) << xyz.y()
                << std::setw(14) << xyz.z()
                << std::setw(14) << kNoClock
                << '\n';
    }

    void writeSatTable(std::ostream &out, const std::string &sv) {
        for (int line = 0; line < 5; ++line) {
            out << "+  ";
            if (line == 0) {
                out << std::setw(3) << 1 << "  " << sv << ' ';
            } else {
                out << "                           ";
            }
            out << '\n';
        }
        for (int line = 0; line < 5; ++line) {
            out << "++ ";
            for (int k = 0; k < 5; ++k) out << std::setw(3) << 0 << ' ';
            out << '\n';
        }
    }
}

namespace SP3 {
    void Writer::write(std::ostream &out, const Header &header, const std::vector<Record> &records) {
        const std::string sv = svName(header.satId);
        const std::string marker = header.marker.empty() ? "gnssLab" : header.marker;
        const std::string frame = Frame::refName(header.frame);
        const std::string comment = header.comment.empty() ? "kinematic orbit" : header.comment;

        CivilTime first = header.epoch;

        out << "#cP";
        out << std::setw(4) << first.year << std::setw(3) << first.month << std::setw(3) << first.day
                << std::setw(3) << first.hour << std::setw(3) << first.minute
                << std::fixed << std::setprecision(8) << std::setw(12) << first.second
                << std::setw(7) << static_cast<long long>(records.size()) << " ORBIT " << frame
                << " HLM  " << marker << '\n';

        // ## 的第 5 个字段是 MJD 的小数部分（当天已过秒数 / 86400）。
        const long mjd = first.toCommonTime().m_day;
        const double fracDay = (first.hour * 3600.0 + first.minute * 60.0 + first.second) / 86400.0;
        out << "## " << header.week
                << std::fixed << std::setprecision(8) << ' ' << header.sow
                << ' ' << header.interval
                << ' ' << mjd
                << ' ' << std::setprecision(13) << fracDay << '\n';

        writeSatTable(out, sv);
        out << "%c M  cc GPS ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n";
        out << "%c cc cc ccc ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n";
        out << "%f  1.2500000  1.025000000  0.00000000000  0.000000000000000\n";
        out << "%f  0.0000000  0.000000000  0.00000000000  0.000000000000000\n";
        out << "%i    0    0    0    0      0      0      0      0         0\n";
        out << "%i    0    0    0    0      0      0      0      0         0\n";
        out << "/* " << marker << ' ' << comment << '\n';
        out << "/* " << std::setw(4) << std::setfill('0') << first.year << ' '
                << std::setw(2) << first.month << ' ' << std::setw(2) << first.day << ' '
                << std::setw(2) << first.hour << ':' << std::setw(2) << first.minute << ':'
                << std::fixed << std::setprecision(8) << std::setw(11) << first.second
                << " GPST" << std::setfill(' ') << '\n';

        for (const auto &[epoch, pos, vel]: records) {
            writeEpochLine(out, epoch);
            writeDataLine(out, "P", sv, pos * 1.0e-3);
            if (header.hasVel) writeDataLine(out, "V", sv, vel * 10.0);
        }
    }
}
