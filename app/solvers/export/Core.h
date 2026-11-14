#pragma once

#include <array>
#include <iosfwd>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

#include "TimeStruct.h"

namespace Solvers {
    struct Task;

    enum class Mode;

    namespace Export {
        enum class Format { CSV, POS, SP3, ORB, TRAJ, KML, SBET };
        enum class Field : unsigned {
            Week, Tow, Stat,
            X, Y, Z, RefX, RefY, RefZ, Lat, Lon, H,
            LatF, LonF, Hf, LatK, LonK, Hk,
            E, N, U,
            Xf, Yf, Zf, Ef, Nf, Uf, SigmaPf,
            Xk, Yk, Zk, Ek, Nk, Uk, SigmaPk,
            BaseXf, BaseYf, BaseZf, BaseLenF, BaseEf, BaseNf, BaseUf,
            BaseXk, BaseYk, BaseZk, BaseLenK, BaseEk, BaseNk, BaseUk,
            Vx, Vy, Vz, Vn, Ve, Vd,
            Pdop, Gdop, Hdop, Vdop, Tdop,
            SigmaP, SigmaV, SigmaE, SigmaN, SigmaU, Sats, Ratio, Rms3dF, Rms3dK,
            R, T, Nrtn,
            SmA, Ecc, Inc, Raan, Argp, MAnom, PerAlt, ApoAlt, Period,
            Roll, Pitch, Heading,
            ZuptFlag, ZuptLat, ZuptLon, ZuptH, ZuptVn, ZuptVe, ZuptVd,
            ZuptRoll, ZuptPitch, ZuptHeading, ZuptE, ZuptN,
            Axb, Ayb, Azb, Wxb, Wyb, Wzb,
            Count
        };

        constexpr unsigned kFieldCount = static_cast<unsigned>(Field::Count);

        enum class Kind { Int, Sig, Fixed, Text };

        struct FieldDef {
            const char *name;
            const char *key;
            Kind kind;
            int prec;
        };

        const FieldDef &fieldDef(Field f);

        std::optional<Format> formatByKey(std::string_view key);

        std::optional<Field> fieldByKey(std::string_view key);

        struct Col {
            Field field{};
            const char *name{};
            const char *group = nullptr;
        };

        struct FieldGroup {
            const char *label;
            std::vector<Col> cols;
        };

        /// RTK 传 withBaseline = true 追加基线列（基线相对基准站，与真值无关）。
        std::vector<Col> floatFixColumns(bool withRtn, bool withBaseline = false);

        std::vector<Col> withoutRefColumns(std::vector<Col> cols);

        std::string civilText(const CivilTime &c, char dateSep, char mid, const char *tail);

        struct Stamp {
            unsigned week = 0;
            double tow = 0.0;
            [[nodiscard]] double abs() const { return static_cast<double>(week) * 604800.0 + tow; }
        };

        struct Row {
            Stamp stamp;
            bool solved = false;
            int stat = 0;
            std::array<double, kFieldCount> v{};

            Row() { v.fill(nan()); }

            static constexpr double nan() { return std::numeric_limits<double>::quiet_NaN(); }

            void reset() {
                stamp = Stamp{};
                solved = false;
                stat = 0;
                v.fill(nan());
            }

            void set(const Field f, const double x) { v[static_cast<unsigned>(f)] = x; }

            [[nodiscard]] double get(const Field f) const { return v[static_cast<unsigned>(f)]; }

            [[nodiscard]] bool has(const Field f) const { return !std::isnan(get(f)); }
        };

        struct Sampling {
            enum class Mode { Native, Seconds, Step };

            Mode mode = Mode::Native;
            double seconds = 30.0;
            int step = 1;
        };

        enum class PosForm { BLH, ECEF, ENU };

        struct Spec {
            Format format = Format::CSV;
            Sampling sampling{};
            std::vector<Field> fields;
            std::size_t begin = 0;
            std::size_t end = 0;
            std::string satId = "L01";
            std::string marker = "GnssLab";
            bool sp3Velocity = false;
            PosForm posForm = PosForm::BLH;
            bool posVelocity = true;
            bool posHeader = true;
        };

        /// 周内秒 → 日历分量
        CivilTime toCivil(const Stamp &s);

        /// 瞬根数：长度 m、角度 rad、周期 s
        struct Elements {
            double a = 0, e = 0, inc = 0, raan = 0, argp = 0, mAnom = 0;
            double perAlt = 0, apoAlt = 0, period = 0;
        };

        Elements elementsFromState(const Eigen::Vector3d &rEci, const Eigen::Vector3d &vEci);

        std::vector<std::size_t> pick(const Task &task, const Spec &spec);

        void write(std::ostream &out, const Task &task, const Spec &spec);

        std::vector<Format> formatsFor(Mode mode);

        std::vector<FieldGroup> groupsFor(const std::vector<Col> &cols);

        std::vector<Col> resolveColumns(const Spec &spec, Format fmt, const Task &task);

        [[nodiscard]] bool formatSolves(Format fmt, Mode mode);

        [[nodiscard]] const char *fileExt(Format fmt);

        [[nodiscard]] const char *formatName(Format fmt);

        [[nodiscard]] bool formatNeedsGrid(Format fmt);
    }
}
