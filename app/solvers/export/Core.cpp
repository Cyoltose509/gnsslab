#include "Core.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "solvers/common/Config.h"
#include "solvers/common/Task.h"
#include "TimeStruct.h"

namespace Solvers::Export {
    namespace {
        constexpr FieldDef kDefs[] = {
            {"Wk", "wk", Kind::Int, 0},
            {"SOW", "sow", Kind::Fixed, 3},
            {"Status", "stat", Kind::Text, 0},
            {"ECEF-X/m", "x", Kind::Sig, 11},
            {"ECEF-Y/m", "y", Kind::Sig, 11},
            {"ECEF-Z/m", "z", Kind::Sig, 11},
            {"REF-X/m", "refx", Kind::Sig, 11},
            {"REF-Y/m", "refy", Kind::Sig, 11},
            {"REF-Z/m", "refz", Kind::Sig, 11},
            {"B/deg", "lat", Kind::Sig, 10},
            {"L/deg", "lon", Kind::Sig, 10},
            {"H/m", "h", Kind::Sig, 4},
            {"Float-B/deg", "fblat", Kind::Sig, 10},
            {"Float-L/deg", "fblon", Kind::Sig, 10},
            {"Float-H/m", "fbh", Kind::Sig, 4},
            {"Fix-B/deg", "kblat", Kind::Sig, 10},
            {"Fix-L/deg", "kblon", Kind::Sig, 10},
            {"Fix-H/m", "kbh", Kind::Sig, 4},
            {"EAST/m", "e", Kind::Sig, 6},
            {"NORTH/m", "n", Kind::Sig, 6},
            {"UP/m", "u", Kind::Sig, 6},
            {"Float-X/m", "fx", Kind::Sig, 11},
            {"Float-Y/m", "fy", Kind::Sig, 11},
            {"Float-Z/m", "fz", Kind::Sig, 11},
            {"Float-E/m", "fe", Kind::Sig, 6},
            {"Float-N/m", "fn", Kind::Sig, 6},
            {"Float-U/m", "fu", Kind::Sig, 6},
            {"Float-SigmaP/m", "spfloat", Kind::Sig, 4},
            {"Fix-X/m", "kx", Kind::Sig, 11},
            {"Fix-Y/m", "ky", Kind::Sig, 11},
            {"Fix-Z/m", "kz", Kind::Sig, 11},
            {"Fix-E/m", "ke", Kind::Sig, 6},
            {"Fix-N/m", "kn", Kind::Sig, 6},
            {"Fix-U/m", "ku", Kind::Sig, 6},
            {"Fix-SigmaP/m", "spfix", Kind::Sig, 4},
            {"Bl-X/m", "bfx", Kind::Sig, 6},
            {"Bl-Y/m", "bfy", Kind::Sig, 6},
            {"Bl-Z/m", "bfz", Kind::Sig, 6},
            {"Bl-Len/m", "bfl", Kind::Sig, 4},
            {"Bl-E/m", "bfe", Kind::Sig, 6},
            {"Bl-N/m", "bfn", Kind::Sig, 6},
            {"Bl-U/m", "bfu", Kind::Sig, 6},
            {"Bl-Xk/m", "bkx", Kind::Sig, 6},
            {"Bl-Yk/m", "bky", Kind::Sig, 6},
            {"Bl-Zk/m", "bkz", Kind::Sig, 6},
            {"Bl-Lenk/m", "bkl", Kind::Sig, 4},
            {"Bl-Ek/m", "bke", Kind::Sig, 6},
            {"Bl-Nk/m", "bkn", Kind::Sig, 6},
            {"Bl-Uk/m", "bku", Kind::Sig, 6},
            {"Vel-X/m", "vx", Kind::Sig, 6},
            {"Vel-Y/m", "vy", Kind::Sig, 6},
            {"Vel-Z/m", "vz", Kind::Sig, 6},
            {"VN/m", "vn", Kind::Sig, 6},
            {"VE/m", "ve", Kind::Sig, 6},
            {"VD/m", "vd", Kind::Sig, 6},
            {"PDOP", "pdop", Kind::Sig, 4},
            {"GDOP", "gdop", Kind::Sig, 4},
            {"HDOP", "hdop", Kind::Sig, 4},
            {"VDOP", "vdop", Kind::Sig, 4},
            {"TDOP", "tdop", Kind::Sig, 4},
            {"SigmaP", "sp", Kind::Sig, 4},
            {"SigmaV", "sv", Kind::Sig, 4},
            {"SigmaE", "se", Kind::Sig, 4},
            {"SigmaN", "sn", Kind::Sig, 4},
            {"SigmaU", "su", Kind::Sig, 4},
            {"SatCount", "sats", Kind::Int, 0},
            {"Ratio", "ratio", Kind::Sig, 6},
            {"Float-3dRMS/m", "rms3df", Kind::Sig, 6},
            {"Fix-3dRMS/m", "rms3dk", Kind::Sig, 6},
            {"R/m", "r", Kind::Sig, 6},
            {"T/m", "t", Kind::Sig, 6},
            {"N/m", "nrtn", Kind::Sig, 6},
            {"a/m", "sma", Kind::Fixed, 3},
            {"e", "ecc", Kind::Fixed, 8},
            {"i/deg", "inc", Kind::Fixed, 6},
            {"RAAN/deg", "raan", Kind::Fixed, 6},
            {"argp/deg", "argp", Kind::Fixed, 6},
            {"M/deg", "ma", Kind::Fixed, 6},
            {"hp/km", "hp", Kind::Fixed, 3},
            {"ha/km", "ha", Kind::Fixed, 3},
            {"T/s", "period", Kind::Fixed, 3},
            {"Roll/deg", "roll", Kind::Fixed, 6},
            {"Pitch/deg", "pitch", Kind::Fixed, 6},
            {"Heading/deg", "heading", Kind::Fixed, 6},
            {"ZuptApplied", "zupt", Kind::Int, 0},
            {"ZuptB/deg", "zuptlat", Kind::Sig, 10},
            {"ZuptL/deg", "zuptlon", Kind::Sig, 10},
            {"ZuptH/m", "zupth", Kind::Sig, 4},
            {"ZuptVN/m", "zuptvn", Kind::Sig, 6},
            {"ZuptVE/m", "zuptve", Kind::Sig, 6},
            {"ZuptVD/m", "zuptvd", Kind::Sig, 6},
            {"ZuptRoll/deg", "zuptroll", Kind::Sig, 6},
            {"ZuptPitch/deg", "zuptpitch", Kind::Sig, 6},
            {"ZuptHeading/deg", "zuptheading", Kind::Sig, 6},
            {"ZuptE/m", "zupte", Kind::Sig, 6},
            {"ZuptN/m", "zuptn", Kind::Sig, 6},
            {"Axb/(m/s2)", "axb", Kind::Fixed, 6},
            {"Ayb/(m/s2)", "ayb", Kind::Fixed, 6},
            {"Azb/(m/s2)", "azb", Kind::Fixed, 6},
            {"Wxb/(rad/s)", "wxb", Kind::Fixed, 8},
            {"Wyb/(rad/s)", "wyb", Kind::Fixed, 8},
            {"Wzb/(rad/s)", "wzb", Kind::Fixed, 8},
        };
        static_assert(std::size(kDefs) == kFieldCount, "字段表与 Field 枚举数量不一致");

        /// 导出弹窗勾选项的中文名：键是成组列的 Col::group 或单列的 CSV 表头名。
        /// 只登记「显示名与 CSV 名不同」的条目，PDOP/Ratio 这类无需翻译的查不到原样显示；
        /// CSV 表头以 Col::name 为准，这张表只管界面。
        constexpr struct { const char *key; const char *ui; } kUiLabels[] = {
            {"Wk", "周数"},
            {"SOW", "周内秒"},
            {"GPSTow", "周内秒"},
            {"Status", "解状态"},
            {"SigmaP", "位置 σ"},
            {"SigmaV", "速度 σ"},
            {"Float-SigmaP/m", "浮点解 σ"},
            {"Fix-SigmaP/m", "固定解 σ"},
            {"SatCount", "卫星数"},
            {"3dRMS/m", "三维 RMS"},
            {"Float-3dRMS/m", "浮点三维 RMS"},
            {"Fix-3dRMS/m", "固定三维 RMS"},
            {"ZuptApplied", "零速修正标记"},
            {"ECEF-XYZ", "坐标 (XYZ)"},
            {"REF-XYZ", "参考坐标 (XYZ)"},
            {"ENU", "站心偏差 (E/N/U)"},
            {"BLH", "经纬高 (B/L/H)"},
            {"Float-BLH", "浮点解经纬高 (B/L/H)"},
            {"Fix-BLH", "固定解经纬高 (B/L/H)"},
            {"Float-XYZ", "浮点解坐标 (XYZ)"},
            {"Float-ENU", "浮点解偏差 (E/N/U)"},
            {"Fix-XYZ", "固定解坐标 (XYZ)"},
            {"Fix-ENU", "固定解偏差 (E/N/U)"},
            {"Bl-XYZ-f", "浮点解基线 (X/Y/Z)"},
            {"Bl-ENU-f", "浮点解基线 (E/N/U)"},
            {"Bl-Len/m", "浮点解基线长度"},
            {"Bl-XYZ-k", "固定解基线 (X/Y/Z)"},
            {"Bl-ENU-k", "固定解基线 (E/N/U)"},
            {"Bl-Lenk/m", "固定解基线长度"},
            {"Vel-XYZ", "速度 (X/Y/Z)"},
            {"Vel-NED", "速度 (N/E/D)"},
            {"RTN", "轨道偏差 (R/T/N)"},
            {"RPH", "姿态角 (R/P/H)"},
            {"EN", "水平位置 (E/N)"},
            {"Zupt-BLH", "零速修正 经纬高 (B/L/H)"},
            {"Zupt-NED", "零速修正 速度 (N/E/D)"},
            {"Zupt-RPH", "零速修正 姿态角 (R/P/H)"},
            {"Zupt-EN", "零速修正 水平位置 (E/N)"},
            {"Accel", "加计零偏 (X/Y/Z)"},
            {"Gyro", "陀螺零偏 (X/Y/Z)"},
        };

        /// 组名 / 单列 CSV 名 → 中文名
        const char *uiLabel(const char *key) {
            for (const auto &[k, name]: kUiLabels)
                if (std::strcmp(k, key) == 0) return name;
            return key;
        }

        std::string lower(const std::string_view s) {
            std::string out(s);
            for (char &c: out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return out;
        }

        /// 抽稀网格：从区间首历元起按固定秒数往后取，末历元必留。
        std::vector<std::size_t> pickOnGrid(const Task &task, const std::size_t b, const std::size_t e,
                                            const double step, const bool padEnd) {
            std::vector<std::size_t> out;
            if (e <= b) return out;
            const double t0 = task.exportStamp(b).abs();
            double next = t0;
            for (std::size_t i = b; i < e; ++i) {
                const double t = task.exportStamp(i).abs();
                if (t + 1e-6 < next) continue;
                out.push_back(i);
                const double k = std::floor((t - t0) / step + 1e-6) + 1.0;
                next = t0 + k * step;
                if (next <= t + 1e-6) next = t + step;
            }
            // SP3 要求严格等间隔：末历元不在网格上就不能硬塞，否则读取器按 interval 插值会错。
            if (padEnd && !out.empty() && out.back() != e - 1) out.push_back(e - 1);
            return out;
        }
    }

    const FieldDef &fieldDef(const Field f) { return kDefs[static_cast<unsigned>(f)]; }

    std::optional<Format> formatByKey(const std::string_view key) {
        const std::string k = lower(key);
        if (k == "csv") return Format::CSV;
        if (k == "pos") return Format::POS;
        if (k == "sp3") return Format::SP3;
        if (k == "orb" || k == "elements") return Format::ORB;
        if (k == "traj" || k == "txt") return Format::TRAJ;
        if (k == "kml") return Format::KML;
        if (k == "sbet") return Format::SBET;
        return std::nullopt;
    }

    std::optional<Field> fieldByKey(const std::string_view key) {
        const std::string k = lower(key);
        for (unsigned i = 0; i < kFieldCount; ++i)
            if (k == kDefs[i].key) return static_cast<Field>(i);
        return std::nullopt;
    }

    CivilTime toCivil(const Stamp &s) {
        return WeekSecond(s.week, s.tow).toCommonTime().toCivilTime();
    }

    std::vector<std::size_t> pick(const Task &task, const Spec &spec) {
        const std::size_t n = task.exportCount();
        const std::size_t b = std::min(spec.begin, n);
        const std::size_t e = spec.end == 0 ? n : std::min(spec.end, n);
        if (e <= b) return {};
        const auto &[mode, seconds, step] = spec.sampling;
        if (formatNeedsGrid(spec.format) && mode != Sampling::Mode::Seconds)
            return pickOnGrid(task, b, e, 30.0, false);
        if (mode == Sampling::Mode::Step) {
            std::vector<std::size_t> out;
            const std::size_t k = static_cast<std::size_t>(std::max(1, step));
            for (std::size_t i = b; i < e; i += k) out.push_back(i);
            if (!out.empty() && out.back() != e - 1) out.push_back(e - 1);
            return out;
        }
        if (mode == Sampling::Mode::Seconds && seconds > 0.0)
            return pickOnGrid(task, b, e, seconds, spec.format != Format::SP3);
        std::vector<std::size_t> out;
        out.reserve(e - b);
        for (std::size_t i = b; i < e; ++i) out.push_back(i);
        return out;
    }

    const char *fileExt(const Format fmt) {
        switch (fmt) {
            case Format::POS: return "pos";
            case Format::SP3: return "sp3";
            case Format::ORB: return "csv";
            case Format::TRAJ: return "txt";
            case Format::KML: return "kml";
            case Format::SBET: return "sbet";
            default: return "csv";
        }
    }

    const char *formatName(const Format fmt) {
        switch (fmt) {
            case Format::POS: return "POS";
            case Format::SP3: return "SP3";
            case Format::ORB: return "轨道根数";
            case Format::TRAJ: return "轨迹";
            case Format::KML: return "KML";
            case Format::SBET: return "SBET";
            default: return "CSV";
        }
    }

    std::vector<Format> formatsFor(const Mode mode) {
        switch (mode) {
            case Mode::LEO: return {Format::CSV, Format::POS, Format::SP3, Format::ORB, Format::KML};
            case Mode::INS: return {Format::CSV, Format::TRAJ, Format::KML, Format::SBET};
            default: return {Format::CSV, Format::POS, Format::SP3, Format::KML};
        }
    }

    bool formatSolves(const Format fmt, const Mode mode) {
        const auto all = formatsFor(mode);
        return std::find(all.begin(), all.end(), fmt) != all.end();
    }

    bool formatNeedsGrid(const Format fmt) { return fmt == Format::SP3; }

    std::string civilText(const CivilTime &c, const char dateSep, const char mid, const char *tail) {
        std::ostringstream os;
        os << std::setfill('0') << std::setw(4) << c.year << dateSep << std::setw(2) << c.month << dateSep
                << std::setw(2) << c.day << mid << std::setw(2) << c.hour << ':' << std::setw(2) << c.minute
                << ':' << std::fixed << std::setprecision(3) << std::setw(6) << c.second << tail;
        return os.str();
    }

    std::vector<Col> floatFixColumns(const bool withRtn, const bool withBaseline) {
        using F = Field;
        std::vector<Col> c = {
            {F::Week, "Wk"}, {F::Tow, "SOW"}, {F::Stat, "Status"},
            {F::Xf, "Float-X/m", "Float-XYZ"},
            {F::Yf, "Float-Y/m", "Float-XYZ"},
            {F::Zf, "Float-Z/m", "Float-XYZ"},
            {F::Ef, "Float-E/m", "Float-ENU"},
            {F::Nf, "Float-N/m", "Float-ENU"},
            {F::Uf, "Float-U/m", "Float-ENU"},
            {F::Rms3dF, "Float-3dRMS/m"}, {F::SigmaPf, "Float-SigmaP/m"},
            {F::Xk, "Fix-X/m", "Fix-XYZ"},
            {F::Yk, "Fix-Y/m", "Fix-XYZ"},
            {F::Zk, "Fix-Z/m", "Fix-XYZ"},
            {F::Ek, "Fix-E/m", "Fix-ENU"},
            {F::Nk, "Fix-N/m", "Fix-ENU"},
            {F::Uk, "Fix-U/m", "Fix-ENU"},
            {F::Rms3dK, "Fix-3dRMS/m"}, {F::SigmaPk, "Fix-SigmaP/m"},
            {F::LatF, "Float-B/deg", "Float-BLH"},
            {F::LonF, "Float-L/deg", "Float-BLH"},
            {F::Hf, "Float-H/m", "Float-BLH"},
            {F::LatK, "Fix-B/deg", "Fix-BLH"},
            {F::LonK, "Fix-L/deg", "Fix-BLH"},
            {F::Hk, "Fix-H/m", "Fix-BLH"},
            {F::Pdop, "PDOP"}, {F::Gdop, "GDOP"}, {F::Hdop, "HDOP"},
            {F::Vdop, "VDOP"}, {F::Tdop, "TDOP"},
            {F::Ratio, "Ratio"}, {F::Sats, "SatCount"},
            {F::RefX, "REF-X/m", "REF-XYZ"},
            {F::RefY, "REF-Y/m", "REF-XYZ"},
            {F::RefZ, "REF-Z/m", "REF-XYZ"},
        };
        if (withBaseline) {
            c.push_back({F::BaseXf, "Bl-X/m", "Bl-XYZ-f"});
            c.push_back({F::BaseYf, "Bl-Y/m", "Bl-XYZ-f"});
            c.push_back({F::BaseZf, "Bl-Z/m", "Bl-XYZ-f"});
            c.push_back({F::BaseLenF, "Bl-Len/m"});
            c.push_back({F::BaseEf, "Bl-E/m", "Bl-ENU-f"});
            c.push_back({F::BaseNf, "Bl-N/m", "Bl-ENU-f"});
            c.push_back({F::BaseUf, "Bl-U/m", "Bl-ENU-f"});
            c.push_back({F::BaseXk, "Bl-Xk/m", "Bl-XYZ-k"});
            c.push_back({F::BaseYk, "Bl-Yk/m", "Bl-XYZ-k"});
            c.push_back({F::BaseZk, "Bl-Zk/m", "Bl-XYZ-k"});
            c.push_back({F::BaseLenK, "Bl-Lenk/m"});
            c.push_back({F::BaseEk, "Bl-Ek/m", "Bl-ENU-k"});
            c.push_back({F::BaseNk, "Bl-Nk/m", "Bl-ENU-k"});
            c.push_back({F::BaseUk, "Bl-Uk/m", "Bl-ENU-k"});
        }
        if (withRtn) {
            c.push_back({F::R, "R/m", "RTN"});
            c.push_back({F::T, "T/m", "RTN"});
            c.push_back({F::Nrtn, "N/m", "RTN"});
        }
        c.push_back({F::Vx, "Vel-X/m", "Vel-XYZ"});
        c.push_back({F::Vy, "Vel-Y/m", "Vel-XYZ"});
        c.push_back({F::Vz, "Vel-Z/m", "Vel-XYZ"});
        return c;
    }

    std::vector<Col> withoutRefColumns(std::vector<Col> cols) {
        using F = Field;
        const auto isRef = [](const F f) {
            switch (f) {
                case F::RefX: case F::RefY: case F::RefZ:
                case F::E: case F::N: case F::U:
                case F::Ef: case F::Nf: case F::Uf:
                case F::Ek: case F::Nk: case F::Uk:
                case F::Rms3dF: case F::Rms3dK:
                case F::R: case F::T: case F::Nrtn:
                    return true;
                default:
                    return false;
            }
        };
        cols.erase(std::remove_if(cols.begin(), cols.end(),
                                  [&](const Col &c) { return isRef(c.field); }),
                   cols.end());
        return cols;
    }

    std::vector<FieldGroup> groupsFor(const std::vector<Col> &cols) {
        std::vector<FieldGroup> out;
        const char *open = nullptr; // 上一列的组名，nullptr 表示它是单列
        for (const Col &col: cols) {
            if (col.group && open && std::strcmp(open, col.group) == 0) {
                out.back().cols.push_back(col);
                continue;
            }
            open = col.group;
            out.push_back({uiLabel(col.group ? col.group : col.name), {col}});
        }
        return out;
    }

    std::vector<Col> resolveColumns(const Spec &spec, const Format fmt, const Task &task) {
        if (spec.fields.empty()) return task.exportDefaults(fmt);
        const std::vector<Col> known = task.exportFields(fmt);
        std::vector<Col> c;
        c.reserve(spec.fields.size());
        for (const Field f: spec.fields) {
            const char *name = fieldDef(f).name;
            for (const Col &col: known)
                if (col.field == f) {
                    name = col.name;
                    break;
                }
            c.push_back({f, name});
        }
        return c;
    }

    void writeCSV(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx);

    void writePOS(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx);

    void writeSP3(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx);

    void writeORB(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx);

    void writeTRAJ(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx);

    void writeKML(std::ostream &out, const Task &task, const std::vector<size_t> &idx);

    void writeSBET(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx);

    void write(std::ostream &out, const Task &task, const Spec &spec) {
        const std::vector<std::size_t> idx = pick(task, spec);
        switch (spec.format) {
            case Format::POS: writePOS(out, task, spec, idx);
                break;
            case Format::SP3: writeSP3(out, task, spec, idx);
                break;
            case Format::ORB: writeORB(out, task, spec, idx);
                break;
            case Format::TRAJ: writeTRAJ(out, task, spec, idx);
                break;
            case Format::KML: writeKML(out, task, idx);
                break;
            case Format::SBET: writeSBET(out, task, spec, idx);
                break;
            default: writeCSV(out, task, spec, idx);
                break;
        }
    }
}
