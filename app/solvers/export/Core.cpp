#include "Core.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "solvers/common/Config.h"
#include "solvers/common/Task.h"
#include "core/TimeStruct.h"
#include "i18n/I18n.h"

namespace Solvers::Export {
    namespace {
        constexpr FieldDef kDefs[] = {
            {"Wk", "wk", "export.col.wk", Kind::Int, 0},
            {"SOW", "sow", "export.col.sow", Kind::Fixed, 3},
            {"Status", "stat", "common.solution_state", Kind::Text, 0},
            {"ECEF-X/m", "x", "export.col.x", Kind::Sig, 11},
            {"ECEF-Y/m", "y", "export.col.y", Kind::Sig, 11},
            {"ECEF-Z/m", "z", "export.col.z", Kind::Sig, 11},
            {"REF-X/m", "refx", "export.col.refx", Kind::Sig, 11},
            {"REF-Y/m", "refy", "export.col.refy", Kind::Sig, 11},
            {"REF-Z/m", "refz", "export.col.refz", Kind::Sig, 11},
            {"B/deg", "lat", "export.col.lat", Kind::Sig, 10},
            {"L/deg", "lon", "export.col.lon", Kind::Sig, 10},
            {"H/m", "h", "common.height_m", Kind::Sig, 4},
            {"Float-B/deg", "fblat", "export.col.fblat", Kind::Sig, 10},
            {"Float-L/deg", "fblon", "export.col.fblon", Kind::Sig, 10},
            {"Float-H/m", "fbh", "export.col.fbh", Kind::Sig, 4},
            {"Fix-B/deg", "kblat", "export.col.kblat", Kind::Sig, 10},
            {"Fix-L/deg", "kblon", "export.col.kblon", Kind::Sig, 10},
            {"Fix-H/m", "kbh", "export.col.kbh", Kind::Sig, 4},
            {"EAST/m", "e", "export.col.e", Kind::Sig, 6},
            {"NORTH/m", "n", "export.col.n", Kind::Sig, 6},
            {"UP/m", "u", "export.col.u", Kind::Sig, 6},
            {"Float-X/m", "fx", "export.col.fx", Kind::Sig, 11},
            {"Float-Y/m", "fy", "export.col.fy", Kind::Sig, 11},
            {"Float-Z/m", "fz", "export.col.fz", Kind::Sig, 11},
            {"Float-E/m", "fe", "export.col.fe", Kind::Sig, 6},
            {"Float-N/m", "fn", "export.col.fn", Kind::Sig, 6},
            {"Float-U/m", "fu", "export.col.fu", Kind::Sig, 6},
            {"Float-SigmaP/m", "spfloat", "export.col.spfloat", Kind::Sig, 4},
            {"Fix-X/m", "kx", "export.col.kx", Kind::Sig, 11},
            {"Fix-Y/m", "ky", "export.col.ky", Kind::Sig, 11},
            {"Fix-Z/m", "kz", "export.col.kz", Kind::Sig, 11},
            {"Fix-E/m", "ke", "export.col.ke", Kind::Sig, 6},
            {"Fix-N/m", "kn", "export.col.kn", Kind::Sig, 6},
            {"Fix-U/m", "ku", "export.col.ku", Kind::Sig, 6},
            {"Fix-SigmaP/m", "spfix", "export.col.spfix", Kind::Sig, 4},
            {"Bl-X/m", "bfx", "export.col.bfx", Kind::Sig, 6},
            {"Bl-Y/m", "bfy", "export.col.bfy", Kind::Sig, 6},
            {"Bl-Z/m", "bfz", "export.col.bfz", Kind::Sig, 6},
            {"Bl-Len/m", "bfl", "export.col.bfl", Kind::Sig, 4},
            {"Bl-E/m", "bfe", "export.col.bfe", Kind::Sig, 6},
            {"Bl-N/m", "bfn", "export.col.bfn", Kind::Sig, 6},
            {"Bl-U/m", "bfu", "export.col.bfu", Kind::Sig, 6},
            {"Bl-Xk/m", "bkx", "export.col.bkx", Kind::Sig, 6},
            {"Bl-Yk/m", "bky", "export.col.bky", Kind::Sig, 6},
            {"Bl-Zk/m", "bkz", "export.col.bkz", Kind::Sig, 6},
            {"Bl-Lenk/m", "bkl", "export.col.bkl", Kind::Sig, 4},
            {"Bl-Ek/m", "bke", "export.col.bke", Kind::Sig, 6},
            {"Bl-Nk/m", "bkn", "export.col.bkn", Kind::Sig, 6},
            {"Bl-Uk/m", "bku", "export.col.bku", Kind::Sig, 6},
            {"Vel-X/m", "vx", "export.col.vx", Kind::Sig, 6},
            {"Vel-Y/m", "vy", "export.col.vy", Kind::Sig, 6},
            {"Vel-Z/m", "vz", "export.col.vz", Kind::Sig, 6},
            {"VN/m", "vn", "export.col.vn", Kind::Sig, 6},
            {"VE/m", "ve", "export.col.ve", Kind::Sig, 6},
            {"VD/m", "vd", "export.col.vd", Kind::Sig, 6},
            {"PDOP", "pdop", "export.col.pdop", Kind::Sig, 4},
            {"GDOP", "gdop", "export.col.gdop", Kind::Sig, 4},
            {"HDOP", "hdop", "export.col.hdop", Kind::Sig, 4},
            {"VDOP", "vdop", "export.col.vdop", Kind::Sig, 4},
            {"TDOP", "tdop", "export.col.tdop", Kind::Sig, 4},
            {"SigmaP", "sp", "export.col.sp", Kind::Sig, 4},
            {"SigmaV", "sv", "export.col.sv", Kind::Sig, 4},
            {"SigmaE", "se", "export.col.se", Kind::Sig, 4},
            {"SigmaN", "sn", "export.col.sn", Kind::Sig, 4},
            {"SigmaU", "su", "export.col.su", Kind::Sig, 4},
            {"SatCount", "sats", "SatelliteCount", Kind::Int, 0},
            {"Ratio", "ratio", "export.col.ratio", Kind::Sig, 6},
            {"Float-3dRMS/m", "rms3df", "export.col.rms3df", Kind::Sig, 6},
            {"Fix-3dRMS/m", "rms3dk", "export.col.rms3dk", Kind::Sig, 6},
            {"R/m", "r", "export.col.r", Kind::Sig, 6},
            {"T/m", "t", "export.col.t", Kind::Sig, 6},
            {"N/m", "nrtn", "export.col.nrtn", Kind::Sig, 6},
            {"a/m", "sma", "export.col.sma", Kind::Fixed, 3},
            {"e", "ecc", "export.col.ecc", Kind::Fixed, 8},
            {"i/deg", "inc", "export.col.inc", Kind::Fixed, 6},
            {"RAAN/deg", "raan", "export.col.raan", Kind::Fixed, 6},
            {"argp/deg", "argp", "export.col.argp", Kind::Fixed, 6},
            {"M/deg", "ma", "export.col.ma", Kind::Fixed, 6},
            {"hp/km", "hp", "export.col.hp", Kind::Fixed, 3},
            {"ha/km", "ha", "export.col.ha", Kind::Fixed, 3},
            {"T/s", "period", "export.col.period", Kind::Fixed, 3},
            {"Roll/deg", "roll", "export.col.roll", Kind::Fixed, 6},
            {"Pitch/deg", "pitch", "export.col.pitch", Kind::Fixed, 6},
            {"Heading/deg", "heading", "export.col.heading", Kind::Fixed, 6},
            {"ZuptApplied", "zupt", "export.col.zupt", Kind::Int, 0},
            {"ZuptB/deg", "zuptlat", "export.col.zuptlat", Kind::Sig, 10},
            {"ZuptL/deg", "zuptlon", "export.col.zuptlon", Kind::Sig, 10},
            {"ZuptH/m", "zupth", "export.col.zupth", Kind::Sig, 4},
            {"ZuptVN/m", "zuptvn", "export.col.zuptvn", Kind::Sig, 6},
            {"ZuptVE/m", "zuptve", "export.col.zuptve", Kind::Sig, 6},
            {"ZuptVD/m", "zuptvd", "export.col.zuptvd", Kind::Sig, 6},
            {"ZuptRoll/deg", "zuptroll", "export.col.zuptroll", Kind::Sig, 6},
            {"ZuptPitch/deg", "zuptpitch", "export.col.zuptpitch", Kind::Sig, 6},
            {"ZuptHeading/deg", "zuptheading", "export.col.zuptheading", Kind::Sig, 6},
            {"ZuptE/m", "zupte", "export.col.zupte", Kind::Sig, 6},
            {"ZuptN/m", "zuptn", "export.col.zuptn", Kind::Sig, 6},
            {"Axb/(m/s2)", "axb", "export.col.axb", Kind::Fixed, 6},
            {"Ayb/(m/s2)", "ayb", "export.col.ayb", Kind::Fixed, 6},
            {"Azb/(m/s2)", "azb", "export.col.azb", Kind::Fixed, 6},
            {"Wxb/(rad/s)", "wxb", "export.col.wxb", Kind::Fixed, 8},
            {"Wyb/(rad/s)", "wyb", "export.col.wyb", Kind::Fixed, 8},
            {"Wzb/(rad/s)", "wzb", "export.col.wzb", Kind::Fixed, 8},
        };
        static_assert(std::size(kDefs) == kFieldCount, "field table out of sync with Field enum");

        /// 导出弹窗里按组显示的名字：键是 Col::group。单列的名字走 FieldDef::colKey，
        /// 那张表与 CSV 表头共用，不在这里重复。
        constexpr struct { const char *key; const char *uiKey; } kUiLabels[] = {
            {"Wk", "export.col.wk"},
            {"SOW", "export.col.sow"},
            {"GPSTow", "export.col.sow"},
            {"Status", "common.solution_state"},
            {"SigmaP", "export.lbl.sig_p"},
            {"SigmaV", "export.lbl.sig_v"},
            {"Float-SigmaP/m", "export.lbl.float_sig_p"},
            {"Fix-SigmaP/m", "export.lbl.fix_sig_p"},
            {"SatCount", "SatelliteCount"},
            {"3dRMS/m", "ThreeDRMS"},
            {"Float-3dRMS/m", "export.lbl.float_rms3d"},
            {"Fix-3dRMS/m", "export.lbl.fix_rms3d"},
            {"ZuptApplied", "export.col.zupt"},
            {"ECEF-XYZ", "export.lbl.xyz"},
            {"REF-XYZ", "export.lbl.ref_xyz"},
            {"ENU", "export.lbl.enu"},
            {"BLH", "export.lbl.blh"},
            {"Float-BLH", "export.lbl.float_blh"},
            {"Fix-BLH", "export.lbl.fix_blh"},
            {"Float-XYZ", "export.lbl.float_xyz"},
            {"Float-ENU", "export.lbl.float_enu"},
            {"Fix-XYZ", "export.lbl.fix_xyz"},
            {"Fix-ENU", "export.lbl.fix_enu"},
            {"Bl-XYZ-f", "export.lbl.bl_xyz_f"},
            {"Bl-ENU-f", "export.lbl.bl_enu_f"},
            {"Bl-Len/m", "export.lbl.bl_len_f"},
            {"Bl-XYZ-k", "export.lbl.bl_xyz_k"},
            {"Bl-ENU-k", "export.lbl.bl_enu_k"},
            {"Bl-Lenk/m", "export.lbl.bl_len_k"},
            {"Vel-XYZ", "export.lbl.vel_xyz"},
            {"Vel-NED", "export.lbl.vel_ned"},
            {"RTN", "export.lbl.rtn"},
            {"RPH", "export.lbl.rph"},
            {"EN", "export.lbl.en"},
            {"Zupt-BLH", "export.lbl.zupt_blh"},
            {"Zupt-NED", "export.lbl.zupt_ned"},
            {"Zupt-RPH", "export.lbl.zupt_rph"},
            {"Zupt-EN", "export.lbl.zupt_en"},
            {"Accel", "export.lbl.accel"},
            {"Gyro", "export.lbl.gyro"},
        };

        /// 组名 / 单列 CSV 名 → 当前语言的名字；查不到就原样返回 CSV 名。
        const char *uiLabel(const char *key) {
            for (const auto &[k, uiKey]: kUiLabels)
                if (std::strcmp(k, key) == 0) return I18n::trc(uiKey);
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
            if (padEnd && out.back() != e - 1) out.push_back(e - 1);
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
            if (out.back() != e - 1) out.push_back(e - 1);
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
            case Format::ORB: return I18n::trc("OrbitalElements");
            case Format::TRAJ: return I18n::trc("Trajectory");
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
            out.push_back({col.group ? uiLabel(col.group) : I18n::trc(fieldDef(col.field).colKey), {col}});
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
