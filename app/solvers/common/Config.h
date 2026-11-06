#pragma once

#include <set>
#include <string>

#include "SolverOptions.h"

namespace Solvers {
    enum class Mode { SPP, PPPStatic, PPPKinematic, LEO, RTK };

    /// 显示名与 ini 键前缀/产品池归属同表维护。
    struct ModeInfo {
        Mode mode;
        const char *label;
        const char *key;
    };

    inline constexpr ModeInfo kModes[] = {
        {Mode::SPP, "SPP", "spp"},
        {Mode::PPPStatic, "PPP (静态)", "ppp_static"},
        {Mode::PPPKinematic, "PPP (动态)", "ppp_kin"},
        {Mode::LEO, "LEO", "leo"},
        {Mode::RTK, "RTK", "rtk"},
    };

    inline const ModeInfo &modeInfo(const Mode m) {
        for (const auto &x: kModes) if (x.mode == m) return x;
        return kModes[0];
    }

    inline const char *modeLabel(const Mode m) { return modeInfo(m).label; }

    inline const char *modeKey(const Mode m) { return modeInfo(m).key; }

    inline std::string sysSetToString(const std::set<char> &s) {
        std::string out;
        for (char c: {'G', 'R', 'E', 'C'}) if (s.count(c)) out += c;
        return out;
    }

    inline std::set<char> stringToSysSet(const std::string &s) {
        std::set<char> out;
        for (char c: s) if (c == 'G' || c == 'R' || c == 'E' || c == 'C') out.insert(c);
        return out.empty() ? std::set{'G', 'R', 'E', 'C'} : out;
    }

    struct RealtimeEndpoint {
        std::string ip;
        std::string port;
    };

    /// 各模式的参数默认值落在模式自己的 Task.h（`SPP::defaultOptions()` 等），
    /// 这里只按 Mode 转发，不维护任何数值。RTK 没有特有默认值，用 SolverOptions 的成员初值。
    SolverOptions defaultSolverOptions(Mode mode);
}
