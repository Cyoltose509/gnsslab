#pragma once

#include <set>
#include <string>

#include "SolverOptions.h"
#include "i18n/I18n.h"

namespace Solvers {
    enum class Mode { SPP, PPPStatic, PPPKinematic, LEO, RTK, INS };

    /// 显示名与 ini 键前缀/产品池归属同表维护。
    struct ModeInfo {
        Mode mode;
        const char *label;
        const char *key;
    };

    inline constexpr ModeInfo kModes[] = {
        {Mode::SPP, "SPP", "spp"},
        {Mode::PPPStatic, "PPP (Static)", "ppp_static"},
        {Mode::PPPKinematic, "PPP (Kinematic)", "ppp_kin"},
        {Mode::LEO, "LEO", "leo"},
        {Mode::RTK, "RTK", "rtk"},
        {Mode::INS, "INS", "ins"},
    };

    inline const ModeInfo &modeInfo(const Mode m) {
        for (const auto &x: kModes) if (x.mode == m) return x;
        return kModes[0];
    }

    inline const char *modeLabel(const Mode m) { return modeInfo(m).label; }

    inline const char *modeKey(const Mode m) { return modeInfo(m).key; }

    constexpr bool modeAllowsRefFixed(const Mode m) {
        return m == Mode::SPP || m == Mode::RTK || m == Mode::PPPStatic;
    }

    constexpr bool modeAllowsRefFile(const Mode m) {
        return m == Mode::SPP || m == Mode::RTK || m == Mode::PPPKinematic || m == Mode::LEO;
    }

    constexpr bool modeUsesRTN(const Mode m) { return m == Mode::LEO; }

    /// 只有估了模糊度的模式才有 Fixed/Float 之分：界面「固定率」「解状态」与固定解列都以此为界，
    /// SPP 纯码解、INS 力学编排都不出现这两个词。
    constexpr bool modeFixesAmbiguities(const Mode m) {
        return m == Mode::RTK || m == Mode::PPPStatic || m == Mode::PPPKinematic || m == Mode::LEO;
    }

    /// 有跨历元状态的模式才谈得上滤波方向：SPP 起家，PPP/LEO 继承，RTK 的双站重放是另一条链路但语义相同。
    constexpr bool modeHasFilter(const Mode m) {
        return m == Mode::SPP || m == Mode::PPPStatic || m == Mode::PPPKinematic || m == Mode::LEO ||
               m == Mode::RTK;
    }

    /// 滤波方向词表唯一真源：界面下拉、`smooth` 键、日志与导出元信息都从这里取。
    constexpr const char *filterKey(const SolverOptions::FilterMode m) {
        using SM = SolverOptions::FilterMode;
        switch (m) {
            case SM::LSQ: return "none";
            case SM::Backward: return "backward";
            case SM::Both: return "both";
            default: return "forward";
        }
    }

    /// 档位显示名随语言变，因此不是 constexpr；表里只存档位，渲染时再取名字。
    inline const char *filterLabel(const SolverOptions::FilterMode m) {
        using SM = SolverOptions::FilterMode;
        switch (m) {
            case SM::LSQ: return I18n::trc("filter.lsq");
            case SM::Backward: return I18n::trc("filter.backward");
            case SM::Both: return I18n::trc("filter.forward_backward");
            default: return I18n::trc("filter.forward");
        }
    }

    inline SolverOptions::FilterMode filterFromKey(const std::string &s, const SolverOptions::FilterMode def) {
        using SM = SolverOptions::FilterMode;
        if (s == "none" || s == "0" || s == "false") return SM::LSQ;
        if (s == "backward") return SM::Backward;
        if (s == "both") return SM::Both;
        if (s == "forward" || s == "1" || s == "true") return SM::Forward;
        return def;
    }

    struct FilterChoice {
        SolverOptions::FilterMode mode;
    };

    inline constexpr FilterChoice kFilterChoices[] = {
        {SolverOptions::FilterMode::LSQ},
        {SolverOptions::FilterMode::Forward},
        {SolverOptions::FilterMode::Backward},
        {SolverOptions::FilterMode::Both},
    };

    inline std::string sysSetToString(const std::set<SatSystem> &s) {
        std::string out;
        for (const SatSystem sys: {SatSystem::G, SatSystem::R, SatSystem::E, SatSystem::C})
            if (s.count(sys)) out += charFrom(sys);
        return out;
    }

    inline std::set<SatSystem> stringToSysSet(const std::string &s) {
        std::set<SatSystem> out;
        for (const char c: s)
            if (const SatSystem sys = satSystemFrom(c); isKnownSystem(sys)) out.insert(sys);
        return out.empty() ? std::set<SatSystem>{SatSystem::G, SatSystem::R, SatSystem::E, SatSystem::C} : out;
    }

    struct RealtimeEndpoint {
        std::string ip;
        std::string port;
    };

    SolverOptions defaultSolverOptions(Mode mode);
}
