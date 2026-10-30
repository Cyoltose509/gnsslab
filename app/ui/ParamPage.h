#pragma once

#include "imgui.h"
#include "core/PathMemory.h"

#include <algorithm>
#include <functional>
#include <string>
#include <type_traits>
#include <vector>

namespace Ui::ParamPage {

    struct Range {
        double lo = 0.0;
        double hi = 0.0;
        bool on = false;
    };

    inline constexpr Range unbounded{};

    constexpr Range between(const double lo, const double hi) { return {lo, hi, true}; }

    template <class>
    inline constexpr bool kDependentFalse = false;

    template <class T>
    struct Traits {
        static_assert(kDependentFalse<T>, "ParamPage 只支持 int / bool / double / std::string");
    };

    template <>
    struct Traits<int> {
        static void store(const std::string &k, const int v) {
            AppConfig::PathMemory::instance().setInt(k, v);
        }

        static void fetch(const std::string &k, int &v, const Range &r) {
            v = AppConfig::PathMemory::instance().getInt(k, v, r.on ? static_cast<int>(r.lo) : -1000,
                                                         r.on ? static_cast<int>(r.hi) : 1000);
        }
    };

    template <>
    struct Traits<bool> {
        static void store(const std::string &k, const bool v) {
            AppConfig::PathMemory::instance().setInt(k, v ? 1 : 0);
        }

        static void fetch(const std::string &k, bool &v, const Range &) {
            v = AppConfig::PathMemory::instance().getInt(k, v ? 1 : 0, 0, 1) != 0;
        }
    };

    template <>
    struct Traits<double> {
        static void store(const std::string &k, const double v) {
            AppConfig::PathMemory::instance().setNum(k, v);
        }

        static void fetch(const std::string &k, double &v, const Range &r) {
            v = AppConfig::PathMemory::instance().getNum(k, v, r.on ? r.lo : -1e12, r.on ? r.hi : 1e12);
        }
    };

    template <>
    struct Traits<std::string> {
        static void store(const std::string &k, const std::string &v) {
            AppConfig::PathMemory::instance().set(k, v);
        }

        static void fetch(const std::string &k, std::string &v, const Range &) {
            v = AppConfig::PathMemory::instance().get(k, v);
        }
    };

    enum Style { kValue, kFlag };

    template <class T>
    void drawValue(const char *label, T &v, const Range r) {
        if constexpr (std::is_same_v<T, int>) {
            ImGui::InputInt(label, &v);
            if (r.on) v = std::clamp(v, static_cast<int>(r.lo), static_cast<int>(r.hi));
        } else if constexpr (std::is_same_v<T, double>) {
            ImGui::InputDouble(label, &v, 0.0, 0.0, "%.10g");
            if (r.on) v = std::clamp(v, r.lo, r.hi);
        } else if constexpr (std::is_same_v<T, bool>) {
            ImGui::Checkbox(label, &v);
        }
    }

    template <class T>
    void drawFlag(const char *label, T &v) {
        bool on = v != 0;
        if (ImGui::Checkbox(label, &on)) v = static_cast<T>(on);
    }

    template <class S>
    struct Field {
        const char *label = "";
        const char *key = "";
        std::function<void(S &)> draw;
        std::function<void(const S &, const std::string &)> store;
        std::function<void(S &, const std::string &)> fetch;
        std::function<void(S &, const S &)> reset;
    };

    template <class S>
    struct Group {
        const char *title = "";
        std::vector<Field<S> > fields;
    };

    template <Style Style_, class S, class T, class At, class Atc>
    Field<S> makeField(const char *label, const char *key, const Range r, At at, Atc atc) {
        Field<S> f;
        f.label = label;
        f.key = key;
        f.draw = [label, r, at](S &s) {
            if constexpr (Style_ == kFlag) drawFlag(label, at(s));
            else drawValue(label, at(s), r);
        };
        f.store = [key, atc](const S &s, const std::string &p) { Traits<T>::store(p + key, atc(s)); };
        f.fetch = [key, r, at](S &s, const std::string &p) { Traits<T>::fetch(p + key, at(s), r); };
        f.reset = [at, atc](S &s, const S &d) { at(s) = atc(d); };
        return f;
    }

    template <class S, class T>
    Field<S> field(const char *label, const char *key, T S::*m, const Range r = unbounded) {
        return makeField<kValue, S, T>(label, key, r,
                                       [m](S &s) -> T & { return s.*m; },
                                       [m](const S &s) -> const T & { return s.*m; });
    }

    template <class S, class T>
    Field<S> flag(const char *label, const char *key, T S::*m) {
        return makeField<kFlag, S, T>(label, key, unbounded,
                                      [m](S &s) -> T & { return s.*m; },
                                      [m](const S &s) -> const T & { return s.*m; });
    }

    template <class S, class C, class T>
    Field<S> field(const char *label, const char *key, T C::*m, C S::*owner, const Range r = unbounded) {
        return makeField<kValue, S, T>(label, key, r,
                                       [m, owner](S &s) -> T & { return s.*owner.*m; },
                                       [m, owner](const S &s) -> const T & { return s.*owner.*m; });
    }

    template <class S, class C, class T>
    Field<S> flag(const char *label, const char *key, T C::*m, C S::*owner) {
        return makeField<kFlag, S, T>(label, key, unbounded,
                                      [m, owner](S &s) -> T & { return s.*owner.*m; },
                                      [m, owner](const S &s) -> const T & { return s.*owner.*m; });
    }

    template <class S, class Fn>
    void forEach(const std::vector<Group<S> > &groups, Fn &&fn) {
        for (const auto &group: groups)
            for (const auto &f: group.fields) fn(f);
    }

    template <class S>
    void render(const std::vector<Group<S> > &groups, S &state, const S &def) {
        ImGui::PushItemWidth(260);
        bool firstGroup = true;
        for (const auto &group: groups) {
            if (!firstGroup) ImGui::Separator();
            firstGroup = false;
            if (group.title && *group.title) ImGui::TextUnformatted(group.title);
            for (const auto &f: group.fields) f.draw(state);
        }
        ImGui::PopItemWidth();
        ImGui::Spacing();
        if (ImGui::Button("恢复默认设置", ImVec2(160, 0)))
            forEach(groups, [&](const Field<S> &f) { f.reset(state, def); });
    }

    template <class S>
    void save(const std::vector<Group<S> > &groups, const S &state, const std::string &prefix) {
        forEach(groups, [&](const Field<S> &f) { f.store(state, prefix); });
    }


    template <class S>
    void load(const std::vector<Group<S> > &groups, S &state, const S &def, const std::string &prefix) {
        state = def;
        forEach(groups, [&](const Field<S> &f) { f.fetch(state, prefix); });
    }
}
