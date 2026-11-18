#pragma once

enum class SatSystem : char {
    Unknown = 0,
    G = 'G',
    C = 'C',
    R = 'R',
    E = 'E'
};

constexpr SatSystem satSystemFrom(const char c) {
    return static_cast<SatSystem>(c);
}

constexpr char charFrom(const SatSystem s) {
    return static_cast<char>(s);
}

constexpr bool isKnownSystem(const SatSystem s) {
    return s == SatSystem::G || s == SatSystem::C || s == SatSystem::R || s == SatSystem::E;
}
