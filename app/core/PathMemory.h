#pragma once

#include "core/IniFile.h"

#include <string>
#include <vector>

namespace AppConfig {

    class PathMemory {
    public:
        static PathMemory &instance();

        [[nodiscard]] std::string get(const std::string &key, const std::string &def = "") const;

        [[nodiscard]] double getNum(const std::string &key, double def, double lo, double hi) const;

        [[nodiscard]] int getInt(const std::string &key, int def, int lo, int hi) const;

        [[nodiscard]] std::vector<std::string> getRecent(const std::string &group) const;

        void set(const std::string &key, const std::string &val) const; //NOLINT

        void setNum(const std::string &key, double v) const; //NOLINT

        void setInt(const std::string &key, int v) const; //NOLINT

        void addRecent(const std::string &group, const std::string &entry, int max = 12) const; //NOLINT

        void flush() const; //NOLINT

        [[nodiscard]] const std::string &path() const { return filePath; }

    private:
        PathMemory();


        IniFile &file() const; //NOLINT

        std::string filePath;
        mutable IniFile cache;
        mutable bool loaded = false;
        mutable bool dirty = false;
    };
}
