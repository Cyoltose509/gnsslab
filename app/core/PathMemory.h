#pragma once

#include <memory>
#include <string>
#include <vector>

namespace GnssCore {
    // GUI 的路径记忆：上次用过的数据文件路径（[paths]）与最近条目列表（[recent_*]）。
    // 单例，配置文件固定为 exe 同目录的 gnsslab.ini。
    //
    // 职责边界：只管"用户上次选过什么"，不管解算参数（那在 GUI 各 Processor 的面板里）。
    // 底层用 core/IniFile（纯 C++）读写，不用 Win32 GetPrivateProfileString——
    // 后者与 CLI 那套 ini 语义略有出入（注释符、大小写、trim），两套并存容易写出读不回来的文件。
    class PathMemory {
    public:
        static PathMemory &instance();

        // value 为空字符串时视为"清除该键"（用户清空了输入框）
        std::string get(const std::string &key, const std::string &def = "") const;
        void set(const std::string &key, const std::string &val) const;

        std::vector<std::string> getRecent(const std::string &group) const;
        void addRecent(const std::string &group, const std::string &entry, int max = 12) const;

        const std::string &path() const { return filePath; }

    private:
        PathMemory();

        std::string filePath;
    };
} // namespace GnssCore
