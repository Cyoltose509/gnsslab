#pragma once
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "solvers/common/Task.h"
#include "SolverOptions.h"
#include "core/IniFile.h"

class CliSource {
public:
    CliSource(const IniFile &ini, std::map<std::string, std::string> over)
        : m_ini(ini), m_over(std::move(over)) { }

    std::string get(const std::string &sec, const std::string &key, const std::string &def = "") const {
        if (const auto v = lookup(sec, key)) return *v;
        return m_ini.get(sec, key, def);
    }

    int getInt(const std::string &sec, const std::string &key, const int def) const {
        if (const auto v = lookup(sec, key)) return IniFile::toInt(*v, def);
        return m_ini.getInt(sec, key, def);
    }

    double getD(const std::string &sec, const std::string &key, const double def) const {
        if (const auto v = lookup(sec, key)) return IniFile::toDouble(*v, def);
        return m_ini.getD(sec, key, def);
    }

    bool getB(const std::string &sec, const std::string &key, const bool def) const {
        if (const auto v = lookup(sec, key)) return IniFile::toBool(*v, def);
        return m_ini.getB(sec, key, def);
    }

    /// 命令行给过、但整个运行里没人读过的键：多半是拼错，收尾时提醒一句。
    std::vector<std::string> unusedKeys() const {
        std::vector<std::string> out;
        for (const auto &[k, v]: m_over)
            if (!m_used.count(k)) out.push_back(k);
        return out;
    }

private:
    /// 命中命令行覆盖返回其值并记账；没命中返回空（调用方回落 ini）。
    std::optional<std::string> lookup(const std::string &sec, const std::string &key) const {
        if (const auto it = m_over.find(sec + "." + key); it != m_over.end()) {
            m_used[sec + "." + key] = true;
            return it->second;
        }
        if (const auto it = m_over.find(key); it != m_over.end()) {
            m_used[key] = true;
            return it->second;
        }
        return std::nullopt;
    }

    const IniFile &m_ini;
    std::map<std::string, std::string> m_over;
    mutable std::map<std::string, bool> m_used;
};

SolverOptions loadConfig(const CliSource &ini, Solvers::Mode mode);

/// ini 的 mode 键 → 解算模式（spp / ppp_static / ppp_kin / rtk / leo / ins）；拼错返回空。
std::optional<Solvers::Mode> modeFromKey(const std::string &s);

namespace Cli {
    std::string defaultCSVName(Solvers::Mode m);

    /// 导出规格 + 落地路径。规格的每一项都能在 ini 的 [output] 段里给；
    /// 命令行只作为临时覆盖，两者都不给时按 CSV、原采样、该格式的默认列集。
    struct ExportOpt {
        Solvers::Export::Spec spec;
        /// 空 = 用 [output] csv 的名字按格式换后缀。
        std::string path;
    };

    /// 从 [output] 段读导出规格：格式、采样、元素、区间、SP3 头与输出路径。
    ExportOpt loadExportOpt(const CliSource &ini, Solvers::Mode mode);

    /// 逗号/空白分隔的字段短名 → 列集，`where` 只用于报错（`[output] fields` 或 `--fields`）。
    /// 认不出的名字跳过并提示，不清空 spec.fields（覆盖由调用方决定）。
    void applyFieldList(const std::string &list, Solvers::Export::Spec &spec, const char *where);

    void dumpTask(Solvers::Task &task, const SolverOptions &cfg, const ExportOpt &opt = {});

    /// 统一入口：cfg.source 决定文件/实时，跑完按 opt 写出；返回进程退出码。
    int runTask(Solvers::Task &task, const SolverOptions &cfg, const ExportOpt &opt = {});
}
