#pragma once
/**
 * AppConfig — 项目外（程序同级目录）的轻量 ini 记忆。
 * --------------------------------------------------------------------------
 * 用途：记住各页面填过的文件路径等用户配置，跨会话保留。
 * 存储：<gnsslab.exe 所在目录>/gnsslab.ini（项目源码树之外，不会被 git 跟踪/清理）。
 * 不同页面用不同的 key / group 区分：
 *   - get/set(key)        : 单一字符串值（section [paths]），如当前 obs 路径、realtime IP。
 *   - getRecent/addRecent : 历史列表（section [recent_<group>]，键 0..N-1，最新在前），如下拉菜单。
 * 本类纯 app 层，不依赖任何 UI / 引擎代码。
 */
#include <string>
#include <vector>

class AppConfig {
public:
    static AppConfig &instance();

    [[nodiscard]] std::string get(const std::string &key, const std::string &def = "") const;
    void set(const std::string &key, const std::string &val) const;

    [[nodiscard]] std::vector<std::string> getRecent(const std::string &group) const;
    void addRecent(const std::string &group, const std::string &path, int max = 12) const;

private:
    AppConfig();
    void ensureIni() const;

    std::string path;
};
