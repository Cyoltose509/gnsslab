#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "core/AppConfig.h"
#include <algorithm>

AppConfig &AppConfig::instance() {
    static AppConfig s;
    return s;
}

AppConfig::AppConfig() {
    char buf[MAX_PATH] = {};
    const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string s(buf, n > 0 ? n : 0);
    if (const auto pos = s.find_last_of("\\/"); pos != std::string::npos) s = s.substr(0, pos + 1);
    s += "gnsslab.ini";
    path = s;
    ensureIni();
}

void AppConfig::ensureIni() const {
    // 确保文件存在（Get/WritePrivateProfile 会自动创建，但目录已确定即可）。
    // 这里仅做一次存在性探测，避免每次读写都做磁盘探测。
    if (const DWORD attr = GetFileAttributesA(path.c_str()); attr == INVALID_FILE_ATTRIBUTES) {
        // 创建一个空文件，保证后续读写稳定
        HANDLE h = CreateFileA(path.c_str(), GENERIC_WRITE, 0, nullptr,
                               CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
    }
}

std::string AppConfig::get(const std::string &key, const std::string &def) const {
    char buf[4096] = {};
    DWORD n = GetPrivateProfileStringA("paths", key.c_str(), def.c_str(),
                                       buf, 4096, path.c_str());
    return {buf, n};
}

void AppConfig::set(const std::string &key, const std::string &val) const {
    WritePrivateProfileStringA("paths", key.c_str(), val.c_str(), path.c_str());
}

std::vector<std::string> AppConfig::getRecent(const std::string &group) const {
    std::vector<std::string> v;
    const std::string sec = "recent_" + group;
    char buf[4096] = {};
    for (int i = 0; i < 4096; ++i) {
        DWORD n = GetPrivateProfileStringA(sec.c_str(), std::to_string(i).c_str(),
                                           "", buf, 4096, path.c_str());
        if (n == 0) break;
        v.emplace_back(buf, n);
    }
    return v;
}

void AppConfig::addRecent(const std::string &group, const std::string &entry, const int max) const {
    if (entry.empty()) return;
    auto v = getRecent(group);
    // 去重（精确匹配），再把新项放到最前
    v.erase(std::remove_if(v.begin(), v.end(),
                           [&](const std::string &s) { return s == entry; }),
            v.end());
    v.insert(v.begin(), entry);
    if (static_cast<int>(v.size()) > max) v.resize(max);

    const std::string sec = "recent_" + group;
    // 写入 0..n-1：最后一个参数必须是本对象的 ini 文件路径 this->path。
    // 待加入的那条路径改名为 entry，不再遮蔽成员 path（原命名误用会写进数据文件）。
    for (int i = 0; i < static_cast<int>(v.size()); ++i)
        WritePrivateProfileStringA(sec.c_str(), std::to_string(i).c_str(),
                                   v[i].c_str(), this->path.c_str());
    // 删除可能残留的旧索引
    char buf[16] = {};
    for (int i = static_cast<int>(v.size()); i < 4096; ++i) {
        const DWORD n = GetPrivateProfileStringA(sec.c_str(), std::to_string(i).c_str(),
                                           "", buf, 16, this->path.c_str());
        if (n == 0) break;
        WritePrivateProfileStringA(sec.c_str(), std::to_string(i).c_str(),
                                   nullptr, this->path.c_str());
    }
}
