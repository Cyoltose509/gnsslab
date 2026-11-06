#pragma once

#include <optional>
#include <string>
#include <vector>

#include "ProductMeta.h"

namespace Download {
    /// 版本型产品
    struct VersionedProduct {
        const char *id;       // 稳定标识
        const char *label;    // 界面显示名
        const char *indexUrl; // 远端目录
        const char *pattern;  // 在索引正文中匹配文件名的正则，第 1 组为版本号
        const char *fallback; // 正则无命中时的固定文件名
        int checkHours;       // 检查周期（小时）
        std::vector<std::wstring> exts;   // 手工导入时的文件对话框后缀（不含点）
        bool (*validate)(const std::string &path, std::string &detail);
    };
}

namespace Download::Store {
    struct Entry {
        std::string type;
        int year = 0;
        int doy = 0;
        std::string datacenter;
        std::string file;
        std::string path;
        long long size = 0;
    };

    struct LocalFile {
        std::string name;
        std::string path;
        long long size = 0;
    };

    /// 本地库里的一条文件：「本地文件」页要同时列日期产品与辅助产品，故自带归类字段。
    struct BrowseEntry {
        std::string path;     // 绝对路径，正斜杠
        std::string name;     // 文件名
        std::string category; // 日期产品＝类型名；辅助产品＝产品 id
        std::string stamp;    // 日期产品＝2026-183；辅助产品＝版本号
        bool aux = false;     // 是否辅助产品（auxdata/ 下）
        long long size = 0;
    };

    /// 扫一遍 downloads/ 下的日期产品目录与 auxdata/，按路径排序。
    std::vector<BrowseEntry> browse();

    /// 批量彻底删除的结果。
    struct Removal {
        int removed = 0;
        int failed = 0;
        long long bytes = 0;
    };

    /// 版本型产品库里已安装的当前版本
    struct Installed {
        std::string id;
        std::string version; // 版本号（ATX 用 IGS 文件名的 4 位序号）：只用于判「是否最新」，不当目录名
        std::string file;    // 文件名
        std::string path;    // 绝对路径
        long long size = 0;
        std::string fetchedAt; // yyyy-mm-dd hh:mm:ss
    };

    std::string downloadRoot();

    std::string dateDir(ProductType t, int year, int doy);

    std::string datePath(ProductType t, int year, int doy, const std::string &filename);

    std::string localNameFor(const std::string &remoteName);

    std::string findLocal(ProductType t, int year, int doy, const std::string &remoteName);

    std::vector<LocalFile> listFiles(ProductType t, int year, int doy);

    bool revealFile(const std::string &path);

    /// 彻底删除（不进回收站），删空的日期目录一并收掉，索引同步清理。
    Removal removeFiles(const std::vector<std::string> &paths);

    void record(const Entry &e);

    std::string decompressIfNeeded(const std::string &localPath);

    std::vector<Entry> readManifest();

    void writeManifest(const std::vector<Entry> &all);

    std::string productRoot();

    std::string productDir(const std::string &id);

    std::optional<Installed> active(const std::string &id);

    std::string activeFile(const std::string &id);

    std::optional<Installed> active(const VersionedProduct &product);

    std::string activeFile(const VersionedProduct &product);

    std::string tempPath(const std::string &id, const std::string &name);

    bool ensureDir(const std::string &id);

    void clearStaging(const std::string &id);

    void clearStagingExcept(const std::string &id, const std::string &keepName);

    bool install(const std::string &id, const std::string &srcPath,
                 const std::string &version, const std::string &fileName,
                 bool removeSource = true);
}
