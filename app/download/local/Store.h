#pragma once

#include <optional>
#include <string>
#include <vector>

#include "download/local/ProductType.h"

namespace Download::Store {
    struct LocalFile {
        std::string name;
        std::string path;
        long long size = 0;
    };

    /// 本地库里的一条文件
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

    /// 同一类型多个候选时的打分：分析中心 × 延迟等级，分数高的更适合解算
    struct Candidate {
        std::string path;
        std::string name;
        std::string center;   // 分析中心
        std::string latency;  // 延迟等级 FIN / RAP / ULT
        int score = 0;        // 越大越好
        std::string groupKey;
        std::string slot;
    };

    /// 解析不出类型、或类型不符的路径不进候选
    std::vector<Candidate> rank(const std::vector<std::string> &paths, ProductType t);

    /// 选该类型最优的一份；无候选时回退首条路径，空输入返回空串。
    std::string bestPath(const std::vector<std::string> &paths, ProductType t);

    /// 时次组类型（如 VMF）：最优组的全部时次路径都要；无候选返回原列表。
    std::vector<std::string> bestPaths(const std::vector<std::string> &paths, ProductType t);

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

    /// 文件或归档展开目录的总字节数；取不到按 0。
    long long sizeOnDisk(const std::string &path);

    std::string findLocal(ProductType t, int year, int doy, const std::string &remoteName);

    std::vector<LocalFile> listFiles(ProductType t, int year, int doy);

    /// 本机该日已落地的低轨轨道（递归到归档展开目录里；.tmp_/.part 半成品跳过），按路径排序。
    /// 轨道到底躺平还是躺在展开目录里由 Unpack 决定，界面不该自己再猜一遍。
    std::vector<std::string> truthFiles(int year, int doy);

    /// 老版本留下的归档包：内容已展开在同名目录里时把包删掉（解压完只剩重复占地方），返回删掉几个。
    int reclaimLegacyArchives();

    bool revealFile(const std::string &path);

    /// 彻底删除（不进回收站），删空的日期目录一并收掉，索引同步清理。
    Removal removeFiles(const std::vector<std::string> &paths);

    /// 内容写进 .tmp 再 rename，父目录自动建；两套本地索引共用这一条写入路径。
    void writeAtomic(const std::string &path, const std::string &content);

    std::string productRoot();

    std::string productDir(const std::string &id);

    std::optional<Installed> active(const std::string &id);

    std::string activeFile(const std::string &id);

    std::string tempPath(const std::string &id, const std::string &name);

    bool ensureDir(const std::string &id);

    void clearStaging(const std::string &id);

    void clearStagingExcept(const std::string &id, const std::string &keepName);

    bool install(const std::string &id, const std::string &srcPath,
                 const std::string &version, const std::string &fileName,
                 bool removeSource = true);
}
