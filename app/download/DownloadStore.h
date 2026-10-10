#pragma once

#include <string>
#include <vector>

#include "ProductMeta.h"   // ProductType

// 一条已下载记录（manifest 用）
struct DownloadEntry {
    std::string type;        // typeDir 名，如 "sp3"
    int         year = 0;
    int         doy  = 0;
    std::string datacenter;  // COD / IGS ...
    std::string file;        // 原始文件名
    std::string path;        // 本地绝对路径
    long long   size = 0;
};

// 下载落地与"已下载"标记。本地根 = exe 同级 downloads/，按 <type>/<year>/<doy>/ 组织。
// "已下载"以 manifest.json（JSON Lines，每行一个紧凑 JSON 对象）索引，跨会话保持。
class DownloadStore {
public:
    // exe 同级 downloads/ 绝对路径
    static std::string root();

    // downloads/<type>/<year>/<doy>/
    static std::string dirFor(ProductType t, int year, int doy);

    // 落盘完整路径（已 mkdir 父目录）
    static std::string localPath(ProductType t, int year, int doy, const std::string &filename);

    // 按 (类型,年,doy) 在本地磁盘直接查找（不依赖 manifest），返回匹配文件名列表
    static std::vector<std::string> findLocal(ProductType t, int year, int doy);

    // 远端名 → 本地落盘名（下载后会解压，故 .gz 后缀要剥掉）
    static std::string localNameFor(const std::string &remoteName);

    // 本地是否已有该远端文件；有则返回绝对路径，无则空串
    static std::string findLocalFile(ProductType t, int year, int doy, const std::string &remoteName);

    // 删除本地文件并从 manifest 移除该记录
    static bool removeFile(const std::string &path);

    // 记录一条下载完成（去重后追加到 manifest）
    static void record(const DownloadEntry &e);

    // 若 localPath 为 .gz，解压为去掉后缀的同名文件并删除压缩包，返回可直接使用的路径；
    // 非 .gz 原样返回；解压失败返回原路径。
    static std::string decompressIfNeeded(const std::string &localPath);

    // manifest 读写（极简 JSONL：仅本项目固定结构，文件名不含引号/反斜杠）
    static std::vector<DownloadEntry> readManifest();
    static void writeManifest(const std::vector<DownloadEntry> &all);
};
