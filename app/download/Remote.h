#pragma once

#include <set>
#include <string>
#include <vector>

#include "download/ProductMeta.h"

namespace Download::Remote {
    /// 一个远端目录的扫描结果：loaded 表示已读回清单。
    struct DirCache {
        bool loaded = false;
        std::string err;
        std::vector<ProductMeta> items;
    };

    /// 目录里的一条候选。localPath 只在「本地有、远端没列出」时填。
    struct Match {
        ProductMeta m;
        std::string dirUrl;
        std::string localPath;
    };

    struct Pending {
        std::string remoteName;
        std::string dirUrl;
        ProductType type = ProductType::Unknown;
        int year = 0;
        int doy = 0;
        int spanDays = 1;
    };

    /// 一次取数：目录还在扫就返回已有部分（scanning 为真），扫完为假。
    struct Snapshot {
        std::vector<Match> matches;
        bool scanning = false;
        int scanned = 0;
        int total = 0;
    };

    struct Progress {
        bool scanning = false;
        int scanDone = 0;
        int scanTotal = 0;
        int active = 0;
        int done = 0;
        int total = 0;
        std::string status;
    };

    /// 确保 dirs 都扫过（没扫过就起后台线程），按类型/日期/数据中心筛出候选，
    /// 再补上本地已有但远端未列出的文件。目录缓存与扫描线程都在本模块内部。
    Snapshot collect(const std::vector<std::string> &dirs, const std::set<ProductType> &types,
                     int year, int doy, const std::set<std::string> &datacenters,
                     const char *scanText = "读取目录…");

    void enqueue(const Pending &p);

    /// 该远端文件正在下载时返回 true 并填进度（总量未知时为负）。
    bool downloadingOf(const std::string &remote, float &frac);

    Progress progress();

    /// 操作反馈只有一行，扫描/下载/删除共用；界面上直接显示。
    void setStatus(const std::string &text);

    /// 停扫描线程与下载线程；窗口关闭时调用。
    void shutdown();
}
