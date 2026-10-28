#pragma once

#include <map>
#include <mutex>
#include "download/ProductMeta.h"
#include <atomic>

namespace Tools::FtpDownload {
    extern bool s_open;

    static const char *TYPE_LABELS[8] = {
        "广播星历", "精密轨道", "精密钟差", "地球自转",
        "电离层", "对流层", "码偏差", "天线文件"
    };

    static bool s_typeOn[8] = {true, true, true, true, true, true, true, true};
    static std::vector<uint8_t> s_dcOn;
    static int s_year = 0, s_month = 0, s_day = 0;

    struct DirCache {
        bool loaded = false;
        bool missing = false;
        std::string err;
        std::vector<ProductMeta> items;
    };

    static std::map<std::string, DirCache> s_dirs;

    static std::mutex s_mtx;
    static std::atomic s_scanning{false};
    static std::atomic s_downloading{false};
    static std::atomic<int> s_done{0}, s_total{0};
    static std::string s_status;

    static std::thread s_scanThread;
    static std::thread s_dlThread;

    struct DlState {
        float frac = -1.0f;
    };

    static std::map<std::string, DlState> s_dlActive;

    struct Pending {
        std::string remoteName;
        std::string dirUrl;
        ProductType type = ProductType::Unknown;
        int year = 0, doy = 0, spanDays = 1;
        std::string localPath; // 非空 = 本地已有
    };

    static Pending s_pending;
    static bool s_pendingOpen = false;

    void Render(bool *p_open);

    void RequestAutoDownload(ProductType type, int year, int doy, const std::string &datacenter);

    void Shutdown();
}
