#pragma once

#include <map>
#include <set>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <thread>
#include <functional>
#include <condition_variable>
#include "download/ProductMeta.h"

namespace Tools::FtpDownload {
    extern bool s_open;

    static constexpr int kDateTypeCount = 10;
    static const char *TYPE_LABELS[kDateTypeCount] = {
        "广播星历", "精密轨道", "精密钟差", "地球自转",
        "电离层", "对流层", "码偏差", "映射函数",
        "卫星姿态", "气象"
    };

    static bool s_typeOn[kDateTypeCount] = {true, true, true, true, true, true, true,
                                            true, true, true};
    static std::vector<std::uint8_t> s_dcOn;
    static int s_year = 0, s_month = 0, s_day = 0;

    static std::string s_importMsg;
    static bool s_importOk = false;

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

    struct DlState {
        float frac = -1.0f;
    };

    static std::map<std::string, DlState> s_dlActive;

    struct Pending {
        std::string remoteName;
        std::string dirUrl;
        ProductType type = ProductType::Unknown;
        int year = 0, doy = 0, spanDays = 1;
        std::string localPath;
    };


    static constexpr int kDlWorkers = 4;
    static std::vector<std::thread> s_dlWorkers;
    static std::mutex s_dlQmtx;
    static std::condition_variable s_dlCv;
    static std::deque<Pending> s_dlQueue;
    static std::atomic s_dlActiveCount{0};   // 正在下载的任务数
    static std::atomic s_dlDone{0};          // 已完成（成功+失败）计数
    static std::atomic s_dlTotal{0};         // 入队总数
    static std::atomic s_dlStop{false};
    static bool s_dlStarted = false;              // 仅主线程访问

    static Pending s_pending;
    static bool s_pendingOpen = false;

    void Render(bool *p_open);

    struct AcquirePanel {
        enum class Kind { Products, Obs, Single };

        Kind kind = Kind::Products;
        bool open = false;
        int year = 0, doy = 0;
        std::vector<std::string> *target = nullptr; // Products：解算产品清单
        std::string obsKey;                         // Obs：PathMemory 键
        std::string *obsOut = nullptr;              // Obs：回填到该填写框
        ProductType singleType = ProductType::Unknown;
        std::string *singleOut = nullptr;
        std::function<void(const std::string &)> onPick; // Single：回填后的副作用
        float height = 280.0f;

        std::vector<std::uint8_t> typeOn;  // 产品类型多选
        std::vector<std::uint8_t> dcOn;    // 数据中心多选
        std::set<std::string> sites;  // 观测站点多选
        char siteFind[32] = {};       // 只缩候选站点标签
        int stateFilter = 0;          // 本地状态
    };

    bool renderAcquirePanel(AcquirePanel &p, const char *idScope);

    void Shutdown();
}
