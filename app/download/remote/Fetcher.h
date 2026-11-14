#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "download/local/ProductMeta.h"
#include "download/local/Unpack.h"
#include "download/remote/http/Core.h"
#include "download/remote/SourceTable.h"

namespace Download::Fetcher {

    struct DirCache {
        int pending = 0;
        int srcOk = 0;
        std::string err;  // 这一路取失败的原因
        std::string note; // 这一路取成功但值得说一句（如服务端 404：该日未发布）
        std::vector<ProductMeta> items;
        std::map<std::string, std::string> sources;
        std::map<std::string, std::string> urls;
    };

    struct Match {
        ProductMeta m;
        std::string dirUrl;
        std::string url;
        std::string localPath;

        [[nodiscard]] std::string remoteUrl() const;
    };

    struct Pending {
        std::string remoteName;
        std::string url;
        ProductType type = ProductType::Unknown;
        std::string datacenter;
        int year = 0;
        int doy = 0;
        int spanDays = 1;
    };

    Pending pendingOf(const Match &m);

    struct DirState {
        std::string url;
        bool done = false;
        bool ok = false;
        std::string err;   // 这一路取失败的原因
        std::string note;  // 取成功但没有文件时的说明，界面画在空列表下方
    };

    struct Snapshot {
        std::vector<Match> matches;
        std::vector<DirState> sources;
        bool scanning = false;
        int scanned = 0;
        int total = 0;
        int failed = 0;
    };

    struct Progress {
        int active = 0;
        int done = 0;
        int total = 0;
        std::string status;
    };

    /// 下载完成后的后台阶段（排队 / 解压 / 合并观测）。stage 为空表示没在处理；
    /// frac < 0 表示该阶段拿不到进度，界面就不画进度条。
    struct PostState {
        std::string stage;
        float frac = -1.0f;
    };

    PostState postStateOf(const std::string &remoteName);

    Snapshot collect(const std::vector<SourceTable::Dir> &dirs, const SourceTable::Query &q);

    /// 把源表解析成具体文件地址：直取型（probeName 非空）原样返回，目录型列一遍再按
    /// 「名字前缀 + 覆盖当天 + 该目录的类型」筛，地址优先取列目录给回的绝对地址。
    /// 解算页「获取」要知道「今天该下哪个文件」，走的就是这一条。
    /// `listErr` 非空时输出「N 个源列目录失败」一句（全部成功时输出空），诊断用。
    std::vector<std::string> resolve(const std::vector<SourceTable::Dir> &dirs, int year, int doy,
                                     std::string *listErr = nullptr);

    void enqueue(const Pending &p);
    void retryFailed();

    /// 一次「取到本地库」的结果
    struct Fetched {
        bool ok = false;
        bool paused = false;   // 被 cancel 打断，已下的部分留着续传
        std::string path;      // 落点：明文文件，或归档展开后的同名目录
        std::string err;
    };

    /// 立刻取一个远端文件并落地，调用线程自己等：下载（断点续传）→ 解压/展开 → 记清单。
    /// 队列下载的每个 worker 走的也是这两步，所以临时名、落点名、解包口径只有一份实现。
    Fetched fetchNow(const Pending &p, const Http::ProgressCb &progress,
                     const std::atomic<bool> *cancel, const Unpack::Progress &unpack = {},
                     const std::function<void(const std::string &)> &stage = {});

    /// 在下的那条记录：进度比例、总量、已收、实测速率（字节/秒，未测出时为 0）。
    bool downloadingOf(const std::string &remote, float &frac, double &totalBytes, double &gotBytes,
                       double &speed);

    void pauseDownload(const std::string &remote);

    Progress progress();

    void setStatus(const std::string &text);
    void shutdown();
}
