#include "AutoSync.h"

#include "DownloadStore.h"
#include "RemoteClient.h"
#include "Log.h"
#include "ProductCatalog.h"
#include "ProductStore.h"
#include "StringUtils.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace Download::AutoSync {
    namespace {
        using Clock = std::chrono::steady_clock;

        std::mutex g_mtx;
        std::condition_variable g_cv;
        std::thread g_thread;
        bool g_running = false;
        bool g_kickAll = false;
        bool g_auto = true; // 自动同步：关掉后只响应 checkNow()
        std::set<std::string> g_kick;
        std::map<std::string, Status> g_status;
        std::map<std::string, Clock::time_point> g_nextDue;

        void setStatus(const std::string &id, Status s) {
            std::lock_guard lk(g_mtx);
            g_status[id] = std::move(s);
        }

        void setProgress(const std::string &id, const std::string &file, const double total,
                         const double now) {
            std::lock_guard lk(g_mtx);
            auto &s = g_status[id];
            s.phase = Status::Downloading;
            s.file = file;
            s.progress = total > 0.0 ? static_cast<float>(now / total) : -1.0f;
            s.gotBytes = now;
            s.totalBytes = total;
        }

        void scheduleNext(const VersionedProduct &p) {
            std::lock_guard lk(g_mtx);
            g_nextDue[p.id] = Clock::now() + std::chrono::hours(p.checkHours);
        }

        void scheduleRetry(const VersionedProduct &p) {
            std::lock_guard lk(g_mtx);
            g_nextDue[p.id] = Clock::now() + std::chrono::minutes(10);
        }

        double stagedBytes(const std::string &id, const std::string &name) {
            for (const std::string &p: {Store::tempPath(id, name + ".gz"), Store::tempPath(id, name)}) {
                std::error_code ec;
                const auto sz = std::filesystem::file_size(p, ec);
                if (!ec) return static_cast<double>(sz);
            }
            return 0.0;
        }

        Status make(const Status::Phase ph, std::string ver = {}, std::string file = {},
                    std::string detail = {}) {
            Status s;
            s.phase = ph;
            s.version = std::move(ver);
            s.file = std::move(file);
            s.detail = std::move(detail);
            return s;
        }

        void checkOne(const VersionedProduct &p) {
            setStatus(p.id, make(Status::Checking));

            std::string name, version;
            if (std::string err; !Catalog::discoverLatest(p, name, version, err)) {
                setStatus(p.id, make(Status::Error, {}, {}, err));
                LOG_WARN << "[产品] " << p.label << " 检查失败: " << err;
                scheduleRetry(p);
                return;
            }

            if (const auto cur = Store::active(p.id); cur && cur->version == version) {
                setStatus(p.id, make(Status::Ready, version, cur->file, "已是最新"));
                scheduleNext(p);
                return;
            }

            const auto prog = [&](const double total, const double now) {
                setProgress(p.id, name, total, now);
            };
            if (!Store::ensureDir(p.id)) {
                setStatus(p.id, make(Status::Error, version, name, "无法创建本地目录"));
                scheduleRetry(p);
                return;
            }
            Store::clearStagingExcept(p.id, version == "latest" ? std::string() : name);
            const double have = stagedBytes(p.id, name);
            if (have > 0.0)
                LOG_INFO << "[产品] " << p.label << " 续传，已有 " << static_cast<long long>(have)
                        << " 字节";
            setProgress(p.id, name, 0.0, have);

            bool ok = false;
            std::string installedFrom;
            if (const std::string gzTmp = Store::tempPath(p.id, name + ".gz"); RemoteClient::downloadFile(p.indexUrl + name + ".gz", gzTmp, prog, nullptr, 3, 0)) {
                if (const std::string got = DownloadStore::decompressIfNeeded(gzTmp); !Str::endsWithIC(got, ".gz")) {
                    installedFrom = got;
                    ok = true;
                }
            }
            if (!ok) {
                if (const std::string rawTmp = Store::tempPath(p.id, name); RemoteClient::downloadFile(p.indexUrl + name, rawTmp, prog, nullptr, 3, 0)) {
                    installedFrom = rawTmp;
                    ok = true;
                }
            }
            if (!ok || !Store::install(p.id, installedFrom, version, name)) {
                setStatus(p.id, make(Status::Error, version, name, "下载或安装失败"));
                LOG_WARN << "[产品] " << p.label << " 获取失败: " << name;
                scheduleRetry(p);
                return;
            }
            setStatus(p.id, make(Status::Ready, version, name, "已更新"));
            LOG_INFO << "[产品] " << p.label << " 已同步到 " << name;
            scheduleNext(p);
        }

        void loop() {
            {
                std::unique_lock lk(g_mtx);
                g_cv.wait_for(lk, std::chrono::seconds(2), [] { return !g_running; });
            }
            while (true) {
                std::vector<std::string> todo;
                {
                    std::unique_lock lk(g_mtx);
                    if (!g_running) return;
                    if (g_kickAll) {
                        g_kickAll = false;
                        g_kick.clear();
                        for (const auto &p: Catalog::all()) todo.emplace_back(p.id);
                    } else if (!g_kick.empty()) {
                        todo.assign(g_kick.begin(), g_kick.end());
                        g_kick.clear();
                    }
                    const auto now = Clock::now();
                    if (g_auto) {
                        for (const auto &p: Catalog::all()) {
                            const auto it = g_nextDue.find(p.id);
                            if (const bool due = it == g_nextDue.end() || it->second <= now; due && std::find(todo.begin(), todo.end(), p.id) == todo.end())
                                todo.emplace_back(p.id);
                        }
                    }
                    if (todo.empty()) {
                        g_cv.wait_for(lk, std::chrono::minutes(5), [] {
                            return !g_running || g_kickAll || !g_kick.empty();
                        });
                        continue;
                    }
                }
                for (const auto &id: todo) {
                    {
                        std::lock_guard lk(g_mtx);
                        if (!g_running) return;
                    }
                    if (const auto *p = Catalog::find(id)) checkOne(*p);
                }
            }
        }
    }

    void start() {
        {
            std::lock_guard lk(g_mtx);
            if (g_running) return;
            g_running = true;
        }
        g_thread = std::thread(loop);
    }

    void stop() {
        {
            std::lock_guard lk(g_mtx);
            if (!g_running) return;
            g_running = false;
        }
        g_cv.notify_all();
        if (g_thread.joinable()) g_thread.join();
    }

    void checkNow(const std::string &id) {
        {
            std::lock_guard lk(g_mtx);
            if (id.empty()) g_kickAll = true;
            else g_kick.insert(id);
        }
        g_cv.notify_all();
    }

    void setEnabled(const bool on) {
        {
            std::lock_guard lk(g_mtx);
            if (g_auto == on) return;
            g_auto = on;
            if (on) g_nextDue.clear();
        }
        g_cv.notify_all();
    }

    bool enabled() {
        std::lock_guard lk(g_mtx);
        return g_auto;
    }

    Status status(const std::string &id) {
        std::lock_guard lk(g_mtx);
        const auto it = g_status.find(id);
        return it == g_status.end() ? Status{} : it->second;
    }

    void noteImported(const std::string &id, const std::string &version,
                      const std::string &file, const std::string &detail) {
        setStatus(id, make(Status::Ready, version, file, detail.empty() ? "已导入" : detail));
        if (const auto *p = Catalog::find(id)) {
            std::lock_guard lk(g_mtx);
            g_nextDue[id] = Clock::now() + std::chrono::hours(p->checkHours);
        }
    }
}
