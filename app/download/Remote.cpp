#include "Remote.h"

#include "Transfer.h"
#include "Store.h"

#include "Log.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <map>
#include <mutex>
#include <thread>
#include <utility>

namespace Download::Remote {
    namespace {
        constexpr int kDlWorkers = 4;

        struct DlState {
            float frac = -1.0f;
        };

        /// 扫描缓存与下载队列的全部状态。线程、锁、队列都只在本 TU 里，
        /// 界面侧只能经 collect/enqueue/progress 取数与发命令。
        struct State {
            std::map<std::string, DirCache> dirs;
            std::mutex mtx;
            std::atomic<bool> scanning{false};
            std::atomic<int> scanDone{0}, scanTotal{0};
            std::string status;
            std::thread scanThread;
            std::atomic<bool> scanStop{false};

            std::map<std::string, DlState> dlActive;
            std::vector<std::thread> dlWorkers;
            std::mutex dlQmtx;
            std::condition_variable dlCv;
            std::deque<Pending> dlQueue;
            std::atomic<int> dlActiveCount{0};
            std::atomic<int> dlDone{0};
            std::atomic<int> dlTotal{0};
            std::atomic<bool> dlStop{false};
            bool dlStarted = false;
        };

        State g;

        void markDownloading(const std::string &remote) {
            std::lock_guard lk(g.mtx);
            g.dlActive[remote] = DlState{};
        }

        void runScan(std::vector<std::string> urls) {
            try {
                for (std::size_t i = 0; i < urls.size(); ++i) {
                    const std::string &u = urls[i];
                    {
                        std::lock_guard lk(g.mtx);
                        g.status = "读取目录 " + u;
                    }
                    std::string err;
                    DirCache dc;
                    if (std::vector<std::string> raw; Transfer::listDir(u, raw, err, &g.scanStop)) {
                        for (const auto &n: raw) {
                            if (auto m = ProductMeta::parse(n); m && m->type != ProductType::Unknown)
                                dc.items.push_back(*m);
                        }
                        dc.loaded = true;
                    } else {
                        dc.err = err;
                    }
                    if (g.scanStop.load()) {
                        std::lock_guard lk(g.mtx);
                        g.status = "扫描已取消";
                        g.scanning = false;
                        return;
                    }
                    std::lock_guard lk(g.mtx);
                    g.dirs[u] = std::move(dc);
                    g.scanDone = static_cast<int>(i) + 1;
                }
            } catch (const std::exception &e) {
                std::lock_guard lk(g.mtx);
                g.status = std::string("扫描异常中断: ") + e.what();
                g.scanning = false;
                return;
            } catch (...) {
                std::lock_guard lk(g.mtx);
                g.status = "扫描异常中断";
                g.scanning = false;
                return;
            }
            {
                std::lock_guard lk(g.mtx);
                g.status = "就绪";
                g.scanning = false;
            }
        }

        void ensureScan(const std::vector<std::string> &dirs, const char *scanText) {
            std::lock_guard lk(g.mtx);
            std::vector<std::string> missing;
            for (const auto &d: dirs)
                if (!g.dirs.count(d)) missing.push_back(d);
            if (missing.empty() || g.scanning) return;
            g.scanning = true;
            g.scanDone = 0;
            g.scanTotal = static_cast<int>(missing.size());
            g.status = scanText;
            if (g.scanThread.joinable()) g.scanThread.join();
            g.scanThread = std::thread(runScan, std::move(missing));
        }

        void runDownload(const std::string &dirUrl, const ProductMeta &m) {
            const std::string remote = dirUrl + m.filename;
            const std::string local = Store::datePath(m.type, m.year, m.doy, m.filename);
            const std::string part = local + ".part";
            std::error_code ec;
            std::filesystem::create_directories(Store::dateDir(m.type, m.year, m.doy), ec);

            LOG_INFO << "[下载] 开始 " << m.filename << " ← " << dirUrl;
            markDownloading(remote);
            auto onProgress = [&remote](const double total, const double now) {
                float f = -1.0f;
                if (total > 0.0) {
                    const double r = now / total;
                    f = static_cast<float>(r < 0.0 ? 0.0 : r > 1.0 ? 1.0 : r);
                }
                std::lock_guard lk(g.mtx);
                if (const auto it = g.dlActive.find(remote); it != g.dlActive.end()) it->second.frac = f;
            };

            std::string log;
            bool ok = Transfer::downloadFile(remote, part, onProgress,
                                                 [&](const std::string &s) { log = s; }, 3, 120,
                                                 &g.dlStop);
            if (ok) {
                std::error_code mvEc;
                std::filesystem::rename(part, local, mvEc);
                if (mvEc) {
                    // 跨卷 / 目标已存在等罕见情况：拷一份再删
                    std::error_code cpEc;
                    std::filesystem::copy_file(part, local,
                                               std::filesystem::copy_options::overwrite_existing, cpEc);
                    if (!cpEc) std::filesystem::remove(part, mvEc);
                    if (cpEc) {
                        log = "改名失败: " + cpEc.message();
                        ok = false;
                    }
                }
            }
            if (ok) {
                const std::string finalPath = Store::decompressIfNeeded(local);
                const std::string finalName = std::filesystem::path(finalPath).filename().string();
                Store::Entry e;
                e.type = ProductMeta::typeDir(m.type);
                e.year = m.year;
                e.doy = m.doy;
                e.datacenter = m.datacenter;
                e.file = finalName;
                e.path = finalPath;
                std::error_code sec;
                e.size = static_cast<long long>(std::filesystem::file_size(finalPath, sec));
                Store::record(e);
                log = "完成: " + finalName;
            }

            g.dlActiveCount.fetch_sub(1);
            {
                std::lock_guard lk(g.mtx);
                g.dlActive.erase(remote);
                g.status = (ok ? "下载成功  " : "下载失败  ") + log;
            }
            if (ok)
                LOG_INFO << "[下载] 完成 " << m.filename;
            else
                LOG_ERROR << "[下载] 失败 " << m.filename << " — " << log;
        }

        ProductMeta metaFromPending(const Pending &p) {
            ProductMeta m;
            m.type = p.type;
            m.year = p.year;
            m.doy = p.doy;
            m.spanDays = p.spanDays;
            m.filename = p.remoteName;
            return m;
        }

        void dlWorker() {
            for (;;) {
                Pending p;
                {
                    std::unique_lock lk(g.dlQmtx);
                    g.dlCv.wait(lk, [] { return g.dlStop.load() || !g.dlQueue.empty(); });
                    if (g.dlStop.load() && g.dlQueue.empty()) return;
                    p = std::move(g.dlQueue.front());
                    g.dlQueue.pop_front();
                }
                runDownload(p.dirUrl, metaFromPending(p));
                g.dlDone.fetch_add(1);
            }
        }

        void ensureDlWorkers() {
            g.dlStarted = true;
            g.dlWorkers.reserve(kDlWorkers);
            for (int i = 0; i < kDlWorkers; ++i)
                g.dlWorkers.emplace_back(dlWorker);
        }

        /// 本地文件清单按目录 mtime 缓存：同一目录没改动就不重新扫盘。
        const std::vector<Store::LocalFile> &cachedLocal(const ProductType t, const int year, const int doy) {
            using Stamp = std::filesystem::file_time_type;
            static const std::vector<Store::LocalFile> empty;
            static std::map<std::string, std::pair<Stamp, std::vector<Store::LocalFile> > > cache;

            const std::string dir = Store::dateDir(t, year, doy);
            std::error_code ec;
            const Stamp mt = std::filesystem::last_write_time(dir, ec);
            if (ec) return empty;

            if (const auto it = cache.find(dir); it != cache.end() && it->second.first == mt)
                return it->second.second;

            auto &[fst, snd] = cache[dir];
            fst = mt;
            snd = Store::listFiles(t, year, doy);
            return snd;
        }

        void appendLocalOnly(std::vector<Match> &list, const ProductType t, const int year, const int doy) {
            std::set<std::string> have;
            for (const auto &mt: list) have.insert(Store::localNameFor(mt.m.filename));

            for (const auto &lf: cachedLocal(t, year, doy)) {
                if (!have.insert(Store::localNameFor(lf.name)).second) continue;
                Match mt;
                mt.m.type = t;
                mt.m.year = year;
                mt.m.doy = doy;
                mt.m.filename = lf.name;
                mt.localPath = lf.path;
                list.push_back(std::move(mt));
            }
        }
    }

    Snapshot collect(const std::vector<std::string> &dirs, const std::set<ProductType> &types,
                     const int year, const int doy, const std::set<std::string> &datacenters,
                     const char *scanText) {
        ensureScan(dirs, scanText);

        Snapshot out;
        {
            std::lock_guard lk(g.mtx);
            std::set<std::string> seen;
            for (const auto &d: dirs) {
                const auto it = g.dirs.find(d);
                if (it == g.dirs.end()) continue;
                if (!it->second.loaded) continue;
                for (const auto &m: it->second.items) {
                    if (!types.count(m.type)) continue;
                    if (!m.covers(year, doy)) continue;
                    if (m.type != ProductType::NAV && !datacenters.empty() && !datacenters.count(m.datacenter))
                        continue;
                    if (!seen.insert(m.filename).second) continue;
                    out.matches.push_back({m, d, {}});
                }
            }
            out.scanning = g.scanning.load();
            out.scanned = g.scanDone.load();
            out.total = g.scanTotal.load();
        }

        std::sort(out.matches.begin(), out.matches.end(),
                  [](const Match &a, const Match &b) { return a.m.filename < b.m.filename; });
        for (const auto t: types) appendLocalOnly(out.matches, t, year, doy);
        return out;
    }

    void enqueue(const Pending &p) {
        {
            std::lock_guard lk(g.mtx);
            g.status = "下载中…";
        }
        {
            std::lock_guard lk(g.dlQmtx);
            g.dlQueue.push_back(p);
            g.dlTotal.fetch_add(1);
            g.dlActiveCount.fetch_add(1);
        }
        g.dlCv.notify_one();
        ensureDlWorkers();
    }

    bool downloadingOf(const std::string &remote, float &frac) {
        std::lock_guard lk(g.mtx);
        const auto it = g.dlActive.find(remote);
        if (it == g.dlActive.end()) return false;
        frac = it->second.frac;
        return true;
    }

    Progress progress() {
        std::lock_guard lk(g.mtx);
        return {g.scanning.load(), g.scanDone.load(), g.scanTotal.load(),
                g.dlActiveCount.load(), g.dlDone.load(), g.dlTotal.load(), g.status};
    }

    void setStatus(const std::string &text) {
        std::lock_guard lk(g.mtx);
        g.status = text;
    }

    void shutdown() {
        g.scanStop = true;
        if (g.scanThread.joinable()) g.scanThread.join();
        {
            std::lock_guard lk(g.dlQmtx);
            g.dlStop = true;
        }
        g.dlCv.notify_all();
        for (auto &w: g.dlWorkers)
            if (w.joinable()) w.join();
    }
}
