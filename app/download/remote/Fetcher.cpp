#include "download/remote/Fetcher.h"

#include "download/local/Unpack.h"
#include "i18n/I18n.h"
#include "download/remote/http/Core.h"
#include "download/local/Store.h"

#include "io/rinex/RINEXObsMerge.h"
#include "Log.h"
#include "util/Clock.h"
#include "StringUtils.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

namespace Download::Fetcher {
    namespace {
        constexpr int kDlWorkers = 4;
        constexpr int kScanWorkers = 4;

        struct ScanJob {
            std::string url; // 实际地址
            std::string key; // 缓存键。
            ProductType type = ProductType::Unknown;
            std::string probeName; // 非空：不列目录，只确认这个文件在不在
            std::string namePrefix; // 非空：列目录后只留以此开头的文件
            int year = 0;
            int doy = 0;
            int attempts = 1; // 这一路取失败时最多试几次，由源的 Dir::attempts 给
        };

        std::string cacheKey(const ProductType type, const std::string &probeName, const std::string &url,
                             const int year, const int doy) {
            return std::to_string(static_cast<int>(type)) + '|' + probeName + '|' +
                   std::to_string(year) + std::to_string(doy) + '|' + url;
        }

        ScanJob makeJob(const std::string &url, const std::string &key, const SourceTable::Dir &d,
                        const int year, const int doy) {
            return {url, key, d.type, d.probeName, d.namePrefix, year, doy, d.attempts};
        }

        struct LocalDirCache {
            std::mutex mtx;
            std::map<std::string, std::pair<std::filesystem::file_time_type, std::vector<Store::LocalFile> > > dirs;
        };

        LocalDirCache gLocalCache;

        struct DlState {
            float frac = -1.0f;
            double total = -1.0; // 字节；服务器没给 Content-Length 时保持负
            double got = 0.0;
            double speed = 0.0; // 字节/秒，按相邻两次进度回调的增量算
            double lastGot = 0.0;
            Clock::Point stamp{};
        };

        struct PostJob {
            std::string remoteName;
            std::string local;
            FileMeta meta;
        };

        struct State {
            std::map<std::string, DirCache> dirs;
            std::mutex mtx;
            std::deque<ScanJob> scanQueue;
            std::condition_variable scanCv;
            std::vector<std::thread> scanWorkers;
            std::string status;
            std::atomic<bool> scanStop{false};

            std::map<std::string, DlState> dlActive;
            std::map<std::string, std::shared_ptr<std::atomic<bool> > > dlCancel;
            std::vector<std::thread> dlWorkers;
            std::mutex dlQmtx;
            std::condition_variable dlCv;
            std::deque<Pending> dlQueue;
            std::atomic<int> dlActiveCount{0};
            std::atomic<int> dlDone{0};
            std::atomic<int> dlTotal{0};
            std::atomic<bool> dlStop{false};

            std::mutex postQmtx;
            std::condition_variable postCv;
            std::deque<PostJob> postQueue;
            std::thread postThread;
            std::atomic<bool> postStop{false};
            bool postStarted = false;
            std::map<std::string, PostState> postJobs; // 远端文件名 → 当前阶段与进度
        };

        State g;

        void publishScan(const std::string &key, const std::string &url,
                         std::vector<FileMeta> &&parsed,
                         std::map<std::string, std::string> &&urls, std::string note = {}) {
            std::lock_guard lk(g.mtx);
            DirCache &dc = g.dirs[key];
            if (!note.empty()) dc.note = std::move(note);
            for (auto &m: parsed) {
                if (dc.sources.count(m.filename)) continue;
                dc.sources.emplace(m.filename, url);
                if (const auto it = urls.find(m.filename); it != urls.end()) dc.urls[m.filename] = it->second;
                dc.items.push_back(std::move(m));
            }
        }

        /// 取回一路目录并发布
        bool fetchJob(const ScanJob &job, std::string &err) {
            if (!job.probeName.empty()) {
                bool exists = false;
                if (!Http::probeFile(job.url, exists, err, &g.scanStop)) return false;
                if (!exists) {
                    publishScan(job.key, job.url, {}, {},
                                I18n::tr("dl.fetch.no_server_file", job.probeName));
                    return true;
                }
                FileMeta m;
                m.filename = job.probeName;
                m.type = job.type;
                m.year = job.year;
                m.doy = job.doy;
                m.spanDays = 1;
                publishScan(job.key, job.url, {m}, {{m.filename, job.url}});
                return true;
            }

            std::vector<std::string> raw;
            std::map<std::string, std::string> fulls;
            if (!Http::listDir(job.url, raw, err, &g.scanStop, &fulls)) return false;
            std::vector<FileMeta> parsed;
            for (const auto &n: raw) {
                if (!job.namePrefix.empty() && n.rfind(job.namePrefix, 0) != 0) continue;
                if (auto m = FileMeta::parseInDir(n, job.type, job.year, job.doy))
                    parsed.push_back(*m);
            }
            publishScan(job.key, job.url, std::move(parsed), std::move(fulls));
            return true;
        }

        bool hasPendingLocked() {
            return std::any_of(g.dirs.begin(), g.dirs.end(), [](const auto &kv) {
                return kv.second.pending > 0;
            });
        }

        void scanWorker() {
            while (true) {
                ScanJob job;
                {
                    std::unique_lock lk(g.mtx);
                    g.scanCv.wait(lk, [] { return g.scanStop.load() || !g.scanQueue.empty(); });
                    if (g.scanQueue.empty()) return;
                    job = std::move(g.scanQueue.front());
                    g.scanQueue.pop_front();
                }

                bool ok = false;
                std::string err;
                const int attempts = job.attempts > 0 ? job.attempts : 1;
                for (int attempt = 0; attempt < attempts && !g.scanStop.load(); ++attempt) {
                    try {
                        if (fetchJob(job, err)) {
                            ok = true;
                            break;
                        }
                    } catch (const std::exception &e) {
                        err = e.what();
                    } catch (...) {
                        err = I18n::tr("dl.fetch.scan_failed");
                    }
                    if (attempt + 1 < attempts)
                        Clock::sleep(Clock::milliseconds(800));
                }

                std::lock_guard lk(g.mtx);
                DirCache &dc = g.dirs[job.key];
                if (dc.pending > 0) --dc.pending;
                if (ok) ++dc.srcOk;
                else if (dc.err.empty()) dc.err = err;
                if (g.scanQueue.empty() && !hasPendingLocked()) g.status = I18n::trc("dl.status.ready");
            }
        }

        void startScanWorkers() {
            std::lock_guard lk(g.mtx);
            if (!g.scanWorkers.empty()) return;
            g.scanWorkers.reserve(kScanWorkers);
            for (int i = 0; i < kScanWorkers; ++i) g.scanWorkers.emplace_back(scanWorker);
        }

        /// 缺哪一路就把哪一路排进队列
        void ensureScan(const std::vector<SourceTable::Dir> &dirs, const int year, const int doy,
                        const char *scanTextKey) {
            startScanWorkers();
            const bool canFallback = Http::hasEarthdata();
            std::lock_guard lk(g.mtx);
            std::vector<ScanJob> fresh;
            for (const auto &d: dirs) {
                const std::string key = cacheKey(d.type, d.probeName, d.url, year, doy);
                if (g.dirs.count(key)) continue;
                int n = 0;
                fresh.push_back(makeJob(d.url, key, d, year, doy));
                ++n;
                if (!d.backupUrl.empty() && canFallback) {
                    fresh.push_back(makeJob(d.backupUrl, key, d, year, doy));
                    ++n;
                }
                g.dirs[key].pending = n;
            }
            if (fresh.empty()) return;
            g.scanQueue.insert(g.scanQueue.begin(), fresh.begin(), fresh.end());
            g.status = I18n::trc(scanTextKey);
            g.scanCv.notify_all();
        }

        FileMeta metaFromPending(const Pending &p) {
            FileMeta m;
            m.type = p.type;
            m.datacenter = p.datacenter;
            m.year = p.year;
            m.doy = p.doy;
            m.spanDays = p.spanDays;
            m.filename = p.remoteName;
            return m;
        }

        void setPostStage(const std::string &remoteName, const std::string &stage) {
            std::lock_guard lk(g.mtx);
            if (stage.empty()) g.postJobs.erase(remoteName);
            else g.postJobs[remoteName] = PostState{stage, -1.0f};
        }

        void setPostFrac(const std::string &remoteName, const float frac) {
            std::lock_guard lk(g.mtx);
            if (const auto it = g.postJobs.find(remoteName); it != g.postJobs.end() && frac > it->second.frac)
                it->second.frac = frac;
        }

        /// 展开目录里的分片按「星 × 天线」合并，只有全部组都成功才删原始分片。
        /// 组数是完成度唯一可靠的来源，进度按已完成组数报给调用方（队列报给 post 状态，同步获取报给界面）。
        void mergeSlices(const std::string &dir, const int year, const int doy,
                         const std::function<void(float)> &onFrac,
                         const std::function<void(const std::string &)> &stage) {
            namespace fs = std::filesystem;
            std::vector<std::string> parts;
            std::error_code ec;
            for (const auto &ent: fs::directory_iterator(dir, ec)) {
                if (std::error_code fec; !ent.is_regular_file(fec) || fec) continue;
                if (const std::string name = ent.path().filename().string(); Str::endsWithIC(name, ".rnx") && name.rfind("merged_", 0) != 0)
                    parts.push_back(ent.path().generic_string());
            }
            if (parts.size() < 2) return;

            const std::vector<RINEX::ObsGroup> groups = RINEX::groupObsFiles(parts);
            if (groups.empty()) return;
            bool anyMulti = false;
            for (const auto &[key, files]: groups)
                if (files.size() > 1) anyMulti = true;
            if (!anyMulti) return; // 普通产品的多个成员，不是同一颗星的分片

            struct Merged {
                std::vector<std::string> srcs;
                int epochs = 0;
            };
            std::vector<Merged> done;
            if (stage) stage(I18n::trc("dl.fetch.merging"));
            if (onFrac) onFrac(0.0f);
            for (std::size_t gi = 0; gi < groups.size(); ++gi) {
                const auto &[key, files] = groups[gi];
                const std::string out = (fs::path(dir) /
                                         ("merged_" + key + "_" + std::to_string(year) + "_" +
                                          std::to_string(doy) + ".rnx")).generic_string();
                if (files.size() == 1) {
                    std::error_code rc;
                    fs::rename(files.front(), out, rc);
                    if (rc) {
                        LOG_WARN << "[DL] " << I18n::tr("dl.fetch.rename_part_failed", rc.message());
                        return;
                    }
                    done.push_back({{}, 0});
                } else {
                    std::string err;
                    const int n = RINEX::mergeObsFiles(files, out, err);
                    if (n <= 0) {
                        LOG_ERROR << "[DL] " << I18n::tr("dl.fetch.merge_obs_failed", key, err);
                        return;
                    }
                    if (!err.empty())
                        LOG_WARN << "[DL] " << I18n::tr("dl.fetch.merge_note", key, err);
                    done.push_back({files, n});
                }
                if (onFrac) onFrac(static_cast<float>(gi + 1) / static_cast<float>(groups.size()));
            }
            for (const auto &[srcs, _]: done) {
                for (const auto &src: srcs) {
                    std::error_code rc;
                    fs::remove(src, rc);
                    if (rc)
                        LOG_WARN << "[DL] " << I18n::tr("dl.fetch.remove_part_failed", src,
                                                        rc.message());
                }
            }
            LOG_INFO << "[DL] " << I18n::tr("dl.fetch.merged_obs", dir, groups.size());
        }

        struct Sp3Epoch {
            double key = 0.0;
            std::vector<std::string> lines;
        };

        struct Sp3Fragment {
            std::string sat;
            std::string src; // 来自哪个分片；只有真正拼进输出的那些才删
            double key = 0.0;
            std::vector<Sp3Epoch> epochs;
        };

        /// `*  2026 11  3 22 12  0.00000000` → 单调递增的秒数，只用于排序去重。
        double sp3EpochKey(const std::string &line) {
            std::istringstream ss(line);
            int y = 0, mo = 0, d = 0, h = 0, mi = 0;
            double s = 0.0;
            if (char star = 0; !(ss >> star >> y >> mo >> d >> h >> mi >> s)) return 0.0;
            return ((static_cast<double>(y) * 13.0 + mo) * 32.0 + d) * 86400.0 +
                   h * 3600.0 + mi * 60.0 + s;
        }

        /// COSMIC-2 的轨道按「每片 128 分钟、每片一颗星」切开，包里是几十份 `*_sp3`。
        /// 片之间大量重叠，按历元去重拼成每颗星一份
        int mergeOrbitSlices(const std::string &dir, const int year, const int doy,
                             const std::function<void(const std::string &)> &stage) {
            namespace fs = std::filesystem;
            std::vector<std::string> files;
            std::error_code ec;
            for (const auto &ent: fs::recursive_directory_iterator(
                     dir, fs::directory_options::skip_permission_denied, ec)) {
                if (ec) break;
                if (!ent.is_regular_file(ec) || ec) continue;
                if (Str::endsWithIC(ent.path().filename().string(), "_sp3"))
                    files.push_back(ent.path().generic_string());
            }
            if (files.empty()) return 0;
            std::sort(files.begin(), files.end());
            if (stage) stage(I18n::trc("dl.fetch.merging"));

            std::map<std::string, std::vector<Sp3Fragment> > groups;
            std::string headC, headD;
            std::map<std::string, std::string> headPlus;

            for (const auto &f: files) {
                std::ifstream in(f);
                if (!in) continue;
                Sp3Fragment frag;
                frag.src = f;
                std::string line;
                while (std::getline(in, line)) {
                    while (!line.empty() && line.back() == '\r') line.pop_back();
                    if (line.empty()) continue;
                    if (line.rfind("#c", 0) == 0) {
                        if (headC.empty()) headC = line;
                        continue;
                    }
                    if (line.rfind("##", 0) == 0) {
                        if (headD.empty()) headD = line;
                        continue;
                    }
                    if (line[0] == '+') {
                        // 每片的第一行才是本片的星，后面是 128 星列表的补零续行
                        if (frag.sat.empty()) {
                            std::istringstream ss(line);
                            if (std::string plus, count; ss >> plus >> count >> frag.sat && headPlus.find(frag.sat) == headPlus.end())
                                headPlus[frag.sat] = line;
                        }
                        continue;
                    }
                    if (line[0] == '*') {
                        const double key = sp3EpochKey(line);
                        if (frag.epochs.empty()) frag.key = key;
                        frag.epochs.push_back({key, {line}});
                    } else if (!frag.epochs.empty()) {
                        frag.epochs.back().lines.push_back(line);
                    }
                }
                if (!frag.sat.empty() && !frag.epochs.empty()) groups[frag.sat].push_back(std::move(frag));
            }

            int merged = 0;
            std::vector<std::string> used;
            for (auto &[sat, frags]: groups) {
                std::sort(frags.begin(), frags.end(),
                          [](const Sp3Fragment &a, const Sp3Fragment &b) { return a.key < b.key; });
                const std::string path = (fs::path(dir) / ("leoOrb_" + std::to_string(year) + "_" +
                                                           std::to_string(doy) + "_" + sat + ".sp3"))
                        .generic_string();
                std::ofstream fo(path, std::ios::trunc);
                if (!fo) continue;
                fo << headC << '\n' << headD << '\n' << headPlus[sat] << '\n';
                double last = 0.0;
                for (const auto &frag: frags) {
                    for (const auto &[key, lines]: frag.epochs) {
                        if (key <= last) continue; // 后面几片的历元已经在前面写过了
                        last = key;
                        for (const auto &l: lines) fo << l << '\n';
                    }
                }
                fo << "EOF\n";
                if (fo.fail()) continue;
                std::error_code sizeEc;
                if (const auto size = fs::file_size(path, sizeEc); sizeEc || size < 1024) {
                    fs::remove(path, sizeEc);
                    continue;
                }
                ++merged;
                for (const auto &frag: frags) used.push_back(frag.src);
            }
            if (merged == 0) return 0;
            for (const auto &f: used) {
                std::error_code rc;
                fs::remove(f, rc);
            }
            LOG_INFO << "[DL] " << I18n::tr("dl.fetch.merged_orb", dir, merged);
            return merged;
        }

        /// 下载完的落地：解外层压缩/归档展开
        bool settleTo(const std::string &local, const FileMeta &meta,
                      const Unpack::Progress &onFrac, const std::function<void(const std::string &)> &stage,
                      std::string &main, std::string &err) {
            if (stage) stage(I18n::trc("dl.fetch.inflating"));
            const auto [settled, members] = Unpack::settle(local, onFrac);
            main = settled;
            if (main.empty()) {
                err = I18n::tr("dl.fetch.unpack_failed");
                return false;
            }
            std::error_code dirEc;
            if (const bool isDir = std::filesystem::is_directory(main, dirEc) && !dirEc; !members.empty() && isDir) {
                if (meta.type == ProductType::LEO_SP3)
                    mergeOrbitSlices(main, meta.year, meta.doy, stage);
                else
                    mergeSlices(main, meta.year, meta.doy, onFrac, stage);
            }
            if (stage) stage({});
            return true;
        }

        /// 下载到正式文件名
        bool downloadTo(const Pending &p, const Http::ProgressCb &progress,
                        const std::atomic<bool> *cancel, std::string &local, std::string &log) {
            const FileMeta m = metaFromPending(p);
            local = Store::datePath(m.type, m.year, m.doy, m.filename);
            const std::string part = local + ".part";
            std::error_code ec;
            std::filesystem::create_directories(Store::dateDir(m.type, m.year, m.doy), ec);

            LOG_INFO << "[DL] " << I18n::tr("dl.fetch.start", m.filename, p.url);
            if (!Http::downloadFile(p.url, part, progress, [&log](const std::string &s) { log = s; },
                                    5, 60, cancel))
                return false;

            std::error_code mvEc;
            std::filesystem::rename(part, local, mvEc);
            if (mvEc) {
                std::error_code cpEc;
                std::filesystem::copy_file(part, local,
                                           std::filesystem::copy_options::overwrite_existing, cpEc);
                if (!cpEc) std::filesystem::remove(part, mvEc);
                if (cpEc) {
                    log = I18n::tr("dl.fetch.rename_failed", cpEc.message());
                    return false;
                }
            }
            return true;
        }

        void postWorker() {
            while (true) {
                PostJob job;
                {
                    std::unique_lock lk(g.postQmtx);
                    g.postCv.wait(lk, [] { return g.postStop.load() || !g.postQueue.empty(); });
                    if (g.postStop.load() && g.postQueue.empty()) return;
                    job = std::move(g.postQueue.front());
                    g.postQueue.pop_front();
                }
                const std::string name = job.remoteName;
                const Unpack::Progress onFrac = [name](const double f) { setPostFrac(name, static_cast<float>(f)); };
                const auto stage = [name](const std::string &s) { setPostStage(name, s); };
                std::string main, err;
                const bool ok = settleTo(job.local, job.meta, onFrac, stage, main, err);
                setPostStage(name, {});
                {
                    std::lock_guard lk(g.mtx);
                    g.status = ok
                                   ? I18n::tr("dl.status.done",
                                              std::filesystem::path(main).filename().string())
                                   : I18n::tr("dl.status.settle_failed", err);
                }
                if (ok)
                    LOG_INFO << "[DL] " << I18n::tr("dl.fetch.post_done",
                                                    std::filesystem::path(main).filename().string());
                else
                    LOG_ERROR << "[DL] " << I18n::tr("dl.fetch.post_failed", job.remoteName, err);
            }
        }

        void ensurePostWorker() {
            std::lock_guard lk(g.postQmtx);
            if (g.postStarted) return;
            g.postStarted = true;
            g.postThread = std::thread(postWorker);
        }

        void enqueuePost(PostJob job) {
            ensurePostWorker();
            setPostStage(job.remoteName, I18n::trc("dl.fetch.queued"));
            {
                std::lock_guard lk(g.postQmtx);
                g.postQueue.push_back(std::move(job));
            }
            g.postCv.notify_one();
        }

        void runDownload(const Pending &p) {
            const FileMeta m = metaFromPending(p);
            const std::string remote = p.url;
            const auto cancel = std::make_shared<std::atomic<bool> >(false);
            {
                std::lock_guard lk(g.mtx);
                g.dlActive[remote] = DlState{};
                g.dlCancel[remote] = cancel;
            }
            auto onProgress = [&remote](const double total, const double now) {
                std::lock_guard lk(g.mtx);
                if (const auto it = g.dlActive.find(remote); it != g.dlActive.end()) {
                    DlState &st = it->second;
                    if (st.stamp == Clock::Point{}) {
                        st.lastGot = now;
                        st.stamp = Clock::now();
                    } else if (const double dt = Clock::since(st.stamp).sec(); dt >= 0.35) {
                        st.speed = (now - st.lastGot) / dt;
                        st.lastGot = now;
                        st.stamp = Clock::now();
                    }
                    st.frac = Http::fracOf(total, now);
                    st.total = total;
                    st.got = now;
                }
            };

            std::string log, local;
            bool ok = downloadTo(p, onProgress, cancel.get(), local, log);
            const bool paused = !ok && cancel->load();
            if (ok) {
                // 解压/合并交给后台队列：几百个成员逐个落明文会占住这条下载通道。
                enqueuePost({m.filename, local, m});
                log = I18n::tr("dl.fetch.background_inflate");
            }

            g.dlActiveCount.fetch_sub(1);
            {
                std::lock_guard lk(g.mtx);
                g.dlActive.erase(remote);
                g.dlCancel.erase(remote);
                if (paused) g.status = I18n::tr("dl.status.paused", m.filename);
                else g.status = I18n::tr(ok ? "dl.status.ok" : "dl.status.failed", log);
            }
            if (paused)
                LOG_INFO << "[DL] " << I18n::tr("dl.fetch.paused_keep", m.filename);
            else if (ok)
                LOG_INFO << "[DL] " << I18n::tr("dl.fetch.done", m.filename);
            else
                LOG_ERROR << "[DL] " << I18n::tr("dl.fetch.failed", m.filename, log);
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
                runDownload(p);
                g.dlDone.fetch_add(1);
            }
        }

        void ensureDlWorkers() {
            std::lock_guard lk(g.mtx);
            if (!g.dlWorkers.empty()) return;
            g.dlWorkers.reserve(kDlWorkers);
            for (int i = 0; i < kDlWorkers; ++i)
                g.dlWorkers.emplace_back(dlWorker);
        }

        /// 本地文件清单按目录 mtime 缓存：同一目录没改动就不重新扫盘。
        std::vector<Store::LocalFile> cachedLocal(const ProductType t, const int year, const int doy) {
            using Stamp = std::filesystem::file_time_type;
            const std::string dir = Store::dateDir(t, year, doy);
            std::error_code ec;
            const Stamp mt = std::filesystem::last_write_time(dir, ec);
            if (ec) return {};

            std::lock_guard lk(gLocalCache.mtx);
            if (const auto it = gLocalCache.dirs.find(dir); it != gLocalCache.dirs.end() && it->second.first == mt)
                return it->second.second;

            auto &[fst, snd] = gLocalCache.dirs[dir];
            fst = mt;
            snd = Store::listFiles(t, year, doy);
            return snd;
        }

        void appendLocalOnly(std::vector<Match> &list, const ProductType t, const int year, const int doy) {
            std::set<std::string> have;
            for (const auto &mt: list) have.insert(Store::localNameFor(mt.m.filename));

            for (const auto &lf: cachedLocal(t, year, doy)) {
                // 名字自报了日期却覆盖不到所查日的（上一版把 S6A 次日文件错放进当日目录）不进列表；
                // 只认 stamped 的名字，无日期的老名字与归档目录照旧按目录日算。
                if (const auto pm = FileMeta::parse(lf.name);
                    pm && pm->stamped && !pm->covers(year, doy))
                    continue;
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

    std::string Match::remoteUrl() const {
        if (!url.empty()) return url;
        return dirUrl.empty() ? std::string() : dirUrl + m.filename;
    }

    Pending pendingOf(const Match &m) {
        Pending p;
        p.remoteName = m.m.filename;
        p.url = m.remoteUrl();
        p.type = m.m.type;
        p.datacenter = m.m.datacenter;
        p.year = m.m.year;
        p.doy = m.m.doy;
        p.spanDays = m.m.spanDays;
        return p;
    }

    Snapshot collect(const std::vector<SourceTable::Dir> &dirs, const SourceTable::Query &q) {
        ensureScan(dirs, q.year, q.doy, q.scanTextKey);

        Snapshot out;
        {
            std::lock_guard lk(g.mtx);
            std::set<std::string> seen;
            for (const auto &d: dirs) {
                DirState st;
                st.url = d.url;
                const auto it = g.dirs.find(cacheKey(d.type, d.probeName, d.url, q.year, q.doy));
                if (it == g.dirs.end()) {
                    out.sources.push_back(std::move(st));
                    continue;
                }
                st.done = it->second.pending == 0;
                st.ok = it->second.srcOk > 0;
                st.err = it->second.err;
                st.note = it->second.note;
                for (const auto &m: it->second.items) {
                    if (!q.types.count(m.type)) continue;
                    if (!m.covers(q.year, q.doy)) continue;
                    if (!seen.insert(Store::localNameFor(m.filename)).second) continue;
                    std::string src;
                    if (const auto s = it->second.sources.find(m.filename); s != it->second.sources.end())
                        src = s->second;
                    std::string full;
                    if (const auto u = it->second.urls.find(m.filename); u != it->second.urls.end())
                        full = u->second;
                    out.matches.push_back({m, src.empty() ? d.url : src, full, {}});
                }
                out.sources.push_back(std::move(st));
            }
            out.total = static_cast<int>(out.sources.size());
            for (const auto &s: out.sources) {
                if (!s.done) continue;
                ++out.scanned;
                if (!s.ok) ++out.failed;
            }
            out.scanning = out.scanned < out.total;
        }

        for (const auto t: q.types) appendLocalOnly(out.matches, t, q.year, q.doy);
        std::sort(out.matches.begin(), out.matches.end(),
                  [](const Match &a, const Match &b) { return a.m.filename < b.m.filename; });
        return out;
    }

    std::vector<std::string> resolve(const std::vector<SourceTable::Dir> &dirs, const int year,
                                     const int doy, std::string *listErr) {
        std::vector<std::string> files;
        int failed = 0;
        std::string firstFailedHost, firstFailedErr;
        for (const auto &d: dirs) {
            if (!d.probeName.empty()) {
                if (!d.url.empty()) files.push_back(d.url);
                continue;
            }
            std::vector<std::string> names;
            std::map<std::string, std::string> fullUrls;
            std::string err;
            if (!Http::listDir(d.url, names, err, nullptr, &fullUrls)) {
                ++failed;
                if (firstFailedHost.empty()) {
                    const std::string host = d.url.substr(d.url.find("://") + 3);
                    firstFailedHost = host.substr(0, host.find('/'));
                    firstFailedErr = err;
                }
                continue;
            }
            const bool slashDir = !d.url.empty() && d.url.back() == '/';
            for (const auto &n: names) {
                if (!d.namePrefix.empty() && n.rfind(d.namePrefix, 0) != 0) continue;
                // 名字自带起止时刻却覆盖不到当天，或认不出类型：都不是这一天的文件
                if (!FileMeta::parseInDir(n, d.type, year, doy)) continue;
                if (const auto fu = fullUrls.find(n); fu != fullUrls.end()) files.push_back(fu->second);
                else if (slashDir) files.push_back(d.url + n);
            }
        }
        if (listErr)
            *listErr = failed > 0
                           ? I18n::tr("dl.fetch.list_dirs_failed", failed, firstFailedHost,
                                      firstFailedErr)
                           : std::string();
        return files;
    }

    void enqueue(const Pending &p) {
        {
            std::lock_guard lk(g.mtx);
            g.status = I18n::tr("dl.status.downloading");
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

    Fetched fetchNow(const Pending &p, const Http::ProgressCb &progress, const std::atomic<bool> *cancel,
                     const Unpack::Progress &unpack, const std::function<void(const std::string &)> &stage) {
        Fetched r;
        std::string log, local;
        if (!downloadTo(p, progress, cancel, local, log)) {
            r.paused = cancel != nullptr && cancel->load();
            r.err = log.empty() ? I18n::tr("dl.fetch.interrupted") : log;
            return r;
        }
        if (!settleTo(local, metaFromPending(p), unpack, stage, r.path, r.err)) return r;
        r.ok = true;
        return r;
    }

    PostState postStateOf(const std::string &remoteName) {
        std::lock_guard lk(g.mtx);
        if (const auto it = g.postJobs.find(remoteName); it != g.postJobs.end()) return it->second;
        return {};
    }

    bool downloadingOf(const std::string &remote, float &frac, double &totalBytes, double &gotBytes,
                       double &speed) {
        std::lock_guard lk(g.mtx);
        const auto it = g.dlActive.find(remote);
        if (it == g.dlActive.end()) return false;
        frac = it->second.frac;
        totalBytes = it->second.total;
        gotBytes = it->second.got;
        speed = it->second.speed;
        return true;
    }

    void pauseDownload(const std::string &remote) {
        std::shared_ptr<std::atomic<bool> > cancel;
        {
            std::lock_guard lk(g.mtx);
            if (const auto it = g.dlCancel.find(remote); it != g.dlCancel.end()) cancel = it->second;
        }
        if (cancel) cancel->store(true);
    }

    Progress progress() {
        std::lock_guard lk(g.mtx);
        return {g.dlActiveCount.load(), g.dlDone.load(), g.dlTotal.load(), g.status};
    }

    void retryFailed() {
        Http::forgetDownHosts();
        std::lock_guard lk(g.mtx);
        for (auto it = g.dirs.begin(); it != g.dirs.end();) {
            if (it->second.pending == 0 && it->second.srcOk == 0) it = g.dirs.erase(it);
            else ++it;
        }
    }

    void setStatus(const std::string &text) {
        std::lock_guard lk(g.mtx);
        g.status = text;
    }

    void shutdown() {
        {
            std::lock_guard lk(g.mtx);
            g.scanStop = true;
            g.scanQueue.clear();
        }
        g.scanCv.notify_all();
        for (auto &w: g.scanWorkers)
            if (w.joinable()) w.join();
        g.scanWorkers.clear();
        {
            std::lock_guard lk(g.mtx);
            for (auto &[remote, cancel]: g.dlCancel) cancel->store(true);
        }
        {
            std::lock_guard lk(g.dlQmtx);
            g.dlStop = true;
        }
        g.dlCv.notify_all();
        for (auto &w: g.dlWorkers)
            if (w.joinable()) w.join();
        {
            std::lock_guard lk(g.postQmtx);
            g.postStop = true;
            g.postQueue.clear();
        }
        g.postCv.notify_all();
        if (g.postThread.joinable()) g.postThread.join();
    }
}
