#include "download/remote/Catalog.h"
#include "FileUtils.h"

#include "Log.h"
#include "util/Clock.h"
#include "download/remote/http/Core.h"
#include "download/remote/SourceTable.h"
#include "download/local/ProductType.h"
#include "core/TimeStruct.h"
#include "download/local/Store.h"
#include "download/local/Unpack.h"
#include "io/SinexReader.h"
#include "StringUtils.h"
#include "i18n/I18n.h"
#include "product/ATXReader.h"
#include "product/GPT3Reader.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace Download::Catalog {
    const std::vector<VersionedProduct> kProducts{
        {
            kGPT3ID,
            ProductType::GPT3,
            "https://vmf.geo.tuwien.ac.at/codes/",
            "",
            "gpt3_5.grd",
            24,
            {L"grd", L"GRD"},
            GPT3Reader::validate,
        },
        {
            kATXID,
            ProductType::ATX,
            "https://files.igs.org/pub/station/general/",
            R"(igs20_(\d{4})\.atx)",
            "igs20.atx",
            24,
            {L"atx", L"ATX", L"gz", L"GZ"},
            ATXReader::validate,
        },
    };

    namespace {
        std::string safeVersion(std::string v) {
            for (char &c: v) {
                const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                                (c >= 'A' && c <= 'Z') || c == '.' || c == '_' || c == '-';
                if (!ok) c = '_';
            }
            return v.empty() ? std::string("local") : v;
        }

        std::string versionFor(const VersionedProduct &p, const std::string &fileName) {
            try {
                const std::regex re(p.pattern);
                if (std::smatch m; std::regex_search(fileName, m, re) && m.size() > 1) return safeVersion(m[1].str());
            } catch (const std::regex_error &) {
            }
            // 名字里没有版本号（别名 igs20.atx、固定名 gpt3_5.grd）：与自动更新的回落同一口径，
            // 否则同一份文件会攒出 latest / igs20 / gpt3_5 三种版本串，去重与比较都没法写。
            return "latest";
        }

        /// 版本号是纯数字时给出数值，否则 -1：只有两边都是数字才比得出大小。
        long long versionNumber(const std::string &v) {
            long long n = 0;
            const char *const first = v.data();
            const char *const last = first + v.size();
            if (auto [ptr, ec] = std::from_chars(first, last, n);
                ec == std::errc{} && ptr == last && !v.empty())
                return n;
            return -1;
        }

        std::mutex g_mtx;
        std::condition_variable g_cv;
        std::thread g_thread;
        bool g_running = false;
        bool g_kickAll = false;
        bool g_auto = true; // 自动同步：关掉后只响应 checkNow()
        std::set<std::string> g_kick;
        std::map<std::string, UpdateStatus> g_status;
        std::map<std::string, Clock::Point> g_nextDue;

        void setStatus(const std::string &id, UpdateStatus s) {
            std::lock_guard lk(g_mtx);
            g_status[id] = std::move(s);
        }

        void setProgress(const std::string &id, const std::string &file, const double total,
                         const double now) {
            std::lock_guard lk(g_mtx);
            auto &s = g_status[id];
            s.phase = UpdateStatus::Downloading;
            s.file = file;
            s.progress = Http::fracOf(total, now);
            s.gotBytes = now;
            s.totalBytes = total;
        }

        void scheduleNext(const VersionedProduct &p) {
            std::lock_guard lk(g_mtx);
            g_nextDue[p.id] = Clock::after(Clock::hours(p.checkHours));
        }

        void scheduleRetry(const VersionedProduct &p) {
            std::lock_guard lk(g_mtx);
            g_nextDue[p.id] = Clock::after(Clock::minutes(10));
        }

        double stagedBytes(const std::string &id, const std::string &name) {
            for (const std::string &p: {Store::tempPath(id, name + ".gz"), Store::tempPath(id, name)}) {
                std::error_code ec;
                const auto sz = std::filesystem::file_size(p, ec);
                if (!ec) return static_cast<double>(sz);
            }
            return 0.0;
        }

        UpdateStatus make(const UpdateStatus::Phase ph, std::string ver = {}, std::string file = {},
                          std::string detail = {}) {
            UpdateStatus s;
            s.phase = ph;
            s.version = std::move(ver);
            s.file = std::move(file);
            s.detail = std::move(detail);
            return s;
        }

        bool installLatest(const VersionedProduct &p, std::string &detail) {
            std::string name, version;
            if (std::string err; !discoverLatest(p, name, version, err)) {
                detail = err;
                return false;
            }
            // 无版本号的固定别名只在首次安装时用：它和已装的版本化文件是同一份内容，顶掉之后
            // 下次拿到版本号再反向顶回来，来回各下一遍。远端索引也可能被缓存成旧的一份，同样不动。
            const auto cur = Store::active(p.id);
            const bool unversioned = version == "latest";
            const long long remoteVer = versionNumber(version);
            const long long localVer = cur ? versionNumber(cur->version) : -1;
            if (cur && (unversioned || cur->version == version ||
                        (remoteVer >= 0 && localVer >= 0 && remoteVer < localVer))) {
                detail = I18n::trc("dl.prod.up_to_date");
                return true;
            }

            const auto prog = [&](const double total, const double now) {
                setProgress(p.id, name, total, now);
            };
            if (!Store::ensureDir(p.id)) {
                detail = I18n::tr("dl.prod.mkdir_local_failed");
                return false;
            }
            // 只保留本次要下的那个名字的断点续传临时文件；回落名与版本名共存时别互相清。
            Store::clearStagingExcept(p.id, name);
            const double have = stagedBytes(p.id, name);
            if (have > 0.0)
                LOG_INFO << "[PROD] " << I18n::tr("dl.prod.resume", typeLabel(p.type),
                                                  static_cast<long long>(have));
            setProgress(p.id, name, 0.0, have);

            bool ok = false;
            std::string installedFrom;
            if (const std::string gzTmp = Store::tempPath(p.id, name + ".gz");
                Http::downloadFile(p.indexUrl + name + ".gz", gzTmp, prog, nullptr, 3, 0)) {
                if (const std::string got = Unpack::settle(gzTmp).main; !Str::endsWithIC(got, ".gz")) {
                    installedFrom = got;
                    ok = true;
                }
            }
            if (!ok) {
                if (const std::string rawTmp = Store::tempPath(p.id, name);
                    Http::downloadFile(p.indexUrl + name, rawTmp, prog, nullptr, 3, 0)) {
                    installedFrom = rawTmp;
                    ok = true;
                }
            }
            // 服务器回 200 但正文是登录页/错误页时，别把 HTML 当产品装进库里（手工导入同样要过这一关）。
            if (std::string bad; ok && p.validate && !p.validate(installedFrom, bad)) {
                std::error_code rmEc;
                std::filesystem::remove(installedFrom, rmEc);
                LOG_WARN << "[PROD] " << I18n::tr("dl.prod.validate_failed", typeLabel(p.type),
                                                  name, bad);
                detail = name + "：" + bad;
                return false;
            }
            if (!ok || !Store::install(p.id, installedFrom, version, name)) {
                detail = I18n::tr("dl.prod.download_failed");
                return false;
            }
            detail = name;
            return true;
        }

        void checkOne(const VersionedProduct &p) {
            setStatus(p.id, make(UpdateStatus::Checking));

            std::string detail;
            if (!installLatest(p, detail)) {
                setStatus(p.id, make(UpdateStatus::Error, {}, {}, detail));
                LOG_WARN << "[PROD] " << I18n::tr("dl.prod.fetch_failed", typeLabel(p.type), detail);
                scheduleRetry(p);
                return;
            }
            setStatus(p.id, make(UpdateStatus::Ready, {}, {}, detail));
            LOG_INFO << "[PROD] " << I18n::tr("dl.prod.synced", typeLabel(p.type), detail);
            scheduleNext(p);
        }

        void loop() {
            {
                std::unique_lock lk(g_mtx);
                g_cv.wait_for(lk, Clock::seconds(2), [] { return !g_running; });
            }
            while (true) {
                std::vector<std::string> todo;
                {
                    std::unique_lock lk(g_mtx);
                    if (!g_running) return;
                    if (g_kickAll) {
                        g_kickAll = false;
                        g_kick.clear();
                        for (const auto &p: kProducts) todo.emplace_back(p.id);
                    } else if (!g_kick.empty()) {
                        todo.assign(g_kick.begin(), g_kick.end());
                        g_kick.clear();
                    }
                    if (g_auto) {
                        for (const auto &p: kProducts) {
                            const auto it = g_nextDue.find(p.id);
                            if (const bool kick = it == g_nextDue.end() || Clock::due(it->second);
                                kick && std::find(todo.begin(), todo.end(), p.id) == todo.end())
                                todo.emplace_back(p.id);
                        }
                    }
                    if (todo.empty()) {
                        g_cv.wait_for(lk, Clock::minutes(5), [] {
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
                    if (const auto *p = find(id)) checkOne(*p);
                }
            }
        }

        /// 从 SINEX 明文里抽出某个站的 SOLUTION/ESTIMATE 行
        std::string stationEstimateBlock(const std::string &plain, const std::string &site) {
            std::istringstream ss(plain);
            std::string line, body;
            bool inEstimate = false;
            while (std::getline(ss, line)) {
                if (line.find("+SOLUTION/ESTIMATE") != std::string::npos) {
                    inEstimate = true;
                    continue;
                }
                if (line.find("-SOLUTION/ESTIMATE") != std::string::npos) break;
                if (!inEstimate) continue;
                std::istringstream ls(Str::strip(line));
                std::vector<std::string> tk;
                for (std::string w; ls >> w;) tk.push_back(w);
                if (tk.size() < 9 || Str::toUpper(tk[2]) != site) continue;
                if (const std::string &type = tk[1]; type != "STAX" && type != "STAY" && type != "STAZ" && type != "VX" && type != "VY" &&
                                                     type != "VZ")
                    continue;
                body += line;
                body += '\n';
            }
            if (body.empty()) return {};
            return "+SOLUTION/ESTIMATE\n" + body + "-SOLUTION/ESTIMATE\n";
        }

        /// 在单个候选里捞这一站的 ESTIMATE 行
        bool probeSite(const std::string &url, const std::string &name, const std::string &want,
                       const std::string &gz, const std::function<void(long long)> &onBytes,
                       std::string &block, std::string &detail) {
            constexpr long long kFirst = 1LL << 18;
            constexpr long long kCap = 1LL << 23;
            std::error_code ec;
            std::filesystem::remove(gz, ec);
            long long have = 0;
            long long target = kFirst;
            for (;;) {
                const long long got = target - have;
                if (!Http::downloadPartial(url, gz, got, {}, 0, nullptr, have, have > 0)) break;
                const auto size = std::filesystem::file_size(gz, ec);
                if (ec || size == 0) break;
                const bool atEnd = static_cast<long long>(size) < target; // 远端比要的短：已经取完
                have = static_cast<long long>(size);
                if (onBytes) onBytes(got);

                const std::string plain = Unpack::inflateHead(gz, true);
                if (plain == gz) break; // 不是能解的压缩流，放大也一样，换下一个候选
                std::ifstream fi(plain, std::ios::binary);
                const std::string text((std::istreambuf_iterator(fi)), std::istreambuf_iterator<char>());
                fi.close();
                std::filesystem::remove(plain, ec);

                if (text.find("-SOLUTION/ESTIMATE") != std::string::npos) {
                    SinexReader reader;
                    if (std::string err; reader.readString(text, err) && reader.hasStation(want))
                        if (block = stationEstimateBlock(text, want); !block.empty()) {
                            detail = I18n::tr("dl.truth.site_from_stream", name, want);
                            std::filesystem::remove(gz, ec);
                            return true;
                        }
                    break; // 块已完整：这份文件里没有该站
                }
                if (atEnd || have >= kCap) break;
                target = std::min(have * 2, kCap);
            }
            std::filesystem::remove(gz, ec);
            return false;
        }

        /// 覆盖该日的 SINEX 候选
        std::vector<std::pair<std::string, std::string> > siteCands(const int year, const int doy) {
            char stamp[16];
            std::snprintf(stamp, sizeof(stamp), "%04d%03d", year, doy);
            const auto weekDirs = SourceTable::snxWeekDir(GPSWeek(year, doy));
            std::vector<std::pair<std::string, std::string> > out;
            for (const std::string &base: {weekDirs.url, weekDirs.backupUrl}) {
                if (base.empty()) continue;
                std::vector<std::string> names;
                if (std::string err; !Http::listDir(base, names, err)) continue;
                std::vector<std::string> hit;
                for (const auto &n: names)
                    if (n.find(stamp) != std::string::npos && Str::endsWithIC(n, ".SNX.gz")) hit.push_back(n);
                std::sort(hit.begin(), hit.end(), [](const std::string &a, const std::string &b) {
                    if (const bool ca = a.rfind("COD0OPS", 0) == 0, cb = b.rfind("COD0OPS", 0) == 0; ca != cb) return ca;
                    return a < b;
                });
                std::set<std::string> seen;
                for (const auto &n: hit) {
                    if (!seen.insert(n.substr(0, n.find('_'))).second) continue;
                    out.emplace_back(base + n, n);
                }
                if (!out.empty()) break; // 主站列到就够了，镜像与它同内容
            }
            return out;
        }
    }

    const VersionedProduct *find(const std::string &id) {
        for (const auto &p: kProducts)
            if (id == p.id) return &p;
        return nullptr;
    }

    bool fetchSiteTruth(const std::string &site, const int year, const int doy, std::string &block,
                        std::string &detail, const Http::ProgressCb &progress) {
        block.clear();
        const std::string want = Str::toUpper(site.substr(0, std::min<std::size_t>(4, site.size())));
        if (want.size() < 4) {
            detail = I18n::tr("dl.truth.site_too_short", site);
            return false;
        }

        const auto cands = siteCands(year, doy);
        if (cands.empty()) {
            detail = I18n::tr("dl.truth.no_sinex");
            return false;
        }

        std::error_code ec;
        std::filesystem::create_directories(Store::productRoot(), ec);

        constexpr long long kFirst = 1LL << 18;
        constexpr std::size_t kWorkers = 4;
        const std::size_t total = cands.size() * static_cast<std::size_t>(kFirst);
        std::atomic<std::size_t> fetched{0};
        const auto onBytes = [&](const long long got) {
            const std::size_t now = fetched.fetch_add(static_cast<std::size_t>(got)) + got;
            if (progress) progress(static_cast<double>(std::max(total, now)), static_cast<double>(now));
        };

        std::size_t from = 0;
        if (cands[0].second.rfind("COD0OPS", 0) == 0) {
            if (probeSite(cands[0].first, cands[0].second, want, Store::productRoot() + ".part_snx0.gz",
                          onBytes, block, detail))
                return true;
            from = 1;
        }

        std::atomic next{from};
        std::atomic found{false};
        std::mutex hitMtx;
        std::string hitBlock, hitDetail;
        std::vector<std::thread> workers;
        for (std::size_t w = 0; w < std::min(kWorkers, cands.size() - from); ++w) {
            workers.emplace_back([&, w] {
                const std::string gz = Store::productRoot() + ".part_snx" + std::to_string(w + 1) + ".gz";
                for (std::size_t i; (i = next.fetch_add(1)) < cands.size() && !found.load();) {
                    std::string got, note;
                    bool ok = false;
                    try {
                        ok = probeSite(cands[i].first, cands[i].second, want, gz, onBytes, got, note);
                    } catch (const std::exception &e) {
                        LOG_WARN << "[TRUTH] " << I18n::tr("dl.truth.read_failed", cands[i].second,
                                                           e.what());
                    } catch (...) {
                        LOG_WARN << "[TRUTH] " << I18n::tr("dl.truth.read_failed_unknown",
                                                           cands[i].second);
                    }
                    if (!ok) continue;
                    std::lock_guard lk(hitMtx);
                    if (found.exchange(true)) return;
                    hitBlock = std::move(got);
                    hitDetail = std::move(note);
                    return;
                }
            });
        }
        for (auto &t: workers) t.join();
        if (found.load()) {
            block = std::move(hitBlock);
            detail = std::move(hitDetail);
            return true;
        }
        block.clear();

        detail = I18n::tr("dl.truth.tried_all", cands.size(), want);
        return false;
    }

    bool ensureSiteTruth(const std::string &site, const int year, const int doy, double xyz[3],
                         bool &fromCache, std::string &detail, const Http::ProgressCb &progress) {
        static std::mutex siteTruthMtx;
        fromCache = false;
        const std::string want = Str::toUpper(site.substr(0, std::min<std::size_t>(4, site.size())));
        if (want.size() < 4) {
            detail = I18n::tr("dl.truth.site_too_short", site);
            return false;
        }
        std::error_code ec;
        std::filesystem::create_directories(Store::productRoot(), ec);
        const std::string cache = Store::productRoot() + "ref_truth_sites.snx";

        std::string err;
        Eigen::Vector3d v;
        int usedYear = 0, usedDoy = 0;
        {
            std::lock_guard lk(siteTruthMtx);
            if (SinexReader have; have.readFile(cache, err) &&
                                  have.getStation(want, year, doy, v, &usedYear, &usedDoy)) {
                xyz[0] = v.x();
                xyz[1] = v.y();
                xyz[2] = v.z();
                fromCache = true;
                detail = I18n::tr("dl.truth.cache_hit", want);
                if (usedYear > 0) {
                    char stamp[24];
                    std::snprintf(stamp, sizeof stamp, I18n::trc("dl.truth.solution_stamp"),
                                  usedYear, usedDoy);
                    detail += stamp;
                }
                return true;
            }
        }

        std::string block;
        if (!fetchSiteTruth(want, year, doy, block, detail, progress)) return false;
        // 先解析校验再落盘：服务器回错误页时的坏块一旦进缓存就永久驻留。
        int refYear = 0, refDoy = 0;
        if (SinexReader fresh; !fresh.readString(block, err) ||
                               !fresh.getStation(want, year, doy, v, &refYear, &refDoy)) {
            detail = I18n::tr("dl.truth.sinex_parse_failed", err);
            return false;
        }
        // 同一期解只留一份：重复追加会让缓存无限膨胀，同一站还会出现多条同历元记录。
        {
            std::lock_guard lk(siteTruthMtx);
            if (SinexReader cached; !cached.readFile(cache, err) || !cached.hasSolution(want, refYear, refDoy)) {
                if (std::ofstream fo(cache, std::ios::binary | std::ios::app); fo) fo << block;
            }
        }
        xyz[0] = v.x();
        xyz[1] = v.y();
        xyz[2] = v.z();
        return true;
    }

    bool discoverLatest(const VersionedProduct &p, std::string &fileName,
                        std::string &version, std::string &err) {
        fileName.clear();
        version.clear();

        if (p.pattern && *p.pattern) {
            std::string body;
            // 索引取不到（超时/DNS/5xx）时远端到底有什么是未知的，不能当成「没有版本化文件名」去
            // 回落固定名——那会把本地已装好的版本顶掉，下一次检查拿到版本号又反向顶回来。
            if (!Http::fetchText(p.indexUrl, body, err)) return false;
            try {
                const std::regex re(p.pattern);
                int best = -1;
                for (std::sregex_iterator it(body.begin(), body.end(), re), end; it != end; ++it) {
                    if (const int v = Str::toInt((*it)[1].str()); v > best) {
                        best = v;
                        version = (*it)[1].str();
                        fileName = (*it)[0].str();
                    }
                }
                if (best >= 0) return true;
            } catch (const std::regex_error &e) {
                err = I18n::tr("dl.prod.regex_error", e.what());
                return false;
            }
        }

        if (p.fallback && *p.fallback) {
            fileName = p.fallback;
            version = "latest";
            std::string why = I18n::tr("dl.prod.index_no_match");
            // 固定文件名的产品（GPT3）压根没有版本化名，能比的只有远端的 Last-Modified：
            // 取到就把它当版本号，此后「改没改过」退化成一次 HEAD。连不上就别断言「最新」，
            // 只有连上了而服务器确实不给时间戳，才退回「不可判更新」。
            if (!(p.pattern && *p.pattern)) {
                std::string stamp;
                bool reachable = false;
                if (Http::probeModified(std::string(p.indexUrl) + p.fallback, stamp, reachable, why)) {
                    version = stamp;
                    return true;
                }
                if (!reachable) {
                    err = why;
                    return false;
                }
            }
            LOG_WARN << "[PROD] " << I18n::tr("dl.prod.no_versioned_name", typeLabel(p.type), why,
                                              p.fallback);
            return true;
        }
        if (err.empty()) err = I18n::tr("dl.prod.index_no_match_long");
        return false;
    }

    ImportResult importLocal(const VersionedProduct &p, const std::string &srcPath) {
        ImportResult r;
        std::error_code ec;

        if (!fs::is_regular_file(srcPath, ec)) {
            r.detail = I18n::tr("dl.prod.src_missing");
            return r;
        }
        if (const auto srcSize = fs::file_size(srcPath, ec); ec || srcSize < 1024) {
            r.detail = I18n::tr("dl.prod.src_too_small");
            return r;
        }

        if (!Store::ensureDir(p.id)) {
            r.detail = I18n::tr("dl.prod.mkdir_local_failed");
            return r;
        }
        std::string work = srcPath;
        std::string fileName = File::baseName(srcPath);
        bool staged = false;
        if (Str::endsWithIC(srcPath, ".gz")) {
            work = Store::tempPath(p.id, fileName);
            fs::copy_file(srcPath, work, fs::copy_options::overwrite_existing, ec);
            if (ec) {
                r.detail = I18n::tr("dl.prod.stage_copy_failed", ec.message());
                LOG_ERROR << "[PROD] " << I18n::tr("dl.prod.import_copy_failed", srcPath, work,
                                                    ec.message());
                return r;
            }
            const std::string got = Unpack::settle(work).main;
            if (Str::endsWithIC(got, ".gz")) {
                fs::remove(work, ec);
                r.detail = I18n::tr("dl.prod.inflate_failed");
                return r;
            }
            work = got;
            staged = true;
            fileName = File::baseName(work).substr(5); // 去掉暂存前缀 ".tmp_"
        }

        std::string detail;
        if (p.validate && !p.validate(work, detail)) {
            if (staged) fs::remove(work, ec);
            LOG_WARN << "[PROD] " << I18n::tr("dl.prod.import_validate_failed", srcPath, detail);
            r.detail = detail;
            return r;
        }
        const std::string version = versionFor(p, fileName);

        if (!Store::install(p.id, work, version, fileName, staged)) {
            if (staged) {
                std::error_code e2;
                fs::remove(work, e2);
            }
            r.detail = I18n::tr("dl.prod.write_library_failed");
            return r;
        }

        r.ok = true;
        r.file = fileName;
        r.version = version;
        r.detail = detail.empty() ? I18n::tr("dl.prod.imported") : detail;
        LOG_INFO << "[PROD] " << I18n::tr("dl.prod.import_done", typeLabel(p.type), fileName,
                                          r.detail);
        return r;
    }

    void startUpdates() {
        std::thread old;
        {
            std::lock_guard lk(g_mtx);
            if (g_running) return;
            g_running = true;
            old = std::move(g_thread);
            g_thread = std::thread(loop);
        }
        if (old.joinable()) old.join();
    }

    void stopUpdates() {
        std::thread old;
        {
            std::lock_guard lk(g_mtx);
            if (!g_running) return;
            g_running = false;
            old = std::move(g_thread);
        }
        g_cv.notify_all();
        if (old.joinable()) old.join();
    }

    void checkNow(const std::string &id) {
        {
            std::lock_guard lk(g_mtx);
            if (id.empty()) g_kickAll = true;
            else g_kick.insert(id);
        }
        g_cv.notify_all();
    }

    bool updateProduct(const std::string &id, std::string &detail) {
        const VersionedProduct *p = find(id);
        if (!p) {
            detail = I18n::tr("dl.prod.unknown_aux", id);
            return false;
        }
        return installLatest(*p, detail);
    }

    void setUpdatesEnabled(const bool on) {
        {
            std::lock_guard lk(g_mtx);
            if (g_auto == on) return;
            g_auto = on;
            if (on) g_nextDue.clear();
        }
        g_cv.notify_all();
    }

    bool updatesEnabled() {
        std::lock_guard lk(g_mtx);
        return g_auto;
    }

    UpdateStatus updateStatus(const std::string &id) {
        std::lock_guard lk(g_mtx);
        const auto it = g_status.find(id);
        return it == g_status.end() ? UpdateStatus{} : it->second;
    }

    void noteImported(const std::string &id, const std::string &version,
                      const std::string &file, const std::string &detail) {
        setStatus(id, make(UpdateStatus::Ready, version, file,
                           detail.empty() ? I18n::tr("dl.prod.imported") : detail));
        if (const auto *p = find(id)) {
            std::lock_guard lk(g_mtx);
            g_nextDue[id] = Clock::after(Clock::hours(p->checkHours));
        }
    }
}
