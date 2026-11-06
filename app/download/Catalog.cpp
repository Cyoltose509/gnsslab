#include "Catalog.h"

#include "Log.h"
#include "Transfer.h"
#include "Store.h"
#include "StringUtils.h"
#include "product/ATXReader.h"
#include "product/GPT3Reader.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace Download::Catalog {
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
                std::smatch m;
                if (std::regex_search(fileName, m, re) && m.size() > 1) return safeVersion(m[1].str());
            } catch (const std::regex_error &) {
            }
            return safeVersion(Str::stripExt(fileName));
        }

        using Clock = std::chrono::steady_clock;

        std::mutex g_mtx;
        std::condition_variable g_cv;
        std::thread g_thread;
        bool g_running = false;
        bool g_kickAll = false;
        bool g_auto = true; // 自动同步：关掉后只响应 checkNow()
        std::set<std::string> g_kick;
        std::map<std::string, UpdateStatus> g_status;
        std::map<std::string, Clock::time_point> g_nextDue;

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

        UpdateStatus make(const UpdateStatus::Phase ph, std::string ver = {}, std::string file = {},
                          std::string detail = {}) {
            UpdateStatus s;
            s.phase = ph;
            s.version = std::move(ver);
            s.file = std::move(file);
            s.detail = std::move(detail);
            return s;
        }

        void checkOne(const VersionedProduct &p) {
            setStatus(p.id, make(UpdateStatus::Checking));

            std::string name, version;
            if (std::string err; !discoverLatest(p, name, version, err)) {
                setStatus(p.id, make(UpdateStatus::Error, {}, {}, err));
                LOG_WARN << "[产品] " << p.label << " 检查失败: " << err;
                scheduleRetry(p);
                return;
            }

            if (const auto cur = Store::active(p.id); cur && cur->version == version) {
                setStatus(p.id, make(UpdateStatus::Ready, version, cur->file, "已是最新"));
                scheduleNext(p);
                return;
            }

            const auto prog = [&](const double total, const double now) {
                setProgress(p.id, name, total, now);
            };
            if (!Store::ensureDir(p.id)) {
                setStatus(p.id, make(UpdateStatus::Error, version, name, "无法创建本地目录"));
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
            if (const std::string gzTmp = Store::tempPath(p.id, name + ".gz"); Transfer::downloadFile(p.indexUrl + name + ".gz", gzTmp, prog, nullptr, 3, 0)) {
                if (const std::string got = Store::decompressIfNeeded(gzTmp); !Str::endsWithIC(got, ".gz")) {
                    installedFrom = got;
                    ok = true;
                }
            }
            if (!ok) {
                if (const std::string rawTmp = Store::tempPath(p.id, name); Transfer::downloadFile(p.indexUrl + name, rawTmp, prog, nullptr, 3, 0)) {
                    installedFrom = rawTmp;
                    ok = true;
                }
            }
            if (!ok || !Store::install(p.id, installedFrom, version, name)) {
                setStatus(p.id, make(UpdateStatus::Error, version, name, "下载或安装失败"));
                LOG_WARN << "[产品] " << p.label << " 获取失败: " << name;
                scheduleRetry(p);
                return;
            }
            setStatus(p.id, make(UpdateStatus::Ready, version, name, "已更新"));
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
                        for (const auto &p: all()) todo.emplace_back(p.id);
                    } else if (!g_kick.empty()) {
                        todo.assign(g_kick.begin(), g_kick.end());
                        g_kick.clear();
                    }
                    const auto now = Clock::now();
                    if (g_auto) {
                        for (const auto &p: all()) {
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
                    if (const auto *p = find(id)) checkOne(*p);
                }
            }
        }
    }

    const std::vector<VersionedProduct> &all() {
        static const std::vector<VersionedProduct> v = {
            {
                "atx",
                "天线文件 (ANTEX)",
                "https://files.igs.org/pub/station/general/",
                R"(igs20_([0-9]{4})\.atx)",
                "igs20.atx",
                24,
                {L"atx", L"ATX", L"atx.gz"},
                ATXReader::validate,
            },
            {
                "gpt3",
                "对流层格网 (GPT3)",
                "https://vmf.geo.tuwien.ac.at/codes/",
                "",
                "gpt3_5.grd",
                24,
                {L"grd", L"GRD"},
                GPT3Reader::validate,
            },
        };
        return v;
    }

    const VersionedProduct *find(const std::string &id) {
        for (const auto &p: all())
            if (id == p.id) return &p;
        return nullptr;
    }

    const VersionedProduct &atx() {
        const auto *p = find("atx");
        assert(p && "版本型产品 atx 必须在目录表 Catalog::all() 内");
        return *p;
    }

    const VersionedProduct &gpt3() {
        const auto *p = find("gpt3");
        assert(p && "版本型产品 gpt3 必须在目录表 Catalog::all() 内");
        return *p;
    }

    bool discoverLatest(const VersionedProduct &p, std::string &fileName,
                        std::string &version, std::string &err) {
        fileName.clear();
        version.clear();

        if (std::string body; p.pattern && *p.pattern && Transfer::fetchText(p.indexUrl, body, err)) {
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
                err = std::string("正则错误: ") + e.what();
            }
        }

        if (p.fallback && *p.fallback) {
            fileName = p.fallback;
            version = "latest";
            LOG_WARN << "[产品] " << p.label << " 未发现版本化文件名（"
                    << (err.empty() ? std::string("索引中无匹配") : "索引获取失败: " + err)
                    << "），回落 " << p.fallback << "（该产品将无法自动判断更新）";
            return true;
        }
        if (err.empty()) err = "索引中未找到匹配文件";
        return false;
    }

    ImportResult importLocal(const VersionedProduct &p, const std::string &srcPath) {
        ImportResult r;
        std::error_code ec;

        if (!fs::is_regular_file(srcPath, ec)) {
            r.detail = "源文件不存在或不可读";
            return r;
        }
        if (const auto srcSize = fs::file_size(srcPath, ec); ec || srcSize < 1024) {
            r.detail = "源文件过小（<1 KB），不像是完整的产品文件";
            return r;
        }

        if (!Store::ensureDir(p.id)) {
            r.detail = "无法创建产品库目录";
            return r;
        }
        std::string work = srcPath;
        std::string fileName = Str::baseName(srcPath);
        bool staged = false;
        if (Str::endsWithIC(srcPath, ".gz")) {
            work = Store::tempPath(p.id, fileName);
            fs::copy_file(srcPath, work, fs::copy_options::overwrite_existing, ec);
            if (ec) {
                r.detail = "拷贝到产品库暂存区失败: " + ec.message();
                LOG_ERROR << "[产品] 导入拷贝失败 " << srcPath << " → " << work << ": " << ec.message();
                return r;
            }
            const std::string got = Store::decompressIfNeeded(work);
            if (Str::endsWithIC(got, ".gz")) { // 解压失败时原样返回 .gz
                fs::remove(work, ec);
                r.detail = "解压失败（压缩包可能已损坏）";
                return r;
            }
            work = got;
            staged = true;
            fileName = Str::baseName(work).substr(5); // 去掉暂存前缀 ".tmp_"
        }

        std::string detail;
        if (p.validate && !p.validate(work, detail)) {
            if (staged) fs::remove(work, ec);
            LOG_WARN << "[产品] 导入校验未通过 " << srcPath << ": " << detail;
            r.detail = detail;
            return r;
        }
        const std::string version = versionFor(p, fileName);

        if (!Store::install(p.id, work, version, fileName, staged)) {
            if (staged) {
                std::error_code e2;
                fs::remove(work, e2);
            }
            r.detail = "写入产品库失败";
            return r;
        }

        r.ok = true;
        r.file = fileName;
        r.version = version;
        r.detail = detail.empty() ? "已导入" : detail;
        LOG_INFO << "[产品] 手工导入 " << p.label << " " << fileName << "（" << r.detail << "）";
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
        setStatus(id, make(UpdateStatus::Ready, version, file, detail.empty() ? "已导入" : detail));
        if (const auto *p = find(id)) {
            std::lock_guard lk(g_mtx);
            g_nextDue[id] = Clock::now() + std::chrono::hours(p->checkHours);
        }
    }
}
