#include "ProductStore.h"

#include "CSV.h"
#include "DownloadStore.h"
#include "Log.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <map>
#include <vector>

namespace fs = std::filesystem;

namespace Download::Store {
    namespace {
        std::string indexPath() { return root() + "index.csv"; }
        void dropStale(const std::string &id, const std::string &keepPath) {
            std::error_code ec;
            const fs::directory_iterator it(dir(id), ec);
            if (ec) return;
            for (const auto &ent: it) {
                const std::string n = ent.path().filename().string();
                if (n.rfind(".tmp_", 0) == 0) continue;
                if (ent.path() == fs::path(keepPath)) continue;
                std::error_code e2;
                fs::remove_all(ent.path(), e2);
                if (e2) LOG_WARN << "[产品] 清理旧版本残留失败 " << n << ": " << e2.message();
            }
        }

        std::string nowStamp() {
            const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::tm tm{};
#if defined(_WIN32)
            localtime_s(&tm, &t);
#else
            localtime_r(&t, &tm);
#endif
            char buf[32];
            std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d",
                          tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                          tm.tm_hour, tm.tm_min, tm.tm_sec);
            return buf;
        }

        std::map<std::string, InstalledProduct> readIndex() {
            std::map<std::string, InstalledProduct> out;
            const std::vector<std::vector<std::string> > rows = CSV::Reader::readFile(indexPath());
            for (std::size_t i = 1; i < rows.size(); ++i) { // 第 0 行是表头
                const std::vector<std::string> &cols = rows[i];
                if (cols.size() < 6) continue;
                InstalledProduct p;
                p.id = cols[0];
                p.version = cols[1];
                p.file = cols[2];
                p.path = cols[3];
                try { p.size = std::stoll(cols[4]); } catch (...) { p.size = 0; }
                p.fetchedAt = cols[5];
                if (!p.id.empty()) out[p.id] = p;
            }
            return out;
        }

        void writeIndex(const std::map<std::string, InstalledProduct> &all) {
            std::error_code ec;
            fs::create_directories(root(), ec);
            const std::string tmp = indexPath() + ".tmp";
            {
                CSV::Writer w;
                w.row("id", "version", "file", "path", "size", "fetched_at");
                for (const auto &[_, p]: all)
                    w.row(p.id, p.version, p.file, p.path, std::to_string(p.size), p.fetchedAt);
                if (!w.save(tmp)) return;
            }
            fs::rename(tmp, indexPath(), ec);
            if (ec) LOG_WARN << "[产品] 写索引失败: " << ec.message();
        }
    }

    std::string root() { return DownloadStore::root() + "/aux/"; }

    std::string dir(const std::string &id) { return root() + id + "/"; }

    std::optional<InstalledProduct> active(const std::string &id) {
        const auto all = readIndex();
        const auto it = all.find(id);
        if (it == all.end() || it->second.path.empty()) return std::nullopt;
        if (std::error_code ec; !fs::exists(it->second.path, ec)) return std::nullopt;
        return it->second;
    }

    std::string activeFile(const std::string &id) {
        if (const auto p = active(id)) return p->path;
        return {};
    }

    std::optional<InstalledProduct> active(const VersionedProduct &product) {
        return active(product.id);
    }

    std::string activeFile(const VersionedProduct &product) {
        if (const auto p = active(product)) return p->path;
        return {};
    }

    std::string tempPath(const std::string &id, const std::string &name) {
        return dir(id) + ".tmp_" + name;
    }

    bool ensureDir(const std::string &id) {
        std::error_code ec;
        fs::create_directories(dir(id), ec);
        if (ec) {
            LOG_ERROR << "[产品] 建目录失败 " << dir(id) << ": " << ec.message();
            return false;
        }
        return true;
    }

    void clearStaging(const std::string &id) {
        clearStagingExcept(id, {});
    }

    void clearStagingExcept(const std::string &id, const std::string &keepName) {
        const std::string keep1 = ".tmp_" + keepName;
        const std::string keep2 = keep1 + ".gz";
        std::error_code ec;
        for (const auto &ent: fs::directory_iterator(dir(id), ec)) {
            const std::string n = ent.path().filename().string();
            if (n.rfind(".tmp_", 0) != 0) continue;
            if (!keepName.empty() && (n == keep1 || n == keep2)) continue;
            std::error_code e2;
            fs::remove(ent.path(), e2);
            if (e2) LOG_WARN << "[产品] 清理暂存件失败 " << n << ": " << e2.message();
        }
    }

    bool install(const std::string &id, const std::string &srcPath,
                 const std::string &version, const std::string &fileName,
                 const bool removeSource) {
        std::error_code ec;
        const std::string dstDir = dir(id);
        fs::create_directories(dstDir, ec);
        if (ec) {
            LOG_ERROR << "[产品] 建目录失败 " << dstDir << ": " << ec.message();
            return false;
        }

        const std::string dst = dstDir + fileName;
        std::error_code eqEc;
        if (const bool same = fs::equivalent(srcPath, dst, eqEc) && !eqEc; !same) {
            fs::copy_file(srcPath, dst, fs::copy_options::overwrite_existing, ec);
            if (ec) {
                LOG_ERROR << "[产品] 安装失败 " << dst << ": " << ec.message();
                return false;
            }
            if (removeSource) fs::remove(srcPath, ec);
        }

        InstalledProduct p;
        p.id = id;
        p.version = version;
        p.file = fileName;
        p.path = dst;
        const auto sz = fs::file_size(dst, ec);
        p.size = ec ? 0 : static_cast<long long>(sz);
        p.fetchedAt = nowStamp();

        auto all = readIndex();
        all[id] = p;
        writeIndex(all);

        dropStale(id, dst);

        LOG_INFO << "[产品] 已安装 " << id << " 版本 " << version << " → " << dst
                << " (" << p.size << " 字节)";
        return true;
    }
}
