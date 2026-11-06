#include "Store.h"

#include "CSV.h"
#include "Log.h"
#include "StringUtils.h"
#include "core/CRXDecoder.h"

#include <windows.h>
#include <shellapi.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <vector>

#include <zlib.h>

namespace fs = std::filesystem;

namespace Download::Store {
    namespace {
        std::string manifestPath() {
            return downloadRoot() + "/manifest.jsonl";
        }

        std::mutex &manifestMutex() {
            static std::mutex m;
            return m;
        }

        std::mutex &decodeMutex() {
            static std::mutex m;
            return m;
        }

        bool isShortCompactName(const std::string &name) {
            return name.size() >= 4 && name[name.size() - 4] == '.' &&
                   std::isdigit(static_cast<unsigned char>(name[name.size() - 3])) != 0 &&
                   std::isdigit(static_cast<unsigned char>(name[name.size() - 2])) != 0 &&
                   (name.back() == 'd' || name.back() == 'D');
        }

        bool isCompactName(const std::string &name) {
            const std::string bare = Str::stripSuffixIC(name, ".gz");
            return Str::endsWithIC(bare, ".crx") || isShortCompactName(bare);
        }

        std::string rinexNameFor(const std::string &name) {
            if (Str::endsWithIC(name, ".crx")) {
                const bool upper = name.substr(name.size() - 3) == "CRX";
                return name.substr(0, name.size() - 3) + (upper ? "RNX" : "rnx");
            }
            if (isShortCompactName(name)) {
                std::string out = name;
                out.back() = name.back() == 'd' ? 'o' : 'O';
                return out;
            }
            return name + ".rnx";
        }

        std::string escapeJson(const std::string &s) {
            std::string o;
            o.reserve(s.size() + 2);
            for (const char c: s) {
                if (c == '"' || c == '\\') {
                    o += '\\';
                    o += c;
                } else o += c;
            }
            return o;
        }

        std::string extractField(const std::string &line, const std::string &key) {
            const std::string pat = "\"" + key + "\":\"";
            const auto p = line.find(pat);
            if (p == std::string::npos) return "";
            std::string o;
            for (std::size_t i = p + pat.size(); i < line.size(); ++i) {
                const char c = line[i];
                if (c == '\\' && i + 1 < line.size()) {
                    o += line[++i];
                    continue;
                }
                if (c == '"') return o;
                o += c;
            }
            return "";
        }

        long long extractNum(const std::string &line, const std::string &key) {
            const std::string pat = "\"" + key + "\":";
            auto p = line.find(pat);
            if (p == std::string::npos) return 0;
            p += pat.size();
            const char *b = line.c_str() + p;
            return std::strtoll(b, nullptr, 10);
        }

        void writeEntry(std::ostream &os, const Entry &e) {
            os << "{"
                    << R"("type":")" << escapeJson(e.type) << "\","
                    << "\"year\":" << e.year << ","
                    << "\"doy\":" << e.doy << ","
                    << R"("datacenter":")" << escapeJson(e.datacenter) << "\","
                    << R"("file":")" << escapeJson(e.file) << "\","
                    << R"("path":")" << escapeJson(e.path) << "\","
                    << "\"size\":" << e.size
                    << "}\n";
        }

        std::vector<Entry> readManifestLocked() {
            std::vector<Entry> out;
            std::ifstream f(manifestPath());
            if (!f) return out;
            std::string line;
            while (std::getline(f, line)) {
                if (line.find("\"type\"") == std::string::npos) continue;
                Entry e;
                e.type = extractField(line, "type");
                e.year = static_cast<int>(extractNum(line, "year"));
                e.doy = static_cast<int>(extractNum(line, "doy"));
                e.datacenter = extractField(line, "datacenter");
                e.file = extractField(line, "file");
                e.path = extractField(line, "path");
                e.size = extractNum(line, "size");
                out.push_back(std::move(e));
            }
            return out;
        }

        void writeManifestLocked(const std::vector<Entry> &all) {
            std::error_code ec;
            fs::create_directories(downloadRoot(), ec);
            const std::string tmp = manifestPath() + ".tmp";
            {
                std::ofstream f(tmp, std::ios::trunc);
                if (!f) return;
                for (const auto &e: all) writeEntry(f, e);
                if (!f) return; // 写失败就不 rename，宁可留旧文件
            }
            fs::rename(tmp, manifestPath(), ec);
            if (ec) {
                LOG_WARN << "[下载] 写索引失败: " << ec.message();
            }
        }

        std::string indexPath() { return productRoot() + "index.csv"; }

        void dropStale(const std::string &id, const std::string &keepPath) {
            std::error_code ec;
            const fs::directory_iterator it(productDir(id), ec);
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

        std::map<std::string, Installed> readIndex() {
            std::map<std::string, Installed> out;
            const std::vector<std::vector<std::string> > rows = CSV::Reader::readFile(indexPath());
            for (std::size_t i = 1; i < rows.size(); ++i) { // 第 0 行是表头
                const std::vector<std::string> &cols = rows[i];
                if (cols.size() < 6) continue;
                Installed p;
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

        void writeIndex(const std::map<std::string, Installed> &all) {
            std::error_code ec;
            fs::create_directories(productRoot(), ec);
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

    std::string downloadRoot() {
        wchar_t buf[MAX_PATH] = {};
        GetModuleFileNameW(nullptr, buf, MAX_PATH);
        fs::path p(buf);
        p = p.parent_path() / "downloads";
        std::string s = p.string();
        std::replace(s.begin(), s.end(), '\\', '/');
        return s;
    }

    std::string dateDir(const ProductType t, const int year, const int doy) {
        return downloadRoot() + "/" + ProductMeta::typeDir(t) + "/" +
               std::to_string(year) + "/" + std::to_string(doy) + "/";
    }

    std::string datePath(const ProductType t, const int year, const int doy, const std::string &filename) {
        return dateDir(t, year, doy) + filename;
    }

    std::string localNameFor(const std::string &remoteName) {
        const std::string bare = Str::stripSuffixIC(remoteName, ".gz");
        if (isCompactName(remoteName)) return rinexNameFor(bare);
        return bare;
    }

    std::string findLocal(const ProductType t, const int year, const int doy, const std::string &remoteName) {
        const std::string dir = dateDir(t, year, doy);
        const std::string target = dir + localNameFor(remoteName);
        std::error_code ec;
        if (fs::exists(target, ec)) return target;

        const std::string bare = Str::stripSuffixIC(remoteName, ".gz");
        for (const std::string &name: {bare, remoteName}) {
            const std::string compressed = dir + name;
            if (!fs::exists(compressed, ec)) continue;
            decompressIfNeeded(compressed);
            if (fs::exists(target, ec)) return target;
        }
        return std::string();
    }

    std::vector<LocalFile> listFiles(const ProductType t, const int year, const int doy) {
        std::vector<LocalFile> out;
        const std::string dir = dateDir(t, year, doy);

        std::error_code ec;
        std::vector<std::string> names;
        {
            const fs::directory_iterator it(dir, ec);
            if (ec) return out;
            for (const auto &e: it) names.push_back(e.path().filename().string());
        }

        if (t == ProductType::OBS) {
            bool changed = false;
            for (const auto &n: names)
                if (isCompactName(n) && decompressIfNeeded(dir + n) != dir + n) changed = true;
            if (changed) {
                names.clear();
                const fs::directory_iterator it(dir, ec);
                if (ec) return out;
                for (const auto &e: it) names.push_back(e.path().filename().string());
            }
        }

        for (const auto &name: names) {
            if (name.empty() || name[0] == '.') continue;
            if (Str::endsWithIC(name, ".part") || Str::endsWithIC(name, ".tmp")) continue;

            LocalFile lf;
            lf.name = name;
            lf.path = dir + name;
            std::replace(lf.path.begin(), lf.path.end(), '\\', '/');
            if (!fs::is_regular_file(lf.path, ec) || ec) continue;
            lf.size = static_cast<long long>(fs::file_size(lf.path, ec));
            if (ec) lf.size = 0;
            out.push_back(std::move(lf));
        }

        std::sort(out.begin(), out.end(),
                  [](const LocalFile &a, const LocalFile &b) { return a.name < b.name; });
        return out;
    }

    bool revealFile(const std::string &path) {
        std::error_code ec;
        const fs::path native = fs::canonical(path, ec);
        if (ec) return false;

        const std::wstring w = native.wstring();
        if (w.empty()) return false;

        const std::wstring args = L"/select,\"" + w + L"\"";
        const auto r = reinterpret_cast<INT_PTR>(
            ShellExecuteW(nullptr, L"open", L"explorer.exe", args.c_str(), nullptr, SW_SHOWNORMAL));
        return r > 32;
    }

    std::vector<BrowseEntry> browse() {
        std::vector<BrowseEntry> out;
        std::error_code ec;

        const ProductType dates[] = {
            ProductType::NAV, ProductType::ORB, ProductType::CLK, ProductType::ERP,
            ProductType::ION, ProductType::TRO, ProductType::OSB, ProductType::VMF,
            ProductType::ATT, ProductType::OBS
        };
        for (const ProductType t: dates) {
            const fs::path root = fs::path(downloadRoot()) / ProductMeta::typeDir(t);
            ec.clear();
            const fs::directory_iterator years(root, ec);
            if (ec) continue;
            for (const auto &y: years) {
                if (!y.is_directory(ec) || ec) continue;
                const std::string year = y.path().filename().string();
                for (const auto &d: fs::directory_iterator(y.path(), ec)) {
                    if (!d.is_directory(ec) || ec) continue;
                    const std::string doy = d.path().filename().string();
                    char stamp[32];
                    std::snprintf(stamp, sizeof stamp, "%s-%03d", year.c_str(), std::atoi(doy.c_str()));
                    for (const auto &f: fs::directory_iterator(d.path(), ec)) {
                        const std::string name = f.path().filename().string();
                        if (name.empty() || name[0] == '.' || Str::endsWithIC(name, ".part")) continue;
                        if (!f.is_regular_file(ec) || ec) continue;

                        BrowseEntry e;
                        e.path = f.path().generic_string();
                        e.name = name;
                        e.category = typeLabel(t);
                        e.stamp = stamp;
                        e.size = static_cast<long long>(f.file_size(ec));
                        if (ec) e.size = 0;
                        out.push_back(std::move(e));
                    }
                }
            }
        }

        ec.clear();
        // 注意：目录名不能用 "aux"（Windows 保留设备名 AUX/CON/NUL/PRN/COMx/LPTx），
        // 否则 Explorer 的命名空间解析不了该路径，/select 会落到桌面。用 auxdata。
        const fs::path aux = fs::path(downloadRoot()) / "auxdata";
        const fs::directory_iterator ids(aux, ec);
        if (ids != fs::directory_iterator()) {
            for (const auto &id: ids) {
                if (!id.is_directory(ec) || ec) continue;
                const std::string pid = id.path().filename().string();
                const auto inst = active(pid);
                for (const auto &f: fs::directory_iterator(id.path(), ec)) {
                    const std::string name = f.path().filename().string();
                    if (name.empty() || name[0] == '.' || Str::endsWithIC(name, ".part")) continue;
                    if (!f.is_regular_file(ec) || ec) continue;

                    BrowseEntry e;
                    e.path = f.path().generic_string();
                    e.name = name;
                    e.category = pid;
                    e.stamp = inst ? inst->version : std::string();
                    e.aux = true;
                    e.size = static_cast<long long>(f.file_size(ec));
                    if (ec) e.size = 0;
                    out.push_back(std::move(e));
                }
            }
        }

        std::sort(out.begin(), out.end(),
                  [](const BrowseEntry &a, const BrowseEntry &b) { return a.path < b.path; });
        return out;
    }

    std::vector<Entry> readManifest() {
        const std::lock_guard lk(manifestMutex());
        return readManifestLocked();
    }

    void writeManifest(const std::vector<Entry> &all) {
        const std::lock_guard lk(manifestMutex());
        writeManifestLocked(all);
    }

    void record(const Entry &e) {
        const std::lock_guard lk(manifestMutex());
        std::error_code ec;

        for (const auto &x: readManifestLocked())
            if (x.path == e.path) return;

        fs::create_directories(downloadRoot(), ec);
        std::ofstream f(manifestPath(), std::ios::app);
        if (!f) return;
        writeEntry(f, e);
    }

    namespace {
        /// 从文件往上删空目录，停在 downloads 根。
        void pruneEmptyParents(const fs::path &file) {
            const fs::path root = fs::path(downloadRoot());
            fs::path dir = file.parent_path();
            while (dir.string().size() > root.string().size()) {
                std::error_code ec;
                if (!fs::is_empty(dir, ec) || ec) return;
                if (!fs::remove(dir, ec) || ec) return;
                dir = dir.parent_path();
            }
        }
    }

    Removal removeFiles(const std::vector<std::string> &paths) {
        Removal res;
        std::vector<std::string> gone;

        for (const auto &p: paths) {
            std::error_code ecSize;
            const auto sz = static_cast<long long>(fs::file_size(p, ecSize));
            std::error_code ec;
            fs::remove(p, ec);

            std::error_code ecChk;
            if (fs::exists(p, ecChk)) {
                ++res.failed;
                LOG_WARN << "[下载] 彻底删除失败(文件被占用?): " << p;
                continue;
            }
            ++res.removed;
            if (!ecSize) res.bytes += sz;
            gone.push_back(p);
            pruneEmptyParents(fs::path(p));
        }

        if (!gone.empty()) {
            const std::lock_guard lk(manifestMutex());
            auto all = readManifestLocked();
            const auto before = all.size();
            all.erase(std::remove_if(all.begin(), all.end(),
                                     [&gone](const Entry &e) {
                                         return std::find(gone.begin(), gone.end(), e.path) != gone.end();
                                     }),
                      all.end());
            if (all.size() != before) writeManifestLocked(all);
        }

        LOG_INFO << "[下载] 彻底删除 " << res.removed << " 个本地文件，失败 " << res.failed
                << " 个，释放 " << res.bytes << " 字节";
        return res;
    }

    std::string decompressIfNeeded(const std::string &localPath) {
        const std::lock_guard lk(decodeMutex());
        std::string path = localPath;
        if (Str::endsWithIC(path, ".gz")) {
            gzFile in = gzopen(path.c_str(), "rb"); //NOLINT
            if (!in) return localPath;

            const std::string out = Str::stripSuffixIC(path, ".gz");
            std::ofstream fo(out, std::ios::binary);
            if (!fo) {
                gzclose(in);
                return localPath;
            }

            char buf[65536];
            int n = 0;
            bool ok = true;
            while ((n = gzread(in, buf, sizeof(buf))) > 0) {
                fo.write(buf, n);
                if (!fo) {
                    ok = false;
                    break;
                }
            }
            if (n < 0) ok = false;
            int errNum = Z_OK;
            gzerror(in, &errNum);
            if (errNum != Z_OK && errNum != Z_STREAM_END) ok = false;
            fo.close();
            if (fo.fail()) ok = false;
            gzclose(in);

            std::error_code ec;
            if (!ok) {
                fs::remove(out, ec);
                LOG_WARN << "[下载] 解压失败，保留压缩包: " << localPath;
                return localPath;
            }
            std::error_code ecSize;
            LOG_INFO << "[下载] 解压 " << out << " (" << fs::file_size(out, ecSize) << " 字节)";
            fs::remove(path, ec);
            path = out;
        }

        if (!CRXDecoder::isCompactRINEX(path)) return path;
        const std::string rnx = rinexNameFor(path);
        std::string err;
        CRXDecoder decoder;
        if (!decoder.decode(path, rnx, err)) {
            LOG_WARN << "[下载] Hatanaka 还原失败，保留压缩文件: " << err;
            return path;
        }
        std::error_code ec;
        fs::remove(path, ec);
        LOG_INFO << "[下载] 已还原 RINEX 观测文件: " << rnx;
        return rnx;
    }

    std::string productRoot() { return downloadRoot() + "/auxdata/"; }

    std::string productDir(const std::string &id) { return productRoot() + id + "/"; }

    std::optional<Installed> active(const std::string &id) {
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

    std::optional<Installed> active(const VersionedProduct &product) {
        return active(product.id);
    }

    std::string activeFile(const VersionedProduct &product) {
        if (const auto p = active(product)) return p->path;
        return {};
    }

    std::string tempPath(const std::string &id, const std::string &name) {
        return productDir(id) + ".tmp_" + name;
    }

    bool ensureDir(const std::string &id) {
        std::error_code ec;
        fs::create_directories(productDir(id), ec);
        if (ec) {
            LOG_ERROR << "[产品] 建目录失败 " << productDir(id) << ": " << ec.message();
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
        for (const auto &ent: fs::directory_iterator(productDir(id), ec)) {
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
        const std::string dstDir = productDir(id);
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

        Installed p;
        p.id = id;
        p.version = version;
        p.file = fileName;
        p.path = dst;
        const auto sz = fs::file_size(dst, ec);
        p.size = ec ? 0 : static_cast<long long>(sz);
        p.fetchedAt = nowStamp();

        {
            std::lock_guard lk(manifestMutex());
            auto all = readIndex();
            all[id] = p;
            writeIndex(all);
        }

        dropStale(id, dst);

        LOG_INFO << "[产品] 已安装 " << id << " 版本 " << version << " → " << dst
                << " (" << p.size << " 字节)";
        return true;
    }
}
