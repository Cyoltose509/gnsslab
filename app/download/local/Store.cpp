#include "download/local/Store.h"

#include "CSV.h"
#include "Log.h"
#include "StringUtils.h"
#include "download/local/Unpack.h"
#include "util/Clock.h"

#include <windows.h>
#include <shellapi.h>

#include <algorithm>
#include <charconv>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <vector>

namespace fs = std::filesystem;

namespace Download::Store {
    namespace {
        std::mutex s_indexMutex;

        /// 日期目录的条目名单，按目录 mtime 缓存（列表每帧每行都要问一次「下过没有」）
        std::mutex s_dirNameMutex;
        std::map<std::string, std::pair<std::filesystem::file_time_type, std::set<std::string> > > s_dirNames;

        bool isCompactName(const std::string &name) {
            const std::string bare = Unpack::plainName(name);
            return Str::endsWithIC(bare, ".crx") || Unpack::isShortCompactName(bare);
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
            const std::tm tm = Clock::localNow();
            char buf[32];
            std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d",
                          tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                          tm.tm_hour, tm.tm_min, tm.tm_sec);
            return buf;
        }

        std::map<std::string, Installed> parseIndex() {
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
                long long bytes = 0;
                const char *const first = cols[4].data();
                const char *const last = first + cols[4].size();
                if (auto [ptr, ec] = std::from_chars(first, last, bytes); ec == std::errc{}) p.size = bytes;
                p.fetchedAt = cols[5];
                if (!p.id.empty()) out[p.id] = p;
            }
            return out;
        }

        /// index.csv 按 mtime 缓存：界面每帧都要问「现在装的是哪版」，
        /// 每帧重开文件解析 CSV 纯属浪费；mtime 变了（自己写过或手工替换过）就重读。
        std::mutex s_indexCacheMutex;
        std::map<std::string, Installed> s_indexCache;
        std::filesystem::file_time_type s_indexStamp{};
        bool s_indexFresh = false;

        /// 返回副本：缓存表可能被另一个线程（自动更新）换掉，返回引用会让调用方读到半张表。
        std::map<std::string, Installed> cachedIndex() {
            const std::string path = indexPath();
            std::error_code ec;
            const auto mt = std::filesystem::last_write_time(path, ec);
            std::lock_guard lk(s_indexCacheMutex);
            if (s_indexFresh && !ec && s_indexStamp == mt) return s_indexCache;
            s_indexCache = parseIndex();
            s_indexStamp = mt;
            s_indexFresh = !ec;
            return s_indexCache;
        }

        void writeIndex(const std::map<std::string, Installed> &all) {
            CSV::Writer w;
            w.row("id", "version", "file", "path", "size", "fetched_at");
            for (const auto &[_, p]: all)
                w.row(p.id, p.version, p.file, p.path, std::to_string(p.size), p.fetchedAt);
            writeAtomic(indexPath(), w.str());
            std::lock_guard lk(s_indexCacheMutex);
            s_indexFresh = false;
        }

        /// 从文件往上删空目录，停在 downloads 根。
        void pruneEmptyParents(const fs::path &file) {
            const auto root = fs::path(downloadRoot());
            fs::path dir = file.parent_path();
            while (dir.string().size() > root.string().size()) {
                std::error_code ec;
                if (!fs::is_empty(dir, ec) || ec) return;
                if (!fs::remove(dir, ec) || ec) return;
                dir = dir.parent_path();
            }
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

    void writeAtomic(const std::string &path, const std::string &content) {
        std::error_code ec;
        fs::create_directories(fs::path(path).parent_path(), ec);
        if (ec) {
            LOG_WARN << "[下载] 建索引目录失败 " << path << ": " << ec.message();
            return;
        }
        const std::string tmp = path + ".tmp";
        {
            std::ofstream f(tmp, std::ios::trunc | std::ios::binary);
            if (!f) return;
            f << content;
            if (!f) return;
        }
        fs::rename(tmp, path, ec);
        if (ec) LOG_WARN << "[下载] 写索引失败 " << path << ": " << ec.message();
    }

    std::string dateDir(const ProductType t, const int year, const int doy) {
        return downloadRoot() + "/" + ProductMeta::typeDir(t) + "/" +
               std::to_string(year) + "/" + std::to_string(doy) + "/";
    }

    std::string datePath(const ProductType t, const int year, const int doy, const std::string &filename) {
        return dateDir(t, year, doy) + filename;
    }

    std::string localNameFor(const std::string &remoteName) {
        const std::string bare = Unpack::plainName(remoteName);
        if (isCompactName(remoteName)) return Unpack::rinexName(bare);
        // 归档落成展开目录，显示的应当是那个目录的名字，不是 tar/zip 本身。
        return Unpack::stemName(bare);
    }

    long long sizeOnDisk(const std::string &path) {
        std::error_code ec;
        if (!fs::is_directory(path, ec)) {
            const auto size = fs::file_size(path, ec);
            return ec ? 0 : static_cast<long long>(size);
        }
        long long total = 0;
        for (const auto &ent: fs::recursive_directory_iterator(path, ec)) {
            std::error_code fec;
            if (!ent.is_regular_file(fec) || fec) continue;
            const auto size = ent.file_size(fec);
            if (!fec) total += static_cast<long long>(size);
        }
        return total;
    }

    std::string findLocal(const ProductType t, const int year, const int doy, const std::string &remoteName) {
        const std::string dir = dateDir(t, year, doy);
        const std::string target = dir + localNameFor(remoteName);
        std::error_code ec;
        // 列表每帧问一遍每行「下过没有」，目录没变就不要每次 stat。
        if (const auto mt = fs::last_write_time(dir, ec); !ec) {
            std::lock_guard lk(s_dirNameMutex);
            auto &ent = s_dirNames[dir];
            if (ent.first != mt) {
                ent.first = mt;
                ent.second.clear();
                std::error_code de;
                for (const auto &f: fs::directory_iterator(dir, de)) {
                    if (de) break;
                    ent.second.insert(f.path().filename().string());
                }
            }
            if (ent.second.count(localNameFor(remoteName)) > 0) return target;
            return {};
        }
        if (fs::exists(target, ec)) return target;
        // 未展开的压缩包一律算「还没有可用文件」：解压由后台队列做，界面线程不能在
        // 这里 settle（一个 podCrx 归档有几百个成员，逐个解明文会把 UI 卡死）。
        return {};
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

        // 这里只列目录，不做任何解压/还原：Hatanaka 与归档展开都由后台队列处理，
        // 界面线程逐文件 settle 会让大产品（数百个分片）整个卡住。
        for (const auto &name: names) {
            if (name.empty() || name[0] == '.') continue;
            if (Str::endsWithIC(name, ".part") || Str::endsWithIC(name, ".tmp")) continue;

            LocalFile lf;
            lf.name = name;
            lf.path = dir + name;
            std::replace(lf.path.begin(), lf.path.end(), '\\', '/');
            if (fs::is_directory(lf.path, ec)) {
                // 归档的落点就是这个目录（远端那一条的本地名字也取目录名），按一条本地产物算。
                lf.size = sizeOnDisk(lf.path);
                out.push_back(std::move(lf));
                continue;
            }
            if (!fs::is_regular_file(lf.path, ec) || ec) continue;
            lf.size = static_cast<long long>(fs::file_size(lf.path, ec));
            if (ec) lf.size = 0;
            out.push_back(std::move(lf));
        }

        std::sort(out.begin(), out.end(),
                  [](const LocalFile &a, const LocalFile &b) { return a.name < b.name; });
        return out;
    }

    std::vector<std::string> truthFiles(const int year, const int doy) {
        std::vector<std::string> out;
        std::error_code ec;
        const std::string dir = dateDir(ProductType::LEO_SP3, year, doy);
        for (const auto &ent: fs::recursive_directory_iterator(dir, fs::directory_options::skip_permission_denied, ec)) {
            if (ec) break;
            if (std::error_code fec; !ent.is_regular_file(fec) || fec) continue;
            if (const std::string name = ent.path().filename().string(); name.rfind(".tmp_", 0) == 0 || !Str::endsWithIC(name, ".sp3")) continue;
            out.push_back(ent.path().generic_string());
        }
        std::sort(out.begin(), out.end());
        return out;
    }

    int reclaimLegacyArchives() {
        int removed = 0;
        std::error_code ec;
        for (int i = 0; i < kStoredTypeCount; ++i) {
            const auto t = static_cast<ProductType>(i);
            const fs::path root = fs::path(downloadRoot()) / ProductMeta::typeDir(t);
            for (const auto &y: fs::directory_iterator(root, ec)) {
                if (!y.is_directory(ec) || ec) continue;
                for (const auto &d: fs::directory_iterator(y.path(), ec)) {
                    if (!d.is_directory(ec) || ec) continue;
                    for (const auto &f: fs::directory_iterator(d.path(), ec)) {
                        std::error_code fec;
                        if (!f.is_regular_file(fec) || fec) continue;
                        if (!Unpack::isArchive(f.path().string())) continue;

                        const std::string name = f.path().filename().string();
                        const fs::path expanded = d.path() / Unpack::stemName(name);
                        if (!fs::is_directory(expanded, fec) || fec) continue;
                        if (fs::directory_iterator(expanded, fec) == fs::directory_iterator() || fec) continue;

                        fs::remove(f.path(), fec);
                        if (fec) continue;
                        ++removed;
                        LOG_INFO << "[下载] 清理旧归档 " << f.path().generic_string() << "（内容已在 "
                                << expanded.filename().string() << "/ 里）";
                    }
                }
            }
        }
        if (removed > 0) LOG_INFO << "[下载] 旧归档共清理 " << removed << " 个";
        return removed;
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

        const auto push = [&out](const fs::path &path, const std::string &name,
                                 const std::string &category, const std::string &stamp,
                                 const bool aux, const long long size) {
            BrowseEntry e;
            e.path = path.generic_string();
            e.name = name;
            e.category = category;
            e.stamp = stamp;
            e.aux = aux;
            e.size = size;
            out.push_back(std::move(e));
        };

        for (int i = 0; i < kStoredTypeCount; ++i) {
            const auto t = static_cast<ProductType>(i);
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
                    int doyNum = 0;
                    std::from_chars(doy.data(), doy.data() + doy.size(), doyNum);
                    char stamp[32];
                    std::snprintf(stamp, sizeof stamp, "%s-%03d", year.c_str(), doyNum);
                    for (const auto &f: fs::directory_iterator(d.path(), ec)) {
                        const std::string name = f.path().filename().string();
                        if (name.empty() || name[0] == '.' || Str::endsWithIC(name, ".part")) continue;

                        // 归档展开在同名子目录里，成员才是真正的产物，逐层钻到这一层列文件。
                        if (f.is_directory(ec) && !ec) {
                            for (const auto &m: fs::directory_iterator(f.path(), ec)) {
                                std::error_code fec;
                                if (!m.is_regular_file(fec) || fec) continue;
                                const auto size = m.file_size(fec);
                                push(m.path(), m.path().filename().string(), typeLabel(t), stamp, false,
                                     fec ? 0 : static_cast<long long>(size));
                            }
                            continue;
                        }
                        if (!f.is_regular_file(ec) || ec) continue;
                        // 老布局的残留包：内容已经在同名目录里，别再把它当文件列一遍。
                        std::error_code dirEc;
                        if (fs::is_directory(d.path() / Unpack::stemName(name), dirEc)) continue;

                        const auto size = f.file_size(ec);
                        push(f.path(), name, typeLabel(t), stamp, false,
                             ec ? 0 : static_cast<long long>(size));
                    }
                }
            }
        }

        ec.clear();
        const fs::path aux = fs::path(downloadRoot()) / "auxdata";
        if (const fs::directory_iterator ids(aux, ec); ids != fs::directory_iterator()) {
            for (const auto &id: ids) {
                if (!id.is_directory(ec) || ec) continue;
                const std::string pid = id.path().filename().string();
                const auto inst = active(pid);
                for (const auto &f: fs::directory_iterator(id.path(), ec)) {
                    const std::string name = f.path().filename().string();
                    if (name.empty() || name[0] == '.' || Str::endsWithIC(name, ".part")) continue;
                    if (!f.is_regular_file(ec) || ec) continue;

                    push(f.path(), name, pid, inst ? inst->version : std::string(), true,
                         static_cast<long long>(f.file_size(ec)));
                }
            }
        }

        std::sort(out.begin(), out.end(),
                  [](const BrowseEntry &a, const BrowseEntry &b) { return a.path < b.path; });
        return out;
    }


    Removal removeFiles(const std::vector<std::string> &paths) {
        Removal res;

        for (const auto &p: paths) {
            const long long sz = sizeOnDisk(p); // 归档展开目录要递归累加，单文件就是文件大小
            std::error_code ec;
            if (fs::is_directory(p, ec)) fs::remove_all(p, ec);
            else fs::remove(p, ec);

            if (std::error_code ecChk; fs::exists(p, ecChk)) {
                ++res.failed;
                LOG_WARN << "[下载] 彻底删除失败(文件被占用?): " << p;
                continue;
            }
            ++res.removed;
            res.bytes += sz;
            pruneEmptyParents(fs::path(p));
        }

        LOG_INFO << "[下载] 彻底删除 " << res.removed << " 个本地文件，失败 " << res.failed
                << " 个，释放 " << res.bytes << " 字节";
        return res;
    }

    std::string productRoot() { return downloadRoot() + "/auxdata/"; }

    std::string productDir(const std::string &id) { return productRoot() + id + "/"; }

    std::optional<Installed> active(const std::string &id) {
        const auto all = cachedIndex();
        const auto it = all.find(id);
        if (it == all.end() || it->second.path.empty()) return std::nullopt;
        if (std::error_code ec; !fs::exists(it->second.path, ec)) return std::nullopt;
        return it->second;
    }

    std::string activeFile(const std::string &id) {
        if (const auto p = active(id)) return p->path;
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
            std::lock_guard lk(s_indexMutex);
            auto all = cachedIndex();
            all[id] = p;
            writeIndex(all);
        }

        dropStale(id, dst);

        LOG_INFO << "[产品] 已安装 " << id << " 版本 " << version << " → " << dst
                << " (" << p.size << " 字节)";
        return true;
    }
}
