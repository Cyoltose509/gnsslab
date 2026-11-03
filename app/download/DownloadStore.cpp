#include "DownloadStore.h"
#include "ProductMeta.h"

#include "Log.h"
#include "StringUtils.h"

#include <windows.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <algorithm>
#include <mutex>

#include <zlib.h>

namespace fs = std::filesystem;

std::string DownloadStore::root() {
    wchar_t buf[MAX_PATH] = {};
    GetModuleFileNameW(nullptr, buf, MAX_PATH);
    fs::path p(buf);
    p = p.parent_path() / "downloads";
    std::string s = p.string();
    std::replace(s.begin(), s.end(), '\\', '/');
    return s;
}

std::string DownloadStore::dirFor(const ProductType t, const int year, const int doy) {
    return root() + "/" + ProductMeta::typeDir(t) + "/" +
           std::to_string(year) + "/" + std::to_string(doy) + "/";
}

std::string DownloadStore::localPath(const ProductType t, const int year, const int doy, const std::string &filename) {
    return dirFor(t, year, doy) + filename;
}

std::string DownloadStore::localNameFor(const std::string &remoteName) {
    return Str::stripSuffixIC(remoteName, ".gz");
}

std::string DownloadStore::findLocalFile(const ProductType t, const int year, const int doy, const std::string &remoteName) {
    const std::string p = dirFor(t, year, doy) + localNameFor(remoteName);
    std::error_code ec;
    return fs::exists(p, ec) ? p : std::string();
}


static std::string manifestPath() {
    return DownloadStore::root() + "/manifest.jsonl";
}

static std::mutex &manifestMutex() {
    static std::mutex m;
    return m;
}

static std::string escapeJson(const std::string &s) {
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

static std::string extractField(const std::string &line, const std::string &key) {
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

static long long extractNum(const std::string &line, const std::string &key) {
    const std::string pat = "\"" + key + "\":";
    auto p = line.find(pat);
    if (p == std::string::npos) return 0;
    p += pat.size();
    const char *b = line.c_str() + p;
    return std::strtoll(b, nullptr, 10);
}

static void writeEntry(std::ostream &os, const DownloadEntry &e) {
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

static std::vector<DownloadEntry> readManifestLocked() {
    std::vector<DownloadEntry> out;
    std::ifstream f(manifestPath());
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.find("\"type\"") == std::string::npos) continue;
        DownloadEntry e;
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

std::vector<DownloadEntry> DownloadStore::readManifest() {
    const std::lock_guard lk(manifestMutex());
    return readManifestLocked();
}

static void writeManifestLocked(const std::vector<DownloadEntry> &all) {
    std::error_code ec;
    fs::create_directories(DownloadStore::root(), ec);
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

void DownloadStore::writeManifest(const std::vector<DownloadEntry> &all) {
    const std::lock_guard lk(manifestMutex());
    writeManifestLocked(all);
}

void DownloadStore::record(const DownloadEntry &e) {
    const std::lock_guard lk(manifestMutex());
    std::error_code ec;

    for (const auto &x: readManifestLocked())
        if (x.path == e.path) return;

    fs::create_directories(root(), ec);
    std::ofstream f(manifestPath(), std::ios::app);
    if (!f) return;
    writeEntry(f, e);
}

bool DownloadStore::removeFile(const std::string &path) {
    std::error_code ec;
    fs::remove(path, ec);

    {
        const std::lock_guard lk(manifestMutex());
        auto all = readManifestLocked();
        const auto before = all.size();
        all.erase(std::remove_if(all.begin(), all.end(),
                                 [&path](const DownloadEntry &e) { return e.path == path; }),
                  all.end());
        if (all.size() != before) writeManifestLocked(all);
    }

    std::error_code ec2;
    const bool gone = !fs::exists(path, ec2);
    LOG_INFO << (gone ? "[下载] 已删除本地文件: " : "[下载] 删除失败(文件被占用?): ") << path;
    return gone;
}

std::string DownloadStore::decompressIfNeeded(const std::string &localPath) {
    if (!Str::endsWithIC(localPath, ".gz")) return localPath;

    gzFile in = gzopen(localPath.c_str(), "rb"); //NOLINT
    if (!in) return localPath;

    std::string out = Str::stripSuffixIC(localPath, ".gz");
    std::ofstream fo(out, std::ios::binary);
    if (!fo) {
        gzclose(in);
        return localPath;
    }

    char buf[65536];
    int n;
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
    fs::remove(localPath, ec);
    return out;
}
