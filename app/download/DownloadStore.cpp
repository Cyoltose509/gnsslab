#define _CRT_SECURE_NO_WARNINGS   // fopen/gzopen 走 C 运行库传统接口（与 src/io/ClkReader.cpp 同）

#include "DownloadStore.h"
#include "ProductMeta.h"

#include "Log.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <algorithm>

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

std::vector<std::string> DownloadStore::findLocal(const ProductType t, const int year, const int doy) {
    std::vector<std::string> out;
    std::error_code ec;
    const auto dir = dirFor(t, year, doy);
    if (!fs::exists(dir, ec)) return out;
    for (const auto &ent: fs::directory_iterator(dir, ec)) {
        if (ent.is_regular_file(ec))
            out.push_back(ent.path().filename().string());
    }
    return out;
}

std::string DownloadStore::localNameFor(const std::string &remoteName) {
    std::string l = remoteName;
    for (auto &c: l) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (l.size() > 3 && l.compare(l.size() - 3, 3, ".gz") == 0)
        return remoteName.substr(0, remoteName.size() - 3);
    return remoteName;
}

std::string DownloadStore::findLocalFile(const ProductType t, const int year, const int doy, const std::string &remoteName) {
    const std::string p = dirFor(t, year, doy) + localNameFor(remoteName);
    std::error_code ec;
    return fs::exists(p, ec) ? p : std::string();
}

bool DownloadStore::removeFile(const std::string &path) {
    std::error_code ec;
    fs::remove(path, ec);

    // manifest 去掉该条（删除是低频操作，整表重写一次即可）
    auto all = readManifest();
    const auto before = all.size();
    all.erase(std::remove_if(all.begin(), all.end(),
                             [&path](const DownloadEntry &e) { return e.path == path; }),
              all.end());
    if (all.size() != before) writeManifest(all);

    std::error_code ec2;
    const bool gone = !fs::exists(path, ec2);
    LOG_INFO << (gone ? "[下载] 已删除本地文件: " : "[下载] 删除失败(文件被占用?): ") << path;
    return gone;
}

// ---- manifest：极简 JSONL（每行一个 {..."key":"val",...}）----
static std::string manifestPath() {
    return DownloadStore::root() + "/manifest.json";
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
    // 在行内查找 "key":"value"；value 不含引号/反斜杠（文件名安全）
    const std::string pat = "\"" + key + "\":\"";
    auto p = line.find(pat);
    if (p == std::string::npos) return "";
    p += pat.size();
    const auto q = line.find('"', p);
    if (q == std::string::npos) return "";
    return line.substr(p, q - p);
}

static long long extractNum(const std::string &line, const std::string &key) {
    const std::string pat = "\"" + key + "\":";
    auto p = line.find(pat);
    if (p == std::string::npos) return 0;
    p += pat.size();
    const char *b = line.c_str() + p;
    return std::strtoll(b, nullptr, 10);
}

std::vector<DownloadEntry> DownloadStore::readManifest() {
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

void DownloadStore::writeManifest(const std::vector<DownloadEntry> &all) {
    std::error_code ec;
    fs::create_directories(root(), ec);
    std::ofstream f(manifestPath(), std::ios::trunc);
    if (!f) return;
    for (const auto &[type, year, doy, datacenter, file, path, size]: all) {
        f << "{"
                << R"("type":")" << escapeJson(type) << "\","
                << "\"year\":" << year << ","
                << "\"doy\":" << doy << ","
                << R"("datacenter":")" << escapeJson(datacenter) << "\","
                << R"("file":")" << escapeJson(file) << "\","
                << R"("path":")" << escapeJson(path) << "\","
                << "\"size\":" << size
                << "}\n";
    }
}

void DownloadStore::record(const DownloadEntry &e) {
    for (const auto &x: readManifest())
        if (x.path == e.path) return; // 同路径不重复记录

    std::error_code ec;
    fs::create_directories(root(), ec);
    std::ofstream f(manifestPath(), std::ios::app);
    if (!f) return;
    f << "{"
            << R"("type":")" << escapeJson(e.type) << "\","
            << "\"year\":" << e.year << ","
            << "\"doy\":" << e.doy << ","
            << R"("datacenter":")" << escapeJson(e.datacenter) << "\","
            << R"("file":")" << escapeJson(e.file) << "\","
            << R"("path":")" << escapeJson(e.path) << "\","
            << "\"size\":" << e.size
            << "}\n";
}

std::string DownloadStore::decompressIfNeeded(const std::string &localPath) {
    std::string l = localPath;
    for (auto &c: l) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (l.size() < 3 || l.compare(l.size() - 3, 3, ".gz") != 0) return localPath;

    gzFile in = gzopen(localPath.c_str(), "rb"); //NOLINT
    if (!in) return localPath;

    std::string out = localPath.substr(0, localPath.size() - 3);
    FILE *fo = std::fopen(out.c_str(), "wb");
    if (!fo) {
        gzclose(in);
        return localPath;
    }

    char buf[65536];
    int n;
    bool ok = true;
    while ((n = gzread(in, buf, sizeof(buf))) > 0) {
        if (std::fwrite(buf, 1, static_cast<size_t>(n), fo) != static_cast<size_t>(n)) {
            ok = false;
            break;
        }
    }
    if (n < 0) ok = false;
    std::fclose(fo);
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
