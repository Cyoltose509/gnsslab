#include "ProductCatalog.h"

#include "RemoteClient.h"
#include "Log.h"
#include "ProductStore.h"
#include "DownloadStore.h"
#include "StringUtils.h"
#include "product/ATXReader.h"
#include "product/GPT3Reader.h"

#include <cassert>
#include <filesystem>
#include <regex>
#include <sstream>
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
                "*.atx;*.ATX;*.atx.gz",
                ATXReader::validate,
            },
            {
                "gpt3",
                "对流层格网 (GPT3)",
                "https://vmf.geo.tuwien.ac.at/codes/",
                "",
                "gpt3_5.grd",
                24,
                "*.grd;*.GRD",
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

        if (std::string body; p.pattern && *p.pattern && RemoteClient::fetchText(p.indexUrl, body, err)) {
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
            const std::string got = DownloadStore::decompressIfNeeded(work);
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

        if (!Store::install(p.id, work, version, fileName,staged)) {
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
}
