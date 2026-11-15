#include "download/remote/http/Core.h"
#include "download/remote/http/Internal.h"
#include "i18n/I18n.h"

#include "StringUtils.h"

#include <map>
#include <regex>
#include <string>
#include <vector>

namespace {
    std::size_t parseSfmList(const std::string &json, const std::string &listUrl,
                             std::vector<std::string> &out,
                             std::map<std::string, std::string> *fullUrls) {
        if (json.find(R"("success":true)") == std::string::npos) return SIZE_MAX;
        const std::size_t arr = json.find(R"("results")");
        if (arr == std::string::npos) return SIZE_MAX;
        const std::size_t q = listUrl.find('?');
        const std::string base = listUrl.substr(0, q == std::string::npos ? listUrl.size() : q);

        const auto encodePath = [](const std::string &p) {
            static auto hex = "0123456789ABCDEF";
            std::string t;
            for (const unsigned char c: p) {
                if (std::isalnum(c) != 0 || c == '.' || c == '_' || c == '-') {
                    t += static_cast<char>(c);
                } else {
                    t += '%';
                    t += hex[c >> 4];
                    t += hex[c & 0xF];
                }
            }
            return t;
        };

        std::size_t count = 0;
        std::size_t at = arr;
        for (;;) {
            const std::size_t nb = json.find(R"("name":")", at);
            if (nb == std::string::npos) break;
            const std::size_t nve = json.find('"', nb + 8);
            if (nve == std::string::npos) return SIZE_MAX;
            const std::string name = json.substr(nb + 8, nve - nb - 8);
            at = nve;
            const std::size_t nextName = json.find(R"("name":")", nve);
            const std::size_t pb = json.find(R"("path":")", nve);
            if (pb == std::string::npos || (nextName != std::string::npos && pb > nextName))
                continue;
            const std::size_t pve = json.find('"', pb + 8);
            if (pve == std::string::npos) return SIZE_MAX;
            const std::size_t nextName2 = json.find(R"("name":")", pve);
            const std::size_t idb = json.find(R"("is_dir":true)", pve);
            const bool isDir = idb != std::string::npos &&
                               (nextName2 == std::string::npos || idb < nextName2);
            at = pve;
            if (name.empty() || isDir) continue;
            std::string path;
            for (std::size_t i = pb + 8; i < pve; ++i) {
                if (json[i] == '\\' && i + 1 < pve && json[i + 1] == '/') {
                    path += '/';
                    ++i;
                } else {
                    path += json[i];
                }
            }
            out.push_back(name);
            ++count;
            if (fullUrls) (*fullUrls)[name] = base + "?do=download&file=" + encodePath(path);
        }
        return count;
    }

    void parseList(const std::string &listing, std::vector<std::string> &out,
                   std::map<std::string, std::string> *fullUrls) {
        if (listing.find("href=") != std::string::npos) {
            std::regex re(R"(href\s*=\s*["']([^"']+)["'])", std::regex::icase);
            for (std::sregex_iterator it(listing.begin(), listing.end(), re), end; it != end; ++it) {
                std::string h = Str::strip((*it)[1].str());
                if (h.empty() || h.back() == '/') continue; // 子目录链接，不是文件
                const bool absolute = h.rfind("http://", 0) == 0 || h.rfind("https://", 0) == 0;
                if (const std::size_t sl = h.find_last_of('/'); sl != std::string::npos) h = h.substr(sl + 1);
                if (h.empty() || h == "." || h == ".." || h == "index.html" ||
                    h.find('*') != std::string::npos)
                    continue;
                if (absolute && fullUrls) (*fullUrls)[h] = Str::strip((*it)[1].str());
                out.push_back(h);
            }
            return;
        }

        for (const std::string &raw: Str::split(listing, '\n')) {
            const std::string line = Str::strip(raw); // 顺带吃掉 \r 与行尾空白
            if (line.empty()) continue;
            const std::size_t sp = line.find_last_of(" \t");
            const std::string name = sp != std::string::npos ? line.substr(sp + 1) : line;
            if (name.empty() || name == "." || name == ".." ||
                name.find('*') != std::string::npos)
                continue;
            out.push_back(name);
        }
    }

    bool listSfm(const std::string &url, std::vector<std::string> &outNames, std::string &err,
                 const std::atomic<bool> *cancel, std::map<std::string, std::string> *fullUrls) {
        const auto numAfter = [](const std::string &tpl, const char *key, const std::size_t def) {
            const std::size_t at = tpl.find(key);
            if (at == std::string::npos) return def;
            const std::size_t b = at + std::strlen(key);
            std::size_t e = b;
            while (e < tpl.size() && tpl[e] >= '0' && tpl[e] <= '9') ++e;
            return e > b ? std::stoull(tpl.substr(b, e - b)) : def;
        };
        const auto withPos = [](const std::string &tpl, const std::size_t pos) {
            const std::size_t at = tpl.find("pos=");
            if (at == std::string::npos) return tpl;
            std::size_t e = at + 4;
            while (e < tpl.size() && tpl[e] >= '0' && tpl[e] <= '9') ++e;
            return tpl.substr(0, at + 4) + std::to_string(pos) + tpl.substr(e);
        };
        const std::size_t pageSize = numAfter(url, "maxfiles=", 500);
        for (std::size_t pos = 0, page = 0; page < 40; pos += pageSize, ++page) {
            std::size_t n = 0;
            bool ok = false;
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (cancel && cancel->load()) break;
                const Download::Http::Response r =
                        Download::Http::request(withPos(url, pos), Download::Http::kListFetch, cancel);
                if (r.err.empty() && r.http < 400) {
                    n = parseSfmList(r.text, url, outNames, fullUrls);
                    if (n != SIZE_MAX) {
                        ok = true;
                        err.clear();
                        break;
                    }
                    err = I18n::tr("dl.http.swarm_json_failed");
                    return !outNames.empty();
                }
                err = !r.err.empty() ? r.err : "HTTP " + std::to_string(r.http);
            }
            if (cancel && cancel->load()) break;
            if (!ok) return !outNames.empty(); // 重试耗尽，已翻到的页保留
            if (n < pageSize) break; // 未满一页 = 到底了
        }
        return true;
    }
}

namespace Download::Http {
    std::string nameOfUrl(const std::string &url) {
        std::string path = url;
        if (const std::size_t q = url.find('?'); q != std::string::npos) {
            for (const auto &kv: Str::split(url.substr(q + 1), '&', false))
                if (kv.rfind("file=", 0) == 0) path = kv.substr(5);
            if (path == url) path = url.substr(0, q);
        }
        std::string last;
        for (std::size_t i = 0; i < path.size();) {
            const bool sep = path[i] == '/' ||
                             (path[i] == '%' && i + 2 < path.size() && path[i + 1] == '2' &&
                              (path[i + 2] == 'F' || path[i + 2] == 'f'));
            if (sep) {
                last.clear();
                i += path[i] == '/' ? 1 : 3;
                continue;
            }
            last += path[i++];
        }
        std::string name;
        name.reserve(last.size());
        for (std::size_t i = 0; i < last.size(); ++i) {
            char c = last[i];
            if (c == '%' && i + 2 < last.size() && last[i + 1] == '2' && last[i + 2] == '0') {
                name += ' ';
                i += 2;
                continue;
            }
            if (c == '%' || c == '?' || c == '*' || c == ':' || c == '"' || c == '<' || c == '>' || c == '|')
                c = '_';
            name += c;
        }
        return name;
    }

    bool listDir(const std::string &url, std::vector<std::string> &outNames, std::string &err,
                 const std::atomic<bool> *cancel,
                 std::map<std::string, std::string> *fullUrls) {
        if (url.find("do=list") != std::string::npos)
            return listSfm(url, outNames, err, cancel, fullUrls);

        std::string dir = url;
        if (!dir.empty() && dir.back() != '/') dir.push_back('/');

        const Response r = request(dir, kListFetch, cancel);
        if (!r.err.empty()) {
            err = r.err;
            return false;
        }
        if (r.http >= 400) {
            err = "HTTP " + std::to_string(r.http);
            return false;
        }
        try {
            parseList(r.text, outNames, fullUrls);
        } catch (const std::exception &e) {
            err = I18n::tr("dl.http.parse_list_failed", e.what());
            return false;
        } catch (...) {
            err = I18n::tr("dl.http.parse_list_failed_unknown");
            return false;
        }
        return true;
    }
}
