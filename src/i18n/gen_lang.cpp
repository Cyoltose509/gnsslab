/// 构建期把语言目录下的 CSV 编成 LangData.cpp（constexpr 表 + 语言码）
/// 顺带做构建期校验
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
    constexpr std::size_t kMaxLanguages = 8;
    const auto kDefaultLang = "ZH-CN";
    const auto kSourceAlias = "ZH-CN_fake"; // 机翻脚本写的中文对照列

    struct Item {
        std::string key;
        std::vector<std::string> text;
        std::string origin; // 文件名:行号，报错用
    };

    struct Row {
        int line = 0;
        std::vector<std::string> cells;
    };

    struct MachineRow {
        std::string key;
        std::string source; // 机翻时的中文原文
        std::vector<std::pair<std::string, std::string> > text; // 语言码 + 译文
    };

    std::string nameOf(const std::string &code) {
        if (code == "ZH-CN") return "简体中文";
        if (code == "ZH-TW" || code == "ZH-HK") return "繁體中文";
        if (code == "EN" || code == "EN-US") return "English";
        if (code == "JA") return "日本語";
        if (code == "KO") return "한국어";
        return code;
    }

    int langRank(const std::string &code) {
        if (code == "ZH-CN") return 0;
        if (code == "EN") return 1;
        if (code == "ZH-TW") return 2;
        if (code == "JA") return 3;
        if (code == "KO") return 4;
        return 100;
    }

    std::string trim(std::string s) {
        const auto notSpace = [](const unsigned char c) { return !std::isspace(c); };
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
        s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
        return s;
    }

    std::vector<Row> parseCsv(const std::string &raw, std::vector<std::string> &errors, const std::string &file) {
        std::string text = raw;
        if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
            static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
            text.erase(0, 3);

        std::vector<Row> rows;
        Row row;
        std::string cell;
        bool quoted = false;
        bool rowDone = false;
        int line = 1;
        row.line = 1;

        for (std::size_t i = 0; i < text.size(); ++i) {
            const char c = text[i];
            if (quoted) {
                if (c == '"') {
                    if (i + 1 < text.size() && text[i + 1] == '"') {
                        cell.push_back('"');
                        ++i;
                    } else {
                        quoted = false;
                    }
                } else {
                    if (c == '\n') ++line;
                    cell.push_back(c);
                }
                continue;
            }
            if (c == '"') {
                quoted = true;
            } else if (c == ',') {
                row.cells.push_back(cell);
                cell.clear();
            } else if (c == '\r') {
                // 交给后面的 \n 收尾；单 \r 结尾的老式换行也当成行尾
                if (i + 1 >= text.size() || text[i + 1] != '\n') rowDone = true;
            } else if (c == '\n') {
                rowDone = true;
            } else {
                cell.push_back(c);
            }
            if (rowDone) {
                row.cells.push_back(cell);
                cell.clear();
                rows.push_back(row);
                row = Row{};
                row.line = line + 1;
                rowDone = false;
                ++line;
            }
        }
        if (quoted) errors.push_back(file + ": 引号没有闭合");
        if (!cell.empty() || !row.cells.empty()) {
            row.cells.push_back(cell);
            rows.push_back(row);
        }
        // 去掉全空行（文件末尾换行、段间空行）
        rows.erase(std::remove_if(rows.begin(), rows.end(), [](const Row &r) {
            return r.cells.size() == 1 && trim(r.cells[0]).empty();
        }), rows.end());
        return rows;
    }

    std::set<int> placeholders(const std::string &text) {
        std::set<int> out;
        for (std::size_t i = 0; i < text.size();) {
            if (text[i] == '{') {
                if (i + 1 < text.size() && text[i + 1] == '{') {
                    i += 2;
                    continue;
                }
                std::size_t j = i + 1;
                int idx = 0;
                bool digits = false;
                while (j < text.size() && std::isdigit(static_cast<unsigned char>(text[j]))) {
                    idx = idx * 10 + (text[j] - '0');
                    digits = true;
                    ++j;
                }
                if (digits && j < text.size() && text[j] == '}') {
                    out.insert(idx);
                    i = j + 1;
                    continue;
                }
            }
            ++i;
        }
        return out;
    }

    std::vector<std::string> termRefs(const std::string &text) {
        std::vector<std::string> out;
        for (std::size_t i = 0; i + 1 < text.size();) {
            if (text[i] == '{' && text[i + 1] == '{') {
                const std::size_t close = text.find("}}", i + 2);
                if (close == std::string::npos) break;
                out.emplace_back(text, i + 2, close - i - 2);
                i = close + 2;
                continue;
            }
            ++i;
        }
        return out;
    }

    std::string escape(const std::string &s) {
        std::string out;
        out.reserve(s.size() + 2);
        for (const char c: s) {
            switch (c) {
                case '\\': out += "\\\\";
                    break;
                case '"': out += "\\\"";
                    break;
                case '\n': out += "\\n";
                    break;
                case '\r': out += "\\r";
                    break;
                case '\t': out += "\\t";
                    break;
                default: out.push_back(c);
            }
        }
        return out;
    }

    bool loadTable(const fs::path &file, std::vector<Item> &out, std::vector<std::string> &langCodes,
                   std::vector<std::string> &errors, const bool machine) {
        std::ifstream in(file, std::ios::binary);
        if (!in) {
            errors.push_back(file.string() + ": 打不开");
            return false;
        }
        std::ostringstream buf;
        buf << in.rdbuf();
        const std::string name = file.filename().string();
        std::vector<Row> rows = parseCsv(buf.str(), errors, name);
        if (rows.empty()) {
            errors.push_back(name + ": 空文件");
            return false;
        }

        const std::vector<std::string> &head = rows.front().cells;
        if (head.size() < 2 || trim(head[0]) != "key") {
            errors.push_back(name + ": 表头必须是 key,<语言码>,…");
            return false;
        }
        std::vector<std::string> codes;
        for (std::size_t i = 1; i < head.size(); ++i) {
            std::string code = trim(head[i]);
            if (code == kSourceAlias) code = kDefaultLang;
            codes.push_back(code);
        }
        if (codes.front() != kDefaultLang)
            errors.push_back(name + ": 第一列语言必须是 " + kDefaultLang + "（回退链的终点）");
        if (machine) {
            langCodes = codes;
        } else {
            if (langCodes.empty()) langCodes = codes;
            else if (langCodes != codes) {
                errors.push_back(name + ": 语言列与其它文件不一致（应为 key," + [&] {
                    std::string s;
                    for (const auto &c: langCodes) s += c + ",";
                    return s.substr(0, s.size() - 1);
                }() + "）");
                return false;
            }
        }

        for (std::size_t r = 1; r < rows.size(); ++r) {
            const auto &[line, cells] = rows[r];
            const std::string where = name + ":" + std::to_string(line);
            if (cells.size() != head.size()) {
                errors.push_back(where + ": 列数 " + std::to_string(cells.size()) + "，表头是 " +
                                 std::to_string(head.size()));
                continue;
            }
            Item item;
            item.key = trim(cells[0]);
            item.origin = where;
            if (item.key.empty()) {
                errors.push_back(where + ": key 为空");
                continue;
            }
            if (item.key.find_first_of(" \t,") != std::string::npos) {
                errors.push_back(where + ": key 里不能有空格或逗号：" + item.key);
                continue;
            }
            for (std::size_t i = 1; i < cells.size(); ++i) item.text.push_back(cells[i]);
            if (item.text.front().empty()) {
                errors.push_back(where + ": " + kDefaultLang + " 为空（" + item.key + "）");
                continue;
            }
            out.push_back(std::move(item));
        }
        return true;
    }

    void emitTable(std::ostream &os, const char *name, const char *countName, const std::vector<Item> &items,
                   const std::size_t langCount) {
        if (items.empty()) {
            os << "extern const I18n::Detail::Entry " << name << "[1] = {{{}, {}}};\n";
            os << "extern const std::size_t " << countName << " = 0;\n";
            return;
        }
        os << "extern const I18n::Detail::Entry " << name << "[] = {\n";
        for (const auto &item: items) {
            os << "        {\"" << escape(item.key) << "\", {";
            for (std::size_t i = 0; i < langCount; ++i) {
                os << "\"" << escape(item.text[i]) << "\"";
                if (i + 1 < langCount) os << ", ";
            }
            os << "}},\n";
        }
        os << "};\n";
        os << "extern const std::size_t " << countName << " = " << items.size() << ";\n";
    }

    std::vector<fs::path> csvFiles(const fs::path &dir) {
        std::vector<fs::path> out;
        std::error_code ec;
        if (!fs::is_directory(dir, ec)) return out;
        for (const auto &e: fs::directory_iterator(dir, ec))
            if (e.is_regular_file() && e.path().extension() == ".csv") out.push_back(e.path());
        std::sort(out.begin(), out.end());
        return out;
    }

    std::map<std::string, std::size_t> keyIndex(const std::vector<Item> &items) {
        std::map<std::string, std::size_t> out;
        for (std::size_t i = 0; i < items.size(); ++i) out.emplace(items[i].key, i);
        return out;
    }
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "用法: gen_lang <输出文件> <语言目录>... [--mt <机翻目录>]...\n";
        return 2;
    }
    const fs::path outFile = argv[1];
    std::vector<fs::path> langDirs, mtDirs;
    for (int i = 2; i < argc; ++i) {
        if (const std::string arg = argv[i]; arg == "--mt") {
            if (++i >= argc) {
                std::cerr << "gen_lang: --mt 后面缺目录\n";
                return 2;
            }
            mtDirs.emplace_back(argv[i]);
        } else {
            langDirs.emplace_back(argv[i]);
        }
    }

    std::vector<std::string> errors;
    std::vector<std::string> baseCodes;
    std::vector<Item> entries;
    std::vector<Item> terms;

    for (const auto &dir: langDirs) {
        for (const auto &f: csvFiles(dir)) loadTable(f, entries, baseCodes, errors, false);
        for (const auto &f: csvFiles(dir / "terms")) loadTable(f, terms, baseCodes, errors, false);
    }
    if (baseCodes.empty()) {
        std::cerr << "gen_lang: 人工表里一个 csv 都没有\n";
        return 1;
    }

    // 机翻表：目录不存在就是没跑过翻译脚本，静默跳过。
    std::vector<MachineRow> machine;
    std::set<std::string> mtCodes;
    for (const auto &dir: mtDirs) {
        for (const auto &f: csvFiles(dir)) {
            std::vector<Item> items;
            std::vector<std::string> codes;
            loadTable(f, items, codes, errors, true);
            for (const auto &it: items) {
                MachineRow row;
                row.key = it.key;
                row.source = it.text.front();
                for (std::size_t i = 1; i < codes.size() && i < it.text.size(); ++i) {
                    if (it.text[i].empty()) continue;
                    row.text.emplace_back(codes[i], it.text[i]);
                    mtCodes.insert(codes[i]);
                }
                machine.push_back(std::move(row));
            }
        }
        for (const auto &f: csvFiles(dir / "terms")) {
            std::vector<Item> items;
            std::vector<std::string> codes;
            loadTable(f, items, codes, errors, true);
            for (const auto &it: items) {
                MachineRow row;
                row.key = it.key;
                row.source = it.text.front();
                for (std::size_t i = 1; i < codes.size() && i < it.text.size(); ++i) {
                    if (it.text[i].empty()) continue;
                    row.text.emplace_back(codes[i], it.text[i]);
                    mtCodes.insert(codes[i]);
                }
                machine.push_back(std::move(row));
            }
        }
    }

    // 语言顺序：人工表的列在前，机翻新增的语言按 langRank 补在后面。
    std::vector<std::string> langCodes = baseCodes;
    for (const auto &c: mtCodes)
        if (std::find(langCodes.begin(), langCodes.end(), c) == langCodes.end()) langCodes.push_back(c);
    std::stable_sort(langCodes.begin(), langCodes.end(), [](const std::string &a, const std::string &b) {
        return langRank(a) < langRank(b);
    });
    if (langCodes.front() != kDefaultLang)
        errors.push_back("第一列语言必须是 " + std::string(kDefaultLang));
    if (langCodes.size() > kMaxLanguages)
        errors.push_back("语言列 " + std::to_string(langCodes.size()) + " 个，超过上限 " +
                         std::to_string(kMaxLanguages));

    auto langPos = [&](const std::string &code) -> int {
        const auto it = std::find(langCodes.begin(), langCodes.end(), code);
        return it == langCodes.end() ? -1 : static_cast<int>(it - langCodes.begin());
    };

    // 把人工表从 baseCodes 的排布摊到最终语言列上；空位稍后统一回退英文。
    const auto widen = [&](std::vector<Item> &items) {
        for (auto &item: items) {
            std::vector<std::string> wide(langCodes.size());
            for (std::size_t i = 0; i < baseCodes.size() && i < item.text.size(); ++i) {
                if (const int p = langPos(baseCodes[i]); p >= 0) wide[p] = item.text[i];
            }
            item.text = std::move(wide);
        }
    };
    widen(entries);
    widen(terms);

    std::map<std::string, std::size_t> entryIdx = keyIndex(entries);
    std::map<std::string, std::size_t> termIdx = keyIndex(terms);
    std::map<std::string, std::size_t> translated;
    std::size_t stale = 0, unknown = 0, badPlaceholder = 0;
    for (const auto &[key, source, text]: machine) {
        bool found = false;
        for (auto *index: {&entryIdx, &termIdx}) {
            const auto hit = index->find(key);
            if (hit == index->end()) continue;
            found = true;
            Item &item = (index == &entryIdx ? entries : terms)[hit->second];
            if (item.text.front() != source) {
                // 中文改过，旧译文作废
                ++stale;
                continue;
            }
            for (const auto &[code, t]: text) {
                const int p = langPos(code);
                if (p <= 0) continue;
                if (placeholders(t) != placeholders(item.text.front())) {
                    ++badPlaceholder;
                    continue;
                }
                item.text[p] = t;
                ++translated[code];
            }
        }
        if (!found) ++unknown;
    }

    const int enPos = langPos("EN");
    std::map<std::string, std::size_t> fallback;
    const auto fillGaps = [&](std::vector<Item> &items) {
        for (auto &item: items) {
            const std::string &en = enPos > 0 ? item.text[enPos] : item.text.front();
            for (std::size_t i = 1; i < item.text.size(); ++i)
                if (item.text[i].empty()) {
                    item.text[i] = en.empty() ? item.text.front() : en;
                    ++fallback[langCodes[i]];
                }
        }
    };
    fillGaps(entries);
    fillGaps(terms);

    // 键与术语名各自唯一
    const auto checkDup = [&errors](const std::vector<Item> &v, const char *what) {
        std::map<std::string, std::string> seen;
        for (const auto &item: v) {
            if (const auto hit = seen.find(item.key); hit != seen.end())
                errors.push_back(item.origin + ": " + what + " 重复键 " + item.key + "（另见 " + hit->second + "）");
            else seen[item.key] = item.origin;
        }
    };
    checkDup(entries, "文案");
    checkDup(terms, "术语");

    // 术语引用必须存在
    std::set<std::string> termNames;
    for (const auto &t: terms) termNames.insert(t.key);
    const auto checkRefs = [&](const std::vector<Item> &v) {
        for (const auto &[key, text, origin]: v) {
            std::set<std::string> refs; // 一行里多语言引同一个术语只报一次
            for (const auto &t: text)
                for (const auto &ref: termRefs(t)) refs.insert(ref);
            for (const auto &ref: refs)
                if (!termNames.count(ref))
                    errors.push_back(origin + ": {{" + ref + "}} 未定义（" + key + "）");
        }
    };
    checkRefs(entries);
    checkRefs(terms);

    // 人工表自身的 {n} 索引集合必须一致；机翻格在合并时已逐格查过，不再重复报。
    for (const auto &[key, text, origin]: [&] {
        std::vector<Item> all = entries;
        all.insert(all.end(), terms.begin(), terms.end());
        return all;
    }()) {
        const std::set<int> base = placeholders(text.front());
        for (std::size_t i = 1; i < text.size(); ++i) {
            if (text[i].empty()) {
                errors.push_back(origin + ": " + langCodes[i] + " 为空（" + key + "）");
                continue;
            }
            if (placeholders(text[i]) == base) continue;
            std::string miss, extra;
            const std::set<int> cur = placeholders(text[i]);
            for (const int k: base) if (!cur.count(k)) miss += "{" + std::to_string(k) + "}";
            for (const int k: cur) if (!base.count(k)) extra += "{" + std::to_string(k) + "}";
            errors.push_back(origin + ": " + langCodes[i] + " 占位与 " + kDefaultLang +
                             " 不一致（少 " + (miss.empty() ? "无" : miss) + "，多 " +
                             (extra.empty() ? "无" : extra) + "）");
        }
    }

    std::sort(entries.begin(), entries.end(), [](const Item &a, const Item &b) { return a.key < b.key; });
    std::sort(terms.begin(), terms.end(), [](const Item &a, const Item &b) { return a.key < b.key; });

    if (!errors.empty()) {
        std::cerr << "gen_lang: " << errors.size() << " 处问题，未生成 " << outFile.string() << "\n";
        for (const auto &e: errors) std::cerr << "  " << e << "\n";
        return 1;
    }

    std::error_code ec;
    if (outFile.has_parent_path()) fs::create_directories(outFile.parent_path(), ec);

    const std::vector<std::string> names = [&] {
        std::vector<std::string> v;
        v.reserve(langCodes.size());
        for (const auto &c: langCodes) v.push_back(nameOf(c));
        return v;
    }();
    std::ofstream os(outFile, std::ios::binary | std::ios::trunc);
    if (!os) {
        std::cerr << "gen_lang: 写不了 " << outFile.string() << "\n";
        return 1;
    }
    os << "#include <cstddef>\n#include <string_view>\n\n#include \"i18n/I18n.h\"\n\n";
    os << "namespace I18n::Detail {\n";
    os << "extern const std::string_view kLangCodes[] = {";
    for (const auto &langCode: langCodes) os << "\"" << escape(langCode) << "\", ";
    os << "};\n";
    os << "extern const std::string_view kLangNames[] = {";
    for (std::size_t i = 0; i < langCodes.size(); ++i) {
        const std::string &nm = i < names.size() ? names[i] : langCodes[i];
        os << "\"" << escape(nm) << "\", ";
    }
    os << "};\n";
    os << "extern const std::size_t kLangCount = " << langCodes.size() << ";\n\n";
    emitTable(os, "kTerms", "kTermCount", terms, langCodes.size());
    os << "\n";
    emitTable(os, "kEntries", "kEntryCount", entries, langCodes.size());
    os << "}\n";
    if (!os) {
        std::cerr << "gen_lang: 写 " << outFile.string() << " 失败\n";
        return 1;
    }
    return 0;
}
