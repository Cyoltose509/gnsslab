#include "Csv.h"

#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <vector>

namespace Csv {

// ---------------- Writer ----------------
Writer::Writer(const char sep) : sep(sep) {}

Writer &Writer::cell(const std::string &v) { return cellRaw(quote(v)); }

Writer &Writer::cell(const char *v) {
    return cellRaw(quote(v ? std::string(v) : std::string()));
}

Writer &Writer::cell(const bool b) { return cellRaw(b ? "1" : "0"); }
Writer &Writer::cell(const int v) { return cellRaw(std::to_string(v)); }
Writer &Writer::cell(const unsigned int v) { return cellRaw(std::to_string(v)); }
Writer &Writer::cell(const long v) { return cellRaw(std::to_string(v)); }
Writer &Writer::cell(const unsigned long v) { return cellRaw(std::to_string(v)); }

Writer &Writer::cell(const double v, const int sig) {
    std::ostringstream os;
    os << std::setprecision(sig) << v;
    return cellRaw(os.str());
}

Writer &Writer::cell(const SigNum n) {
    std::ostringstream os;
    os << std::setprecision(n.sig) << n.v;
    return cellRaw(os.str());
}

Writer &Writer::cell(const FixedNum n) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(n.dec) << n.v;
    return cellRaw(os.str());
}

Writer &Writer::endRow() {
    buf += '\n';
    lastCols = curCols;
    curCols = 0;
    first = true;
    return *this;
}

[[nodiscard]] int Writer::colCount() const { return lastCols; }

[[nodiscard]] const std::string &Writer::str() const { return buf; }

[[nodiscard]] bool Writer::save(const std::string &path) const {
    std::ofstream out(std::filesystem::path(path), std::ios::out | std::ios::binary);
    if (!out) return false;
    out << buf; // UTF-8，无 BOM
    return static_cast<bool>(out);
}

[[nodiscard]] bool Writer::save(const std::wstring &path) const {
    std::ofstream out(std::filesystem::path(path), std::ios::out | std::ios::binary);
    if (!out) return false;
    out << buf;
    return static_cast<bool>(out);
}

Writer &Writer::cellRaw(const std::string &s) {
    if (!first) buf += sep;
    else first = false;
    buf += s;
    ++curCols;
    return *this;
}

[[nodiscard]] std::string Writer::quote(const std::string &s) const {
    if (s.find(sep) != std::string::npos ||
        s.find('"') != std::string::npos ||
        s.find('\n') != std::string::npos ||
        s.find('\r') != std::string::npos) {
        std::string out = "\"";
        for (const char c: s) {
            if (c == '"') out += "\"\"";
            else out += c;
        }
        out += "\"";
        return out;
    }
    return s;
}

// ---------------- Reader ----------------
std::vector<std::vector<std::string> > Reader::parse(const std::string &text) {
    std::vector<std::vector<std::string> > rows;
    std::vector<std::string> fields;
    std::string cur;
    bool inQuotes = false;
    const size_t n = text.size();
    for (size_t i = 0; i < n; ++i) {
        const char c = text[i];
        if (inQuotes) {
            if (c == '"') {
                if (i + 1 < n && text[i + 1] == '"') {
                    cur += '"';
                    ++i;
                } else inQuotes = false;
            } else {
                cur += c;
            }
        } else {
            if (c == '"') {
                inQuotes = true;
            } else if (c == ',') {
                fields.push_back(cur);
                cur.clear();
            } else if (c == '\n') {
                fields.push_back(cur);
                rows.push_back(std::move(fields));
                fields.clear();
                cur.clear();
            } else if (c == '\r') {
                // 跳过，兼容 \r\n
            } else {
                cur += c;
            }
        }
    }
    if (!cur.empty() || !fields.empty()) {
        fields.push_back(cur);
        rows.push_back(std::move(fields));
    }
    return rows;
}

std::vector<std::vector<std::string> > Reader::parse(const std::istream &in) {
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse(ss.str());
}

std::vector<std::vector<std::string> > Reader::readFile(const std::string &path) {
    std::ifstream f;
    f.open(std::filesystem::path(path));
    if (!f) return {};
    return parse(f);
}

} // namespace Csv
