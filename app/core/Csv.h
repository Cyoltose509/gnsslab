#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <vector>

namespace Csv {
    // 数字字段：携带自定义格式，避免在调用点散落 setprecision。
    struct SigNum {
        double v;
        int sig;
    }; // 有效数字位数
    struct FixedNum {
        double v;
        int dec;
    }; // 定点小数位数

    inline SigNum f(const double v, const int sig = 6) { return {v, sig}; }
    inline FixedNum fx(const double v, const int dec) { return {v, dec}; }

    // ---------------- 写出 ----------------
    class Writer {
    public:
        explicit Writer(const char sep = ',');

        // ---- 追加一个字段（自动处理引号转义）----
        Writer &cell(const std::string &v);
        Writer &cell(const char *v);
        Writer &cell(const bool b);
        Writer &cell(const int v);
        Writer &cell(const unsigned int v);
        Writer &cell(const long v);
        Writer &cell(const unsigned long v);
        Writer &cell(const double v, const int sig = 6);
        Writer &cell(const SigNum n);
        Writer &cell(const FixedNum n);

        // 结束当前行并开始新行。返回 *this 以支持流式链式调用。
        Writer &endRow();

        // 一次性写完整一行（自动结束行）。各字段按重载自动格式化。
        template<typename... Ts>
        Writer &row(Ts &&... ts) {
            (cell(std::forward<Ts>(ts)), ...);
            return endRow();
        }

        // 上一完整行的字段数（用于未解算历元写等长空占位行，保证列数一致）。
        [[nodiscard]] int colCount() const;

        // ---- 输出 ----
        [[nodiscard]] const std::string &str() const;

        [[nodiscard]] bool save(const std::string &path) const;
        [[nodiscard]] bool save(const std::wstring &path) const;

    private:
        Writer &cellRaw(const std::string &s);
        // 含分隔符/引号/换行的字段必须包裹并转义内部引号（RFC 4180）。
        [[nodiscard]] std::string quote(const std::string &s) const;

        char sep;
        std::string buf;
        int curCols = 0;
        int lastCols = 0;
        bool first = true;
    };

    // ---------------- 读取 ----------------
    // RFC4180 风格解析：支持引号包裹、字段内逗号/换行、"" 转义引号。
    class Reader {
    public:
        static std::vector<std::vector<std::string> > parse(const std::string &text);
        static std::vector<std::vector<std::string> > parse(const std::istream &in);
        static std::vector<std::vector<std::string> > readFile(const std::string &path);
    };
}
