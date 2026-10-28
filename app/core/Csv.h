#pragma once

#include <string>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <vector>

namespace Csv {
    struct SigNum {
        double v;
        int sig;
    }; // 有效数字位数
    struct FixedNum {
        double v;
        int dec;
    }; // 定点小数位数（= printf "%.<dec>f"）
    struct SciNum {
        double v;
        int dec;
    }; // 科学计数小数位数（= printf "%.<dec>e"）

    inline SigNum f(const double v, const int sig = 6) { return {v, sig}; }
    inline FixedNum fx(const double v, const int dec) { return {v, dec}; }
    inline SciNum sci(const double v, const int dec) { return {v, dec}; }

    class Writer {
    public:
        explicit Writer(char sep = ',');

        Writer &cell(const std::string &v);
        Writer &cell(const char *v);
        Writer &cell(bool b);
        Writer &cell(int v);
        Writer &cell(unsigned int v);
        Writer &cell(long v);
        Writer &cell(unsigned long v);
        Writer &cell(double v, int sig = 6);
        Writer &cell(SigNum n);
        Writer &cell(FixedNum n);
        Writer &cell(SciNum n);


        Writer &endRow();


        template<typename... Ts>
        Writer &row(Ts &&... ts) {
            (cell(std::forward<Ts>(ts)), ...);
            return endRow();
        }

        [[nodiscard]] int colCount() const;

        // ---- 输出 ----
        [[nodiscard]] const std::string &str() const;

        [[nodiscard]] bool save(const std::string &path, bool withBom = false) const;
        [[nodiscard]] bool save(const std::wstring &path, bool withBom = false) const;

    private:
        Writer &cellRaw(const std::string &s);

        [[nodiscard]] std::string quote(const std::string &s) const;

        char sep;
        std::string buf;
        int curCols = 0;
        int lastCols = 0;
        bool first = true;
    };


    class Reader {
    public:
        static std::vector<std::vector<std::string> > parse(const std::string &text);
        static std::vector<std::vector<std::string> > parse(const std::istream &in);
        static std::vector<std::vector<std::string> > readFile(const std::string &path);
    };
}
