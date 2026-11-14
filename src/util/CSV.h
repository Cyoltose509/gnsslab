#pragma once

#include <iosfwd>
#include <string>
#include <filesystem>
#include <vector>

namespace CSV {
    struct SigNum {
        double v;
        int sig;
    };
    struct FixedNum {
        double v;
        int dec;
    };
    struct SciNum {
        double v;
        int dec;
    };

    inline SigNum f(const double v, const int sig = 6) { return {v, sig}; }
    inline FixedNum fx(const double v, const int dec) { return {v, dec}; }
    inline SciNum sci(const double v, const int dec) { return {v, dec}; }

    class Writer {
    public:
        explicit Writer(char sep = ',');

        /// 流式模式：绑定输出流，每写完一行直接落流，不再攒整个文件再 str()。
        explicit Writer(std::ostream &out, char sep = ',');

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

        /// 空行：报表分节之间的分隔（只补一个换行，不写任何单元格）。
        Writer &blank() { return endRow(); }


        template<typename... Ts>
        Writer &row(Ts &&... ts) {
            (cell(std::forward<Ts>(ts)), ...);
            return endRow();
        }

        [[nodiscard]] int colCount() const;


        [[nodiscard]] const std::string &str() const;

        /// UTF-8 BOM：Excel 靠它认编码；落盘统一走 File.h，BOM 由调用方先写。
        static void writeBom(std::ostream &out);

    private:
        Writer &cellRaw(const std::string &s);

        [[nodiscard]] std::string quote(const std::string &s) const;

        char sep;
        std::ostream *sink = nullptr;
        std::string buf;
        int curCols = 0;
        int lastCols = 0;
        bool first = true;
    };

    struct Meta {
        Writer &w;
        const char *kind = "";
        const char *flagOn = "1";
        const char *flagOff = "0";

        void item(const char *k, const char *name, const std::string &value) const { w.row(k, name, value); }
        void num(const char *name, const double v) const { w.row(kind, name, f(v, 10)); }
        void integer(const char *name, const int v) const { w.row(kind, name, v); }
        void flag(const char *name, const bool v) const { w.row(kind, name, v ? flagOn : flagOff); }
        void text(const char *name, const char *v) const { w.row(kind, name, v); }
        void xyz(const char *name, const double x, const double y, const double z) const {
            w.row(kind, name, std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z));
        }

        void section(const std::string &name) const { w.cell("[" + name + "]").endRow(); }
    };


    class Reader {
    public:
        static std::vector<std::vector<std::string> > parse(const std::string &text);
        static std::vector<std::vector<std::string> > parse(const std::istream &in);
        static std::vector<std::vector<std::string> > readFile(const std::string &path);
    };
}
