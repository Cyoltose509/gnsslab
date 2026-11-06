#pragma once

#include <string>
#include <vector>

namespace AppConfig {
    /// 精密产品清单，每个求解模式各一份：ini 键 = <scope>_files / <scope>_sp3 …
    class Products {
    public:
        explicit Products(std::string scope);

        static Products &of(const std::string &scope);

        std::vector<std::string> files;

        std::string sp3;
        std::string clk;
        std::string brdc;
        std::string osb;
        std::string erp;
        std::string tro;
        std::string ionex;
        std::string vmf;
        std::string att;


        void load();

        void save() const;

        void syncFromFiles();

        [[nodiscard]] std::vector<std::string> VMF3Paths() const;

        [[nodiscard]] std::vector<std::string> recent() const;

        void addRecent(const std::string &path) const; //NOLINT

    private:
        [[nodiscard]] std::string key(const char *name) const;

        std::string scopeName;
    };
}
