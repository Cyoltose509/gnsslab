#pragma once

#include <string>
#include <vector>

namespace AppConfig {
    class Products {
    public:
        static Products &instance();

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
        std::string met;


        void load();



        [[nodiscard]] std::vector<std::string> VMF3Paths() const;

        void save() const;



        void syncFromFiles();

    private:
        Products() = default;
    };
}
