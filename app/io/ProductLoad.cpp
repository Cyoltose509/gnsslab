#include "ProductLoad.h"

namespace ProductLoad {
    namespace {
        std::function<Defaults(const std::string &scope)> g_defaults;
        SiteTruthFetcher g_siteTruth;
    }

    std::function<Defaults(const std::string &scope)> &defaultsProvider() {
        return g_defaults;
    }

    SiteTruthFetcher &siteTruthFetcher() {
        return g_siteTruth;
    }
}
