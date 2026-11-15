#include "SiteTruth.h"

namespace SiteTruth {
    namespace {
        Fetcher g_fetcher;
    }

    Fetcher &fetcher() { return g_fetcher; }
}
