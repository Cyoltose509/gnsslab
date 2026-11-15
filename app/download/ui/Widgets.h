#pragma once

#include <functional>
#include <string>
#include <vector>

#include "download/remote/Fetcher.h"
#include "i18n/I18n.h"

namespace Download::Widgets {
    void dateRow(int &year, int &month, int &day, const char *id, const char *quickLabel,
                 void (*quick)(int &, int &, int &), bool showWeek);

    void chipWrap(const std::function<const char *(int)> &label, std::vector<unsigned char> &on);

    void typeChips(std::vector<unsigned char> &on);

    bool progressLine(const Fetcher::Snapshot &snap);

    void failedSources(const Fetcher::Snapshot &snap);

    void downloadChip(const char *label, float frac, float width, const char *tip,
                      const char *stage = nullptr);

    void stateFilterChips(const char *scope, int *v);

    void searchBox(const char *id, char *buf, int size);

    void askDelete(const std::vector<std::string> &paths);

    bool deleteConfirm();

    std::string sizeText(long long bytes);

    std::string progressText(const char *verb, const std::string &file, double frac,
                             double gotBytes, double totalBytes, double speed = 0.0);

    void renderProgress(const char *verb, const std::string &file, double frac,
                        double gotBytes, double totalBytes, double speed = 0.0);

    enum class Grouping {
        None,
        ByType
    };

    struct ListOptions {
        Grouping grouping = Grouping::ByType;
        bool scanning = false;
        int stateFilter = 0;
        const char *find = nullptr;
        std::vector<std::string> notes;
        std::function<bool(const std::string &local)> isSelected;

        std::function<void(const Fetcher::Match &, const std::string &local)> onClick;

        std::function<std::string(bool selected)> localHint =
            [](bool) { return I18n::tr("dl.ui.hint.redownload"); };
    };

    struct ListResult {
        int shown = 0;
        int total = 0;
    };

    ListResult fileList(std::vector<Fetcher::Match> &matches, const ListOptions &opt);

    ListResult remoteList(const Fetcher::Snapshot &snap, const ListOptions &opt, const char *childId,
                          const char *noun, float height = 0.0f);
}
