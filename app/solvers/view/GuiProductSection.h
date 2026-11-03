#pragma once

#include <functional>
#include <string>

#include "ui/Gui.h"

namespace Solvers::Helpers {
    void fileRow(const char *label, std::string *buf, const char *iniKey,
                 const GuiFileFilter *filters, int nFilters,
                 const std::function<void(const std::string &)> &onSelect = {},
                 const char *obsIniKey = nullptr,
                 std::string *autoNavBuf = nullptr);

    void renderProductSection(const char *obsIniKey);
}
