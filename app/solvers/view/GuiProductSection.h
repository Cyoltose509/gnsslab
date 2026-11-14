#pragma once

#include <functional>
#include <string>

#include "download/local/ProductMeta.h"
#include "ui/Gui.h"

namespace AppConfig {
    class Products;
}

namespace Solvers::Helpers {
    /// 文件行：输入框 + 浏览，「获取」一律内联展开取文件面板。
    /// obsType 只作用于观测行：普通观测取 IGS 观测目录，低轨观测取低轨源表。
    void fileRow(const char *label, std::string *buf, const char *iniKey,
                 const GuiFileFilter *filters, int nFilters,
                 const std::function<void(const std::string &)> &onSelect = {},
                 const char *obsIniKey = nullptr,
                 std::string *autoNavBuf = nullptr,
                 ProductType obsType = ProductType::OBS);

    /// 产品文件清单与产品一致性提示。`usesTropo` 为假时「获取」列表默认不勾对流层产品。
    /// `crossDayHint` 为真时（低轨跨日弧）在清单上方给出「建议选择 ±1 日产品」的提示。
    void renderProductSection(AppConfig::Products &p, const char *obsIniKey, bool usesTropo,
                             bool crossDayHint = false);
}
