#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "download/ProductMeta.h"

namespace AppConfig {
    class Products;
}

namespace Download::Acquire {
    /// 解算模式里内联展开的「取文件」面板：选产品、选观测文件、选单个产品共用一套。
    struct Panel {
        enum class Kind { Products, Obs, Single };

        Kind kind = Kind::Products;
        bool open = false;
        int year = 0, doy = 0;
        std::vector<std::string> *target = nullptr; // Products：解算产品清单
        AppConfig::Products *owner = nullptr;       // Products：清单所属的模式产品池
        std::string obsKey;                         // Obs：PathMemory 键
        std::string *obsOut = nullptr;              // Obs：回填到该填写框
        ProductType singleType = ProductType::Unknown;
        std::string *singleOut = nullptr;
        std::function<void(const std::string &)> onPick; // Single：回填后的副作用
        float height = 280.0f;      // 文件列表高度

        std::vector<std::uint8_t> typeOn;  // 产品类型多选
        std::vector<std::uint8_t> dcOn;    // 数据中心多选
        char nameFind[40] = {};            // 文件名关键字
        int stateFilter = 0;               // 本地状态

        /// 该路径当前是否已被选入（观测框 / 单文件回填 / 产品清单三种去向）。
        [[nodiscard]] bool isPicked(const std::string &local) const;

        /// 切换选中：观测与单文件走回填空串，产品清单增删条目。
        void pick(const std::string &local, bool selected) const;
    };

    bool render(Panel &p, const char *idScope);
}
