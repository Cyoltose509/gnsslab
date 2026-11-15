#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "download/local/ProductType.h"

namespace AppConfig {
    class ProductPaths;
}

namespace Download::Acquire {
    /// 解算模式里内联展开的「取文件」面板：选产品、选观测文件、选单个产品共用一套。
    struct Panel {
        enum class Kind { Products, Obs, Single };

        Kind kind = Kind::Products;
        bool open = false;
        int year = 0, doy = 0;
        std::vector<std::string> *target = nullptr; // Products：解算产品清单
        AppConfig::ProductPaths *owner = nullptr;   // 清单所属的模式产品池
        std::string obsKey;                         // Obs：PathMemory 键
        std::string *obsOut = nullptr;              // Obs：回填到该填写框
        ProductType obsType = ProductType::OBS;     // Obs：观测类别（低轨观测走另一套源表）
        ProductType singleType = ProductType::Unknown;
        std::string *singleOut = nullptr;
        std::function<void(const std::string &)> onPick; // Single：回填后的副作用
        float height = 280.0f;      // 文件列表高度

        std::vector<std::uint8_t> typeOn;  // 产品类型多选
        /// 该模式是否用得上对流层产品：低轨卫星在大气层外，默认不勾「对流层」。
        bool usesTropo = true;
        char nameFind[40] = {};            // 文件名关键字
        int stateFilter = 0;               // 本地状态

        /// 归档产品展开成目录后，里面常是十几到几百份观测（COSMIC 按星号 + 弧段切）。
        /// membersOf 非空表示当前正在挑成员，产品列表让位给成员列表。
        std::string membersOf;
        std::vector<std::string> members;
        char memberFind[40] = {};
        std::string memberGroup; // 选中的分组（星号这类公共前缀之后的第一段），空＝全部

        /// 该路径当前是否已被选入（观测框 / 单文件回填 / 产品清单三种去向）。
        [[nodiscard]] bool isPicked(const std::string &local) const;

        /// 切换选中：观测与单文件走回填空串，产品清单增删条目。
        void pick(const std::string &local, bool selected) const;
    };

    bool render(Panel &p, const char *idScope);
}
