#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "download/local/ProductType.h"
#include "solver/Products.h"

namespace AppConfig {
    class ProductPaths;
}

/// 产品读取：按路径表并行读出一份 Products，缺项跳过，读失败只落日志不抛。
/// 产品身份沿用 ProductType（下载库那张唯一词表），各解算器只列出自己要的那几项。
namespace ProductReader {
    struct Loaded {
        std::shared_ptr<Products> prod = std::make_shared<Products>();
        bool readOk = true; // 凡是给了路径的产品都读成功
    };

    /// 读哪些项直接决定结果，别为了「看起来统一」把 LEO 的清单并成 PPP 的
    Loaded load(const AppConfig::ProductPaths &d, const std::string &tag,
                const std::set<SatSystem> &systems, const std::vector<ProductType> &wanted);

    /// 产品身份与该次运行实际用的路径，按产品表顺序列出：供「参与文件」这类只读用途遍历
    struct Info {
        ProductType type;
        const std::string *path;
    };

    std::vector<Info> describe(const AppConfig::ProductPaths &d);
}
