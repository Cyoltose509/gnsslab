#pragma once

#include <string>
#include <vector>

#include "SolverOptions.h"

namespace AppConfig {
    /// 每模式一份产品路径池：ini 往返、从下载清单派生、与显式配置合并成最终路径表。
    /// 字段名与 typeDir 同词根，一个产品在这三处（ini 键、下载目录、解算输入）只有一个名字。
    class ProductPaths {
    public:
        explicit ProductPaths(std::string scope);

        static ProductPaths &of(const std::string &scope);

        std::string nav, sp3, clk, osb, att, erp, atx, tro, ion, vmf, gpt3;
        std::vector<std::string> files;

        /// 读 ini 并把版本型产品换成当前安装版本
        void load();

        void save() const;

        /// 以文件清单为准重算派生字段（下载页刚导入完时用）
        void syncFromFiles();

        [[nodiscard]] std::vector<std::string> VMF3Paths() const;

        /// 显式配置优先，缺项回落本池里的值；CLI 不读 ini 池，等价于只用显式配置
        [[nodiscard]] ProductPaths resolved(const SolverOptions &opts) const;

    private:
        [[nodiscard]] std::string key(const char *name) const;

        std::string scopeName;
    };
}
