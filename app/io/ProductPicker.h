#pragma once

#include "download/ProductMeta.h"

#include <string>
#include <vector>

// 扫描本地下载目录、按质量挑选精密产品（分析中心 / 延迟等级打分排序）。
namespace ProductPicker {
    struct Candidate {
        std::string path;
        std::string name;
        ProductMeta meta;
        std::string center;   // 分析中心
        std::string latency;  // 延迟等级 FIN / RAP / ULT
        int score = 0;        // 越大越好
        std::string groupKey;
        std::string slot;
        std::string reason;   // 入选理由
        long long size = 0;
    };

    std::vector<Candidate> rank(const std::vector<std::string> &paths, ProductType t);

    std::vector<Candidate> scan(ProductType t, int year, int doy, int backDays = 8);

    std::vector<Candidate> pick(ProductType t, int year, int doy);

    std::string describe(const Candidate &c);

    // 从给定路径中选出该类型最优的一份；无候选时回退首条路径，空输入返回空串。
    std::string bestPath(const std::vector<std::string> &paths, ProductType t);

    // 时次组类型（如 VMF）：最优组的全部时次路径都要；无候选返回原列表。
    std::vector<std::string> bestPaths(const std::vector<std::string> &paths, ProductType t);
}
