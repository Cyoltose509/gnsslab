#pragma once


#include <memory>
#include <vector>


#include "solvers/common/Core.h"

namespace Solvers::SolverPanel {
    // 模板必须留在头文件（项目"定义出头规则"），即便当前仅本 TU 使用
    template<typename T>
    inline void copySampled(const std::vector<T> &src, std::vector<T> &dst,
                            const std::size_t maxPts = 3000) {
        if (src.size() <= maxPts) {
            dst = src;
            return;
        }
        std::size_t step = 1;
        while (step * maxPts < src.size()) step <<= 1;
        dst.clear();
        dst.reserve(src.size() / step + 2);
        for (std::size_t i = 0; i < src.size(); i += step)
            dst.push_back(src[i]);
        if ((src.size() - 1) % step != 0)
            dst.push_back(src.back());
    }

    void renderTask(const std::shared_ptr<Solvers::Task> &job);
    void launchQC(const std::shared_ptr<Solvers::Core> &task);
    void exportCSV(const std::shared_ptr<Solvers::Core> &task);
}
