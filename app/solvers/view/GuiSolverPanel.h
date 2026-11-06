#pragma once


#include <memory>
#include <vector>


#include "solvers/common/Task.h"

namespace Solvers::SolverPanel {
    template<typename T>
    void copySampled(const std::vector<T> &src, std::vector<T> &dst,
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

    void renderTask(const std::shared_ptr<Task> &job);
    void launchQC(View &view);
    void exportCSV(const std::shared_ptr<Task> &task);
}
