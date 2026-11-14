#pragma once

#include <string>

namespace Solvers {
    struct Task;
}

namespace Solvers::RefTruth {
    /// 渲染「参考真值」整体 UI：静态给恒值 ECEF（剪贴板 / 联网取 SINEX），
    /// 动态给 SP3 真值文件（浏览）；SPP、RTK 两种都可切。
    void renderRefTruthSection(Task *task);

    /// 把当前真值形式、恒值坐标与 SP3 路径写入 PathMemory（开始解算时调用）。
    void persist(const Task *task);

    /// 从 PathMemory 恢复真值形式与被记住的值（loadFromMemory 中调用）。
    void restore(Task *task);
}
