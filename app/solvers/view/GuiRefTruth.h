#pragma once

#include <string>

namespace Solvers {
    struct Task;
}

namespace Solvers::RefTruth {
    /// 渲染「参考真值」整体 UI：恒值 ECEF 或 SP3 真值文件。
    /// realtime 下只留恒值坐标手填——实时流无 obs 头，联网/下载真值无从解析。
    void renderRefTruthSection(Task *task, bool realtime = false);

    /// 把当前真值形式、恒值坐标与 SP3 路径写入 PathMemory（开始解算时调用）。
    void persist(const Task *task);

    /// 从 PathMemory 恢复真值形式与被记住的值（loadFromMemory 中调用）。
    void restore(Task *task);
}
