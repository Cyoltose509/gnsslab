#pragma once

#include <string>

namespace Solvers {
    struct Task;
}

namespace Solvers::RefTruth {
    /// 渲染「参考真值」整体 UI：勾选 + X/Y/Z 填写 + 从剪贴板获取 + 联网获取（WHU 数据中心）。
    /// task 持有状态位（useRefTruth / refXyz / refFetch*），iniPrefix 用于 PathMemory 持久化键前缀。
    void renderRefTruthField(Task *task);

    /// 把当前填写的参考真值写入 PathMemory（开始解算时调用）。
    void persist(Task *task, const char *iniPrefix);

    /// 从 PathMemory 恢复上次填写的参考真值（loadFromMemory 中调用）。
    void restore(Task *task, const char *iniPrefix);
}
