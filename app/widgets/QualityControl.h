#pragma once
/**
 * QualityControl — 质量分析界面 (UI 层)
 * --------------------------------------------------------------------------
 * 本文件只负责 ImGui 渲染，所有质量评估算法已下沉到 lib/qc/QCProcessor.h
 * (QC::compute)。UI 把解算结果 (GuiFileProcessor::SppEpochData) 转换为
 * QC::QCObsEpoch 后调用 QC::compute() 得到 QC::QualityReport，再绘制。
 *
 * 为兼容 GuiFileProcessor.h 中 `QualityReport qcReport;` 等成员，这里用 using
 * 把 QC::SatQC / QC::QualityReport 重新暴露到全局，保持已有引用不变。
 */
#include <memory>
#include "QCProcessor.h"   // namespace QC

// 兼容别名：原 QualityControl.h 中定义的全局 SatQC / QualityReport 现位于 namespace QC
using QC::SatQC;
using QC::QualityReport;

// 前向声明，避免在头文件引入 GUI 依赖
namespace GuiFileProcessor { struct SppTask; struct SppEpochData; }

namespace QualityControl {
    /// 在主程序中渲染质量分析界面 (需在 ImGui 上下文内调用)
    void render(const std::shared_ptr<GuiFileProcessor::SppTask> &task);
}
