#pragma once

#include <string>
#include <vector>

#include "core/InsStruct.h"
#include "ins/ImuCalibrator.h"


namespace ImuCalibDetect {
    struct DetectParams {
        int winSamples = 50; // 滑窗长度（样本）
        double minSegSeconds = 0.15; // 段最短时长(s)
        int minSegSamples = 30; // 兜底
        double rotThrCounts = 30.0; // 旋转判定绝对下限（counts）
        double rotFrac = 0.4; // 旋转阈值
    };

    // 单段分类结果。i0/i1 为 [i0, i1) 样本区间，用于从原日志抽取子段。
    struct CalibSegment {
        bool isStatic = false; // true=静置, false=旋转
        ImuCalibrator::Pos pos = ImuCalibrator::XUP; // 仅 isStatic 有效
        ImuCalibrator::Rot rot = ImuCalibrator::XFWD; // 仅 !isStatic 有效
        size_t i0 = 0, i1 = 0; // [i0, i1) 样本区间
    };


    std::string fileContentHash(const std::string &path);


    std::vector<CalibSegment> detect(const ImuLog &log, const DetectParams &p = DetectParams{});

    const char *posLabel(ImuCalibrator::Pos p);

    const char *rotLabel(ImuCalibrator::Rot r);
}
