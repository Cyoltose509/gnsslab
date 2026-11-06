#pragma once

#include <map>
#include <string>
#include <vector>

#include "GnssStruct.h"
#include "imgui.h"

/// 无状态的界面小工具：字符串输入、系统名、残差显示范围。
/// Task 的界面骨架（配置页/实时页/取消按钮）是它的成员，实现见 solvers/view/TaskGui.cpp。
namespace Solvers::Helpers {
    bool inputTextStd(const char *label, std::string *str, ImGuiInputTextFlags flags = 0);

    struct ConstellationName {
        char ch;
        const char *label;
    };

    inline constexpr ConstellationName kConstellations[] = {
        {'G', "GPS"}, {'R', "GLO"}, {'E', "GAL"}, {'C', "BDS"}
    };

    inline const char *systemName(const char s) {
        for (const auto &[ch, label]: kConstellations)
            if (ch == s) return label;
        return "?";
    }

    struct RobustResRange {
        double lo = -8.0, hi = 8.0;
    };

    RobustResRange computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes,
                                         double minHalf = 1.0);
}
