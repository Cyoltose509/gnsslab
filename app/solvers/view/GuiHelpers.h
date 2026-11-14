#pragma once

#include <map>
#include <string>
#include <vector>

#include "core/SatID.h"
#include "imgui.h"

/// 无状态的界面小工具：字符串输入、系统名、残差显示范围。
/// Task 的界面骨架（配置页/实时页/取消按钮）是它的成员，实现见 solvers/view/TaskGui.cpp。
namespace Solvers::Helpers {
    bool inputTextStd(const char *label, std::string *str, ImGuiInputTextFlags flags = 0);

    /// 「从剪贴板获取」按钮的统一形态：标签固定，`id` 只用来区分同屏出现的多个（如 LEO 的
    /// ARP / PCO 两段），传什么都行、不进标签。点中就把剪贴板里的前三个数写进 `xyz`
    /// （有几个写几个，单位换成 m）。`clicked` 为真而 `count` 为 0 时是剪贴板里一个数都没有，
    /// 调用方该提示一句。
    struct PasteResult {
        bool clicked = false;
        int count = 0;
        double xyz[3] = {0.0, 0.0, 0.0};
    };

    PasteResult clipboardXYZButton(const char *id);

    /// 结果页统一的时间游标导航：<< < 滑动条 > >> + 当前历元说明。
    void renderEpochNav(int &selectedEpoch, int epochCount,
                        const char *epochInfo = nullptr, const char *tailText = nullptr);

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
