#pragma once

#include <functional>
#include <string>
#include <vector>

#include "Remote.h"

/// 下载界面的公共控件：胶囊按钮、状态筛选、关键字筛选、右键菜单与候选列表。
namespace Download::Widgets {
    /// 圆角胶囊按钮，选中时高亮。
    bool tagChip(const char *label, bool on);

    /// 胶囊按 `label` 算出来的宽度，供换行排版用。
    float chipWidth(const char *label);

    /// 列表下沿的水平拖拽把手：按住上下拖动改 height，越界按 minH/maxH 夹住。
    void heightGrip(const char *id, float &height, float minH, float maxH);

    /// 日期行：日历按钮（弹出月历）+ 快捷按钮 + 年积日（可选 GPS 周）。
    /// `quick` 为空则不画按钮，否则点击后由它把年月日改成目标日期。
    void dateRow(int &year, int &month, int &day, const char *id, const char *quickLabel,
                 void (*quick)(int &, int &, int &), bool showWeek);

    /// 一排 chip 按窗口宽度自动换行，点击就地取反 on[i]（0/1）。
    /// `label(i)` 给出第 i 个 chip 的文字，on 的个数即 chip 个数。
    void chipWrap(const std::function<const char *(int)> &label, std::vector<unsigned char> &on);

    /// 扫描/下载进行中时画进度行并返回 true，同时带出状态文本。
    bool progressLine(std::string &status);

    /// 远端才有（未下载）的胶囊样式：整体压暗一档。
    void pushRemoteChipStyle();

    void popRemoteChipStyle();

    /// 带百分比文字的进度胶囊；`frac < 0` 表示总量未知。
    void downloadChip(const char *label, float frac, float width, const char *tip);

    bool passStateFilter(int f, bool hasLocal);

    /// 本地状态三选一：全部 / 已下载 / 未下载。
    void stateFilterChips(const char *scope, int *v);

    /// 关键字筛选行：「搜索 [____] 清空」，就地改写 buf。
    void searchBox(const char *id, char *buf, int size);

    enum class FileAction { None, Reveal, Delete };

    /// 文件胶囊的右键菜单；选彻底删除会打开确认弹窗。
    FileAction fileContextMenu(const std::string &localPath);

    /// 挂起一次彻底删除，由 deleteConfirm() 弹出确认。
    void askDelete(const std::vector<std::string> &paths);

    /// 彻底删除确认弹窗，每帧调用；本帧真的删了东西则返回 true。
    bool deleteConfirm();

    /// 字节数写成 12.3 MB 这种，供列表与汇总用。
    std::string sizeText(long long bytes);

    /// 文件胶囊的分组方式。
    enum class Grouping {
        None,  ///< 平铺，不画分组标题
        ByType ///< 按产品类型分组，每组前画一行类型名
    };

    /// 一次列表渲染的策略。
    struct ListOptions {
        /// 分组方式。
        Grouping grouping = Grouping::ByType;

        /// 本地状态筛选：0 全部 / 1 已下载 / 2 未下载。
        int stateFilter = 0;

        /// 文件名关键字（大小写不敏感子串，同时比对远端名与本地落盘名），空串表示不筛。
        const char *find = nullptr;

        /// 该文件是否已选入清单。为真时在文件名前画「√ 」前缀。
        std::function<bool(const std::string &local)> isSelected;

        /// 点击回调。local 为空表示本地还没有这个文件。
        std::function<void(const Remote::Match &, const std::string &local)> onClick;

        /// 本地已有文件的 tooltip 末行，参数是当前是否已选中。
        std::function<std::string(bool selected)> localHint =
            [](bool) { return std::string("（点击可重新下载 · 右键管理）"); };
    };

    /// 筛选前后的条数，供调用方写「共 N / M 个」那行汇总。
    struct ListResult {
        int shown = 0;
        int total = 0;
    };

    /// 先按关键字、再按本地状态筛掉 matches（就地改写）再画成胶囊行，返回筛选前后的条数。
    /// 一条不剩时自己画一行占位文字（扫描中则写「扫描中…」）。
    ListResult fileList(std::vector<Remote::Match> &matches, const ListOptions &opt);
}
