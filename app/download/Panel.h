#pragma once

#include <set>
#include <string>
#include <vector>

#include "Store.h"

namespace Download::Panel {
    /// 「产品下载」窗口的全部界面状态：三个页签各管自己的日期与筛选，互不干扰。
    /// 窗口是单例，实例由 state() 给出（落 Panel.cpp），外部只拿它开/关窗口。
    struct State {
        bool open = false;

        // 日精密产品页
        int year = 0, month = 0, day = 0;
        std::vector<unsigned char> typeOn;
        std::vector<unsigned char> dcOn;
        int dateStateFilter = 0;
        char dateFind[40] = {};

        // 观测文件页
        int obsYear = 0, obsMonth = 0, obsDay = 0;
        int obsStateFilter = 0;
        char obsFind[40] = {};

        // 辅助数据页：手工导入的结果提示
        std::string importMsg;
        bool importOk = false;

        // 本地文件页：扫描结果缓存到 localDirty，选中项按绝对路径记，跨筛选保留
        std::vector<Store::BrowseEntry> localFiles;
        std::set<std::string> localSel;
        char localFind[40] = {};
        bool localDirty = true;
        bool localShown = false;
        int localDone = 0; // 已完成的下载数，变化即说明有新文件落地

        /// 数据中心勾选取值，空集表示不限。
        [[nodiscard]] std::set<std::string> selectedDatacenters() const;

        void renderDateTab();

        void renderObsTab();

        void renderAuxTab();

        /// 浏览本地库里的日期产品与辅助产品：左键定位，右键/多选彻底删除。
        void renderLocalTab();

        /// 选本地文件 → 校验 → 入库。
        void openImportDialog(const VersionedProduct &p);

        /// 窗口体：三个页签 + 彻底删除确认弹窗。
        void render();

        void shutdown();
    };

    State &state();
}
