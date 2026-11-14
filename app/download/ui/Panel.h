#pragma once

#include <set>
#include <string>
#include <vector>

#include "download/local/Store.h"

namespace Download {
    struct VersionedProduct;
}

namespace Download::Panel {
    /// 窗口里标签页的分页。
    enum class Page { Date, Obs, Leo, Ref, Aux, Local };

    /// 一页的日期与本地筛选条件。四个按日期取数的页签各持一份，
    /// 日期只在年月日与人见人爱的年积日之间转，别再各存一套。
    struct PageDate {
        int year = 0, month = 0, day = 0;
        int stateFilter = 0; // 0 全部 / 1 已下载 / 2 未下载
        char find[40] = {};

        [[nodiscard]] bool isSet() const { return year != 0; }

        /// 未设过日期时落成今天（本地时区）。
        void today();

        [[nodiscard]] int doy() const;

        /// 由年积日回填日历日期（从观测文件带日期进来时用）。
        void setFromDoy(int _year, int _doy);
    };

    /// 「产品下载」窗口的全部界面状态：各页签各管自己的日期与筛选，互不干扰。
    /// 窗口是单例，实例由 state() 给出（落 Panel.cpp），外部只拿它开/关窗口。
    struct State {
        bool open = false;
        /// 内容区当前显示哪一页：页面切换由点击或 openPage() 决定，标签栏只是跟着显示。
        Page page = Page::Date;
        /// 窗口外的 openPage() 改了 page 时置真，下一帧让标签栏强制选中对应项（ImGui 自己不会跟随）。
        bool tabSync = false;

        // 日精密产品页：日期之外还有一排产品类型勾选
        PageDate prod;
        std::vector<unsigned char> typeOn;

        PageDate obs;
        PageDate leo;
        PageDate ref;

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

        void renderDateTab();

        void renderObsTab();

        void renderLeoTab();

        void renderRefTab();

        void renderAuxTab();

        /// 浏览本地库里的日期产品与辅助产品：左键定位，右键/多选彻底删除。
        void renderLocalTab();

        /// 选本地文件 → 校验 → 入库。
        void openImportDialog(const VersionedProduct &p);

        /// 窗口体：一排页签 + 彻底删除确认弹窗。
        void render();

        void shutdown();
    };

    State &state();

    /// Earthdata 账号窗口：主菜单呼出，独立于下载窗口，界面状态落在 Panel.cpp。
    void renderEarthdata(bool *p_open);

    /// 退出时收掉账号窗口的后台连接测试。
    void shutdownEarthdata();

    /// 打开窗口并切到某页，日期一并设好（低轨观测 / 参考真值的「获取」都走这里）。
    void openPage(Page page, int year, int doy);
}
