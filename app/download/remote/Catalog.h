#pragma once

#include <string>
#include <vector>

#include "download/local/Store.h"
#include "download/remote/http/Core.h"

namespace Download {
    /// 版本型产品
    struct VersionedProduct {
        const char *id;       // 稳定标识
        const char *label;    // 界面显示名
        const char *indexUrl; // 远端目录
        const char *pattern;  // 在索引正文中匹配文件名的正则，第 1 组为版本号
        const char *fallback; // 正则无命中时的固定文件名
        int checkHours;       // 检查周期（小时）
        std::vector<std::wstring> exts;   // 手工导入时的文件对话框后缀（不含点）
        bool (*validate)(const std::string &path, std::string &detail);
    };

    /// 版本型产品的目录表
    namespace Catalog {
        constexpr auto kGPT3ID = "gpt3";
        constexpr auto kATXID = "atx";
        extern const std::vector<VersionedProduct> kProducts;

        const VersionedProduct *find(const std::string &id);


        bool discoverLatest(const VersionedProduct &p, std::string &fileName,
                            std::string &version, std::string &err);

        struct ImportResult {
            bool ok = false;
            std::string file;    // 入库后的文件名
            std::string version; // 版本号
            std::string detail;  // 成功=格式摘要；失败=原因
        };

        ImportResult importLocal(const VersionedProduct &p, const std::string &srcPath);

        /// 某个产品的自动更新状态
        struct UpdateStatus {
            enum Phase { Idle, Checking, Downloading, Ready, Error };

            Phase phase = Idle;
            std::string version;   // 最近一次发现的远端版本
            std::string file;      // 对应文件名
            std::string detail;    // 进度 / 错误说明
            float progress = -1.f; // 0..1；<0 表示总量未知
            double gotBytes = 0;   // 已下载字节
            double totalBytes = 0; // 总字节；<=0 表示未知
        };

        void startUpdates();

        void stopUpdates();

        void setUpdatesEnabled(bool on);

        bool updatesEnabled();

        void checkNow(const std::string &id = {});

        bool updateProduct(const std::string &id, std::string &detail);

        bool fetchSiteTruth(const std::string &site, int year, int doy, std::string &block,
                            std::string &detail, const Http::ProgressCb &progress = {});

        bool ensureSiteTruth(const std::string &site, int year, int doy, double xyz[3], bool &fromCache,
                             std::string &detail, const Http::ProgressCb &progress = {});

        UpdateStatus updateStatus(const std::string &id);

        void noteImported(const std::string &id, const std::string &version,
                          const std::string &file, const std::string &detail);
    }
}
