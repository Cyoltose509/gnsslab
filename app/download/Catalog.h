#pragma once

#include <string>
#include <vector>

#include "Store.h"

namespace Download {
    /// 版本型产品的目录表、远端最新版发现、手工导入，以及后台的定期自动更新。
    /// 日期型产品不在这里，走 Remote + Store。
    namespace Catalog {
        const std::vector<VersionedProduct> &all();

        const VersionedProduct *find(const std::string &id);

        // 已知版本型产品的类型化句柄
        const VersionedProduct &atx();
        const VersionedProduct &gpt3();

        // 从远端目录索引挑出版本号最大的文件
        bool discoverLatest(const VersionedProduct &p, std::string &fileName,
                            std::string &version, std::string &err);

        // 手工导入本地已有文件
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

        UpdateStatus updateStatus(const std::string &id);

        void noteImported(const std::string &id, const std::string &version,
                          const std::string &file, const std::string &detail);
    }
}
