#pragma once

#include <string>
#include <vector>

namespace Download {
    struct VersionedProduct {
        const char *id;       // 稳定标识
        const char *label;    // 界面显示名
        const char *indexUrl; // 远端目录
        const char *pattern;  // 在索引正文中匹配文件名的正则，第 1 组为版本号
        const char *fallback; // 正则无命中时的固定文件名
        int checkHours;       // 检查周期（小时）
        const char *exts;     // 手工导入时的文件对话框过滤
        bool (*validate)(const std::string &path, std::string &detail);
    };

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
    }
}
