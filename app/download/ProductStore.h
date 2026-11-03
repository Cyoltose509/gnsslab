#pragma once

#include <optional>
#include <string>

#include "ProductCatalog.h"

namespace Download {
    struct InstalledProduct {
        std::string id;
        std::string version; // 版本号（ATX 用 IGS 文件名的 4 位序号）：只用于判「是否最新」，不当目录名
        std::string file;    // 文件名
        std::string path;    // 绝对路径
        long long size = 0;
        std::string fetchedAt; // yyyy-mm-dd hh:mm:ss
    };

    namespace Store {
        std::string root();

        std::string dir(const std::string &id);

        std::optional<InstalledProduct> active(const std::string &id);

        std::string activeFile(const std::string &id);

        std::optional<InstalledProduct> active(const VersionedProduct &product);

        std::string activeFile(const VersionedProduct &product);

        std::string tempPath(const std::string &id, const std::string &name);

        bool ensureDir(const std::string &id);

        void clearStaging(const std::string &id);

        void clearStagingExcept(const std::string &id, const std::string &keepName);

        bool install(const std::string &id, const std::string &srcPath,
                     const std::string &version, const std::string &fileName,
                     bool removeSource = true);
    }
}
