#pragma once

#include <string>

namespace Download::AutoSync {
    struct Status {
        enum Phase { Idle, Checking, Downloading, Ready, Error };

        Phase phase = Idle;
        std::string version;  // 最近一次发现的远端版本
        std::string file;     // 对应文件名
        std::string detail;   // 进度 / 错误说明
        float progress = -1.f; // 0..1；<0 表示总量未知
        double gotBytes = 0;   // 已下载字节
        double totalBytes = 0; // 总字节；<=0 表示未知
    };

    void start();

    void stop();

    void setEnabled(bool on);

    bool enabled();

    void checkNow(const std::string &id = {});

    Status status(const std::string &id);

    void noteImported(const std::string &id, const std::string &version,
                      const std::string &file, const std::string &detail);
}
