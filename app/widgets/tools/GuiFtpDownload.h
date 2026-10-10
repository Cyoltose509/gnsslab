#pragma once

#include "download/ProductMeta.h"

namespace GuiFtpDownload {
    extern bool s_open;

    void Render(bool *p_open);

    void RequestAutoDownload(ProductType type, int year, int doy, const std::string &datacenter);

    /// 退出前收尾：等待扫描/下载线程结束。
    /// 必须在 Application::Shutdown 里调用，否则 std::thread 成员析构时若仍 joinable 会 std::terminate。
    void Shutdown();
}
