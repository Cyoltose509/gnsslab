#include "Session.h"

#include <chrono>
#include <thread>

#include "Log.h"

namespace Solvers::Realtime {
    void Session::run() {
        while (!stopped()) {
            try {
                if (!connect()) {
                    if (!loggedConnectFail) {
                        LOG_WARN << "[" << tag() << "] 连接失败 " << target()
                                << "，每 " << connectFailBackoffMs() / 1000 << "s 重试";
                        loggedConnectFail = true;
                    }
                    setConnectError();
                    sleep(connectFailBackoffMs());
                    continue;
                }
                clearError();
                if (loggedConnectFail) {
                    LOG_INFO << "[" << tag() << "] 已连接 " << target();
                    loggedConnectFail = false;
                }

                const bool broke = pump();
                close();
                if (broke) break;

                if (!stopped()) {
                    if (streamBreakBackoffMs() > 0)
                        LOG_WARN << "[" << tag() << "] 数据流中断，"
                                << streamBreakBackoffMs() / 1000 << "s 后重连 " << target();
                    setStreamBroken();
                    sleep(streamBreakBackoffMs());
                }
            } catch (const std::exception &e) {
                close();
                onException(e);
                sleep(connectFailBackoffMs());
            }
        }
        finish();
    }

    void Session::sleep(const int ms) {
        if (ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}
