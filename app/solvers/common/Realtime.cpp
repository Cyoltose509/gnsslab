#include "Realtime.h"

#include <algorithm>
#include <thread>

#include "Log.h"
#include "util/Clock.h"
#include "i18n/I18n.h"

namespace Solvers::Realtime {
    namespace {
        /// 拨号失败（服务器不可达、端口占满）后的重试间隔
        constexpr int kConnectFailBackoffMs = 2000;

        /// 断流（对端关闭或长时间无历元）后的重连间隔
        constexpr int kStreamBreakBackoffMs = 1000;
    }

    void Session::run() {
        while (!stopped()) {
            try {
                if (!connect()) {
                    if (!loggedConnectFail) {
                        LOG_WARN << "[" << tag() << I18n::tr("rt.log.connect_failed", target(),
                                                                      kConnectFailBackoffMs / 1000);
                        loggedConnectFail = true;
                    }
                    setConnectError();
                    sleep(kConnectFailBackoffMs);
                    continue;
                }
                clearError();
                if (loggedConnectFail) {
                    LOG_INFO << "[" << tag() << I18n::tr("rt.log.connected", target());
                    loggedConnectFail = false;
                }

                const bool broke = pump();
                close();
                if (broke) break;

                if (!stopped()) {
                    LOG_WARN << "[" << tag() << I18n::tr("rt.log.stream_broken",
                                                              kStreamBreakBackoffMs / 1000, target());
                    setStreamBroken();
                    sleep(kStreamBreakBackoffMs);
                }
            } catch (const std::exception &e) {
                close();
                onException(e);
                sleep(kConnectFailBackoffMs);
            } catch (...) {
                close();
                sleep(kConnectFailBackoffMs);
            }
        }
        finish();
    }

    void Session::sleep(const int ms) {
        constexpr int kSliceMs = 100;
        for (int left = ms; left > 0 && !stopped(); left -= kSliceMs)
            Clock::sleep(Clock::milliseconds(std::min(left, kSliceMs)));
    }
}
