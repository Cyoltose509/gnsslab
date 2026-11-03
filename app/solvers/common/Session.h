#pragma once

#include <chrono>
#include <exception>
#include <string>
#include <vector>

#include "QCProcessor.h"
#include "Observer.h"
#include "Config.h"

namespace Solvers::Realtime {
    /// 实时会话骨架：连接失败退避重试 → pump 消费 → 断流重连，停止位与错误上报由派生类给出。
    class Session {
    public:
        virtual ~Session() = default;

        void run();

    protected:
        virtual const char *tag() const noexcept = 0;
        virtual std::string target() const = 0;

        virtual bool connect() = 0;

        virtual void close() noexcept = 0;

        virtual bool pump() = 0;

        virtual bool stopped() = 0;

        virtual void clearError() = 0;

        virtual void setConnectError() = 0;

        virtual void setStreamBroken() {
        }
        virtual void onException(const std::exception &e) = 0;

        virtual void finish() = 0;

        virtual int connectFailBackoffMs() const noexcept { return 2000; }
        virtual int streamBreakBackoffMs() const noexcept { return 0; }

    private:
        static void sleep(int ms);

        bool loggedConnectFail = false;
    };

    /// 质量分析输入的滑动窗口（7200 历元）。
    inline void pushQcWindow(std::vector<QC::QCObsEpoch> &dst, const ObsData &epoch, const double sow) {
        constexpr std::size_t kQcWindow = 7200;
        dst.push_back(QC::makeQCObsEpoch(epoch, sow));
        if (dst.size() > kQcWindow)
            dst.erase(dst.begin(), dst.begin() + static_cast<std::ptrdiff_t>(dst.size() - kQcWindow));
    }
}
