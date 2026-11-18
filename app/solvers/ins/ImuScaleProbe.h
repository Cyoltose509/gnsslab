#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "core/ins/InsStruct.h"

namespace Solvers::INS {
    /// 自动检测当量的结果。等级决定界面文字颜色：记录自描述最可信，反推是推测（建议核对），
    /// 失败就停在界面上等人手填。
    struct ScaleProbe {
        enum class Level { Pass, Warn, Fail };

        Level level = Level::Fail;
        ImuScale scale;
        std::string note;

        [[nodiscard]] bool ok() const { return level != Level::Fail; }
    };

    /// 读日志开头定当量：记录里有型号自描述就查内置表，否则用开头静置段的 |f| 反推再匹配型号表。
    ScaleProbe probeScale(const std::string &path, double gravity);

    /// 「自动检测」的状态：点一下起一个读盘线程，界面每帧收一次结论，读文件期间界面照常刷新。
    /// 结论等级与文字只由界面线程写（start 之后 poll 收回），后台线程只碰 done。
    struct ScaleDetect {
        enum class Level { None, Pass, Warn, Fail };

        Level level = Level::None;
        std::string text;

        ScaleDetect() = default;
        ScaleDetect(const ScaleDetect &) = delete;
        ScaleDetect &operator=(const ScaleDetect &) = delete;
        ~ScaleDetect();

        [[nodiscard]] bool running() const { return busy; }

        /// 起一个后台线程读文件定当量；上一轮还没落定就先等它。
        void start(const std::string &path, double gravity);

        /// 每帧调一次：后台出了结论就收回界面线程，成功则覆盖当量。
        void poll(ImuScale &scale);

    private:
        std::atomic<bool> busy{false};
        std::thread worker;
        std::mutex mtx;
        ScaleProbe done;
    };
}
