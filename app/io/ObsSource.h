#pragma once

#include <memory>
#include <string>
#include <vector>

#include "solver/rtk/DualPair.h"
#include "io/Observer.h"

namespace ObsSource {
    using Puller = ::DualPair::Puller;

    /// 一对观测文件（基准站 / 流动站）。两侧都是 ObsStream，格式在 open 时定死，
    /// 配对、是否需要广播星历都从流自己的属性读，不看文件名也不问格式。
    struct ObsPair {
        Observer::ObsStream base, rover;
        std::string err;

        bool open(const std::string &basePath, const std::string &roverPath,
                  const std::string &navPath);

        /// 任一侧的历元里不带星历（RINEX 观测）就得另配广播星历。
        [[nodiscard]] bool needsNav() const { return !base.carriesEph() || !rover.carriesEph(); }

        /// 两侧各缓一帧，把能跳的（RINEX）跳到对方起点，免得从头逐历元空转。
        bool alignStart();
    };

    /// ObsStream 拆成 DualPair 要的 (ObsData, EphPtr) 回调。
    Puller nextOf(Observer::ObsStream &s);

    std::string RTKInputError(const std::string &basePath, const std::string &roverPath,
                              const std::string &navPath);

    bool loadNav(const std::string &navPath, EphemerisTable &navTable, std::string &err);

    /// 同目录里与观测文件配套的广播星历：RINEX2 按扩展名换字母，RINEX3 按日期扫目录。
    std::vector<std::string> scanNavFiles(const std::string &obsPath);

    std::string autoDetectNav(const std::string &obsPath);
}
