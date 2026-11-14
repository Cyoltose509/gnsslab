#pragma once

#include <fstream>
#include <iosfwd>
#include <memory>
#include <string>

#include "core/geom/EphemerisTable.h"
#include "Exception.h"
#include "core/obs/ObsStruct.h"

#include "novatel/LogReader.h"
#include "rinex/RINEXObsReader.h"

/// 观测文件的格式差异全部收在这一层：认格式、造读取器、取帧、探首历元日期。
/// 上层（解算 / 日期预读 / RTK 配对）只认 ObsStream，不再自己写 RINEX/NovAtel 分派。
namespace Observer {
    enum class Format {
        RINEX,
        NovAtel,
        Unknown
    };

    [[nodiscard]] constexpr const char *formatName(const Format f) {
        switch (f) {
            case Format::RINEX: return "RINEX";
            case Format::NovAtel: return "NovAtel";
            default: return "未知";
        }
    }

    /// 历元里是否自带星历：NovAtel 日志把 EPHEM 与 RANGE 混在一条流里，
    /// RINEX 观测必须另配广播星历文件。判「要不要 BRDC」只读这一条。
    [[nodiscard]] constexpr bool carriesEph(const Format f) { return f == Format::NovAtel; }

    struct Sniff {
        bool asciiLog = false; ///< 纯文本的 NovAtel 日志（'#' / '%' 开头的行）
        double rinexVersion = 0.0; ///< 观测类 RINEX 的版本号，其余为 0
    };

    Sniff sniffObsStream(std::istream &in);

    Format classifyObsStream(std::istream &in);

    /// 先看前几行，认不出再按 NovAtel 惯用后缀兜底。
    Format classifyObsPath(const std::string &path);

    /// 版本已知就直接造；给了流则先嗅版本再 attach（流由调用方持有）。
    std::unique_ptr<RINEXObsReader> makeRINEXReader(double version);

    std::unique_ptr<RINEXObsReader> makeRINEXReader(std::fstream &fs);

    /// 按首个非空白字符是 '#'/'%' 分文本/二进制读取器。
    std::unique_ptr<NovAtel::LogReader> makeNovAtelReader(const std::string &path);

    struct Item {
        ObsData obs;
        std::shared_ptr<EphemerisTable> eph;
    };

    struct Source {
        virtual ~Source() = default;

        virtual bool next(Item &out) = 0;
    };

    /// 观测日期是从哪儿读出来的。
    enum class DateFrom {
        Name, ///< 文件名里的 YYYYDDD / YYDDD
        Header, ///< 文件头的 TIME OF FIRST OBS
        Epoch ///< 真读了首个观测历元（NovAtel 日志、缺该头字段的旧 RINEX）
    };

    /// 观测日期：week 是 GPS 周，只有从历元算得出。
    struct ObsDate {
        int year = 0;
        int doy = 0;
        int week = 0;
        double sow = 0.0;
        DateFrom from = DateFrom::Name;
    };

    /// 一个观测文件。open 时把格式与读取器定死，之后取帧、留一帧、快进、
    /// 探首历元日期都是同一套调用；哪一套实现是 RINEX 专属的，就在实现里分一次。
    class ObsStream final : public Source {
    public:
        /// 打开并定格式。失败时 err 说明是打不开还是认不出。
        /// gloSlots 是广播星历给的 GLONASS 频率号，用于补观测头不全的槽位，可为空。
        bool open(const std::string &fpath, std::string *err = nullptr,
                  const std::map<std::string, int> *gloSlots = nullptr);

        [[nodiscard]] Format format() const { return fmt; }

        [[nodiscard]] const char *formatName() const { return Observer::formatName(fmt); }

        [[nodiscard]] bool carriesEph() const { return Observer::carriesEph(fmt); }

        /// 解析器不抛 EOF、一直在文件尾打转时的保险；只有 RINEX 会触发。
        [[nodiscard]] bool hitEpochLimit() const { return limitHit; }

        bool next(Item &out) override;

        /// 取一帧留作下一帧：RTK 要先把两只文件的起点对齐再开始配对。
        bool peek(Item &out);

        /// 跳到不早于 t 的第一帧：RINEX 的按行文本能跳，其余格式空操作。
        void fastForward(const CommonTime &t) const;

        /// 首历元的日期。RINEX 先读头的 TIME OF FIRST OBS（不解析历元），
        /// 该字段缺失或非 RINEX 时真解一帧，from 说明用的哪条。
        bool firstDate(ObsDate &out);

    private:
        bool pull(Item &out);

        Format fmt = Format::Unknown;
        std::string path;
        std::unique_ptr<RINEXObsReader> rnx;
        std::unique_ptr<NovAtel::LogReader> oem;
        int rinexEpochs = 0;
        bool limitHit = false;
        bool hasPending = false;
        Item pending;
    };

    /// 只读文件内容的首历元日期（先头后历元），文件名不参与。
    bool probeObsDate(const std::string &path, ObsDate &out);

    /// 观测文件头里的天线记录（RINEX 定宽头）；NovAtel 日志不带这些字段，返回 false。
    struct AntennaInfo {
        std::string station;
        std::string serial;
        std::string type;
        Eigen::Vector3d deltaENU = Eigen::Vector3d::Zero();
    };

    bool probeAntenna(const std::string &path, AntennaInfo &out);
}
