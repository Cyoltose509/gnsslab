#pragma once
#include "core/geom/CoordStruct.h"
#include "core/geom/Ephemeris.h"
#include "core/SatID.h"
#include "TimeStruct.h"
#include "MathUtils.h"

#include <map>
#include <string>
#include <vector>

namespace SP3 {
    /// SP3-c 精密轨道读取：按卫星分组存历元，取任意时刻时在 getPVT 里做拉格朗日插值。
    class Reader {
    public:
        static constexpr int kSp3FieldW = 14;
        static constexpr int kSp3IdLen = 4;

        static SatID parseSatId(const std::string &s) {
            if (s.size() < 3) return {};
            return {s[0], std::stoi(s.substr(1, 2))};
        }


        static bool isValid(const double v) { return std::fabs(v) < 999999.0; }


        static bool getField(const std::string &line, const int i, double &out) {
            const std::size_t pos = kSp3IdLen + kSp3FieldW * static_cast<std::size_t>(i);
            if (pos >= line.size()) return false;
            const std::string f = line.substr(pos, kSp3FieldW);
            const std::size_t a = f.find_first_not_of(' ');
            if (a == std::string::npos) return false;
            try { out = std::stod(f.substr(a)); } catch (...) { return false; }
            return !std::isnan(out);
        }

        struct Record {
            CommonTime t;
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            double clock = 0.0;
            double clockRate = 0.0;
            bool hasVel = false;
            bool hasClock = false;
            bool hasClockRate = false;
        };


        bool read(const std::string &path);
        /// append=true：把 path 里的历元并入已有 data（按卫星合并、重排、重算钟漂与相对时刻），
        /// 用于跨日弧加载相邻多日 SP3。append=false 与 read(path) 等价。
        bool read(const std::string &path, bool append);

        /// 只读 SP3 头的 ANTEX 声明（/* PCV:IGS20_2434 ... */），读不出返回空。
        /// 产品用它生成的天线模型，天线文件版本不配套会留下毫米级系统差。
        static std::string probeAntexModel(const std::string &path);

        /// 参考框架：`#cP` 行 `ORBIT <FRAME>` 段。产品在哪个框架里，解就在哪个框架里，
        /// 与真值比对时这条不能靠猜；表外的名字置 `hasFrame = false` 并按 IGS14 记。
        Frame::Ref frame = Frame::Ref::IGS14;
        bool hasFrame = false;

        [[nodiscard]] bool empty() const { return data.empty(); }


        [[nodiscard]] bool hasClock() const { return anyClock; }


        [[nodiscard]] bool contains(const SatID &sat) const {
            const auto it = data.find(sat);
            return it != data.end() && !it->second.empty();
        }


        [[nodiscard]] bool covers(const SatID &sat, const CommonTime &t) const {
            const auto it = data.find(sat);
            if (it == data.end() || it->second.empty()) return false;

            constexpr double tol = 900.0;
            return t - it->second.front().t >= -tol && it->second.back().t - t >= -tol;
        }


        [[nodiscard]] PVT getPVT(const SatID &sat, const CommonTime &t) const;


        [[nodiscard]] std::pair<CommonTime, CommonTime> timeSpan(const SatID &sat) const;

        [[nodiscard]] int satCount() const { return static_cast<int>(data.size()); }


        [[nodiscard]] const std::map<SatID, std::vector<Record> > &getData() const { return data; }

    private:
        std::map<SatID, std::vector<Record> > data;
        std::map<SatID, std::vector<double> > relTimes;
        bool anyClock = false;

        /// 合并/载入后统一重整：每星按时间排序、用相邻样本重算钟漂、重建相对时刻表。
        void finalize();
    };
}
