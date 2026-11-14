#pragma once

#include "core/geom/CoordStruct.h"
#include "MathUtils.h"
#include "TimeStruct.h"

#include <iosfwd>
#include <string>
#include <vector>

namespace SP3 {
    /// SP3-c 文本写出：只负责格式（列宽、单位换算、表头各段），数据由调用方拼成记录。
    /// 与 Reader 对称：读进来什么样、写出去还什么样。
    class Writer {
    public:
        struct Header {
            /// 首历元，写进 #d 行；必须与 records.front() 一致。
            CivilTime epoch;
            unsigned week = 0;
            /// 首历元周内秒与名义采样间隔（s）。SP3 要求等间隔，这个值是「标称」不是实测。
            double sow = 0.0;
            double interval = 30.0;
            /// 卫星号固定 3 字符（SP3-c）：4 字符会顶掉后面的坐标列。
            std::string satId = "L01";
            std::string marker = "gnssLab";
            /// 参考框架：写进 #cP 行，只能是 Frame::Ref 里那几个名字。
            Frame::Ref frame = Frame::Ref::IGS14;
            std::string comment = "kinematic orbit";
            /// V 行要么每历元都写、要么都不写，所以是整份的属性而不是单条记录。
            bool hasVel = false;
        };

        struct Record {
            CivilTime epoch;
            /// 位置 m、速度 m/s；写出时分别换成 km 与 dm/s。
            Eigen::Vector3d pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        };

        static void write(std::ostream &out, const Header &header, const std::vector<Record> &records);
    };
}
