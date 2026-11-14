#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "core/obs/ObsStruct.h"
#include "SolverOptions.h"

class SPP;

class Filter {
public:
    SolverOptions::FilterMode mode = SolverOptions::FilterMode::Forward;


    struct Feed {
        std::function<bool(std::size_t)> ensure;
        std::function<std::size_t()> count;
        std::function<const ObsData &(std::size_t)> obs;
        std::function<const CommonTime &(std::size_t)> epoch;
    };


    struct Out {
        enum class Act {
            Skip, // 本趟不解算（纯后向档的正序扫描），只留空槽位
            UseSolver, // 整份采用本趟解算器的结果
            Fuse, // 两趟位置按协方差融合，只覆盖位置类字段
            Keep, // 保留已有解
            Fail // 无解
        };

        Act act = Act::Keep;
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero(); // Fuse 的位置
        double sigma = 0.0; // Fuse 的 σ
        /// 前向趟由调用方回填本历元是否算有效解；前向+后向档据此留正序轨迹。
        bool solved = false;
    };

    using Factory = std::function<std::unique_ptr<SPP>()>;
    using Prepare = std::function<void(std::size_t index, SPP &solver)>;
    using Handler = std::function<bool(std::size_t index, SPP &solver, Out &out)>;

    /// 两趟共用同一份上下文：正序解算器、倒序解算器工厂、惰性历元序列与本趟回调。
    struct Session {
        SPP *solver = nullptr; // 正序趟用它解算；倒序趟从它搬产品
        Factory make;          // 倒序趟新建反向解算器
        Feed feed;
        Prepare prepare;       // 正序趟逐历元准备；倒序趟产品与星历已备齐，不再调用
        Handler onEpoch;       // 本趟逐历元回填
    };

    /// 正序与倒序只是遍历方向不同：都按同一上下文逐历元解算、交回调回填，
    /// 返回处理过的历元数（解算器创建失败为 0）。
    std::size_t runForward(const Session &session);

    std::size_t runBackward(const Session &session) const;

    /// 本档正序趟是否真的解算（纯后向档只扫不解，留空槽位给倒序趟填）。
    [[nodiscard]] bool solvesForward() const { return filterSolvesForward(mode); }

    /// 本档要不要跑倒序趟。
    [[nodiscard]] bool hasBackward() const { return filterHasBackward(mode); }

    void clear();

private:
    struct Trail {
        bool valid = false;
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1.0e12;
        double sigma0 = 1.0;
    };

    std::vector<Trail> mTrail;
};
