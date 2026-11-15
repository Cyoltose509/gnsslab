#pragma once

#include <set>
#include <string>
#include <vector>

#include "download/local/ProductType.h"
#include "i18n/I18n.h"

namespace Download::SourceTable {
    /// 一条数据源。同一个远端文件只有这一份描述：下载页按它列目录/直取，
    /// 解算页的「获取」也按它选源，两处不再各写一张表。
    struct Dir {
        std::string url;
        ProductType type = ProductType::Unknown;
        /// 非空＝不列目录，只按这个名字直取（名字已定，如 Swarm 的日观测、COSMIC 的 leoOrb 包）
        std::string probeName;
        /// 非空＝列目录后只留以此开头的文件
        std::string namePrefix;
        std::string backupUrl;
        /// 该源属于哪个低轨任务；空＝不限任务（IGS 周目录、SINEX 这类谁都能用）
        std::string mission;
        /// 该源的星（A/B/C）；0＝不限
        char sat = 0;
        /// 「获取」自动取真值走这条；下载页仍把同任务的全部源列出来供手选
        bool autoPick = false;
        /// 这一路取失败时最多试几次。带镜像的源（走 CDDIS 的 Earthdata 认证）链路脆，给 3
        int attempts = 1;
    };

    /// 一页的取数条件
    struct Query {
        std::set<ProductType> types;
        int year = 0;
        int doy = 0;
        const char *scanTextKey = "dl.status.reading_dir";
    };

    /// 日精密产品页的全部来源
    std::vector<Dir> daily(const Query &q);

    /// IGS 系的日产品目录
    std::vector<Dir> IGS(const std::set<ProductType> &types, int year, int doy);

    /// 基站坐标 SINEX 的周目录
    Dir snxWeekDir(int week);

    /// 按日取低轨星载观测
    std::vector<Dir> LEOObs(int year, int doy);

    /// 按日取参考真值：低轨轨道（带任务/星标记）与基站坐标混在一页里
    std::vector<Dir> refTruth(int year, int doy);

    /// AIUB 的 CODE 年度目录
    std::vector<Dir> codeDir(const std::set<ProductType> &types, int year);

    /// 从观测文件名认低轨任务
    std::string LEOMission(const std::string &obsName);

    /// Swarm 的星号（A/B/C）：观测名 GPSx_RO 里取 x，合并名按头里的 SWARM x 取；认不出回 0
    char SwarmSat(const std::string &obsName);

    /// 该观测的星号：Swarm A/B/C、Sentinel-6 A/B，其余回 0
    char LEOSat(const std::string &obsName);

    /// 本地轨道文件是不是这份观测的（任务与星都对上）
    bool orbitIsFor(const std::string &mission, char sat, const std::string &fileName);

    /// 「获取」自动取真值用的源：refTruth 里与该观测任务/星对上、且标了 autoPick 的那几条
    std::vector<Dir> LEOTruths(const std::string &obsName, int year, int doy);
}
