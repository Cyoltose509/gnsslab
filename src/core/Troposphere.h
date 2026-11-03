#pragma once

#include "Const.h"
#include <string>

namespace Troposphere {
    enum class Map {
        Hopfield,
        NMF,
        GMF,
        VMF3,
        GPT3
    };

    /// 模型元信息：key 用于配置读写，name 用于界面显示
    struct Model {
        Map id;
        const char *key;
        const char *name;
    };

    /// Hopfield 模型：线性大气分层，天顶延迟与映射都由高度角直接给出
    class Hopfield {
    public:
        static constexpr auto kId = Map::Hopfield;
        static constexpr auto kKey = "hopfield";
        static constexpr auto kName = "Hopfield";

        /// 天顶干延迟(m)
        static double zhd(double H);

        /// 天顶湿延迟(m)
        static double zwd(double H);

        /// 干映射
        static double mapH(double E);

        /// 湿映射
        static double mapW(double E);

        /// 总斜路径延迟(m)
        static double delay(double H, double E);
    };

    /// Saastamoinen 模型：只给天顶延迟，映射由其他模型提供
    class Saastamoinen {
    public:
        /// 天顶总延迟(m)
        static double zenith(double latRad, double hgtM, double humi);

        /// 天顶干延迟(m)，presHpa <= 0 时取标准大气
        static double zhd(double latRad, double hgtM, double presHpa = 0.0);

        /// 由天顶干延迟反解气压(hPa)
        static double zhdToPressure(double zhdM, double latRad, double hgtM);

        /// 标准大气气压(hPa)
        static double stdPressure(double hgtM);
    };

    class NMF {
    public:
        static constexpr auto kId = Map::NMF;
        static constexpr auto kKey = "nmf";
        static constexpr auto kName = "NMF (Niell 1996)";

        /// 干映射；mapfw 非空时回填湿映射
        static double mapping(double elRad, double latDeg, double hgtM, double doy, double *mapfw = nullptr);
    };

    class GMF {
    public:
        static constexpr auto kId = Map::GMF;
        static constexpr auto kKey = "gmf";
        static constexpr auto kName = "GMF (Boehm 2006)";

        /// 球谐展开求干/湿映射
        static void mapping(double latRad, double lonRad, double hgtM,
                            double mjd, double elevRad, double *mh, double *mw);
    };

    class VMF3 {
    public:
        static constexpr auto kId = Map::VMF3;
        static constexpr auto kKey = "vmf3";
        static constexpr auto kName = "VMF3";

        struct Coef {
            double ah = 0.0, aw = 0.0;
            double bh = 0.0, bw = 0.0;
            double ch = 0.0, cw = 0.0;
        };

        static void coefBC(double latRad, double lonRad, double doy,
                           double &bh, double &bw, double &ch, double &cw);

        /// 组装一套完整的 VMF3 系数
        static Coef makeCoef(double ah, double aw, double latRad, double lonRad, double doy);

        /// 干映射
        static double mapH(const Coef &c, double elevRad, double hgtM);

        /// 湿映射
        static double mapW(const Coef &c, double elevRad);
    };

    /// GPT3 只提供气象参数与 ah/aw，映射本体仍走 VMF3
    class GPT3 {
    public:
        static constexpr auto kId = Map::GPT3;
        static constexpr auto kKey = "gpt3";
        static constexpr auto kName = "GPT3";
    };

    /// 全部模型，按枚举值顺序排列，界面下拉框直接取 name
    inline constexpr Model kModels[] = {
        {Hopfield::kId, Hopfield::kKey, Hopfield::kName},
        {NMF::kId, NMF::kKey, NMF::kName},
        {GMF::kId, GMF::kKey, GMF::kName},
        {VMF3::kId, VMF3::kKey, VMF3::kName},
        {GPT3::kId, GPT3::kKey, GPT3::kName},
    };
    inline constexpr int kModelCount = std::size(kModels);

    /// 配置文件里用的名字
    const char *keyOf(Map m);

    /// 界面显示名
    const char *nameOf(Map m);

    Map fromKey(const std::string &key);

    /// 湿延迟先验(m)，各模型共用
    double zwdPrior(double latDeg, double hgtM, double doy);

    /// 干/湿映射函数统一入口
    double mapping(Map m, double elevRad, double latRad, double lonRad, double hgtM,
                   double doy, double mjd, double *mapfw = nullptr);
}
