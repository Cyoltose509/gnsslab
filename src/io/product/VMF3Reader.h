#pragma once

#include <string>
#include <vector>

class VMF3Reader {
public:
    static constexpr int kLat = 36;
    static constexpr int kLon = 72;
    static constexpr int kCell = kLat * kLon;

    /// 一个 6 小时时次的一层格网
    struct Epoch {
        double mjd = 0.0;
        std::vector<double> ah;  // kCell，行优先：纬度降序、经度升序
        std::vector<double> aw;
        std::vector<double> zhd; // 格点地形高处的天顶干延迟(m)
        std::vector<double> zwd; // 格点地形高处的天顶湿延迟(m)
    };

    bool loadFile(const std::string &path);
    bool loadFiles(const std::vector<std::string> &paths);

    void clear() { epochs.clear(); }

    [[nodiscard]] bool empty() const { return epochs.empty(); }
    [[nodiscard]] int epochCount() const { return static_cast<int>(epochs.size()); }

    /// 双线性（格网）+ 线性（时次）插值出 ah/aw
    [[nodiscard]] bool coefficients(double mjd, double latRad, double lonRad,
                                    double &ah, double &aw) const;

    /// 同 coefficients，但插值出格点地形高处的天顶干/湿延迟(m)
    [[nodiscard]] bool zenithDelays(double mjd, double latRad, double lonRad,
                                    double &zhd, double &zwd) const;

private:
    static void bilinear(double latDeg, double lonDeg, int idx[4], double w[4]);

    std::vector<Epoch> epochs;
};
