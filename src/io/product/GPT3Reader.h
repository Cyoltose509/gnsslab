#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

/// GPT3 格网
struct GPT3Values {
    double pHpa = 0.0;  // 测站高处气压(hPa)
    double tC = 0.0;    // 测站高处气温(℃)
    double dTKm = 0.0;  // 温度递减率(K/km)
    double tmK = 0.0;   // 水汽加权平均温度(K)
    double eHpa = 0.0;  // 测站高处水汽压(hPa)
    double ah = 0.0;    // 零高度干映射系数
    double aw = 0.0;    // 湿映射系数
    double la = 0.0;    // 水汽递减因子
    double unduM = 0.0; // 大地水准面起伏(m)
    double gnH = 0.0, geH = 0.0; // 干梯度北/东(m)
    double gnW = 0.0, geW = 0.0; // 湿梯度北/东(m)
};

class GPT3Reader {
public:
    static constexpr int kRows = 2592;
    static constexpr int kCols = 64;

    // 校验 GPT3 格网文件格式与完整性（下载 / 手工导入时用，不进逐历元路径）。
    // 返回 false 时 detail 写原因，返回 true 时写人读摘要。
    static bool validate(const std::string &path, std::string &detail);

    bool load(const std::string &path);

    void clear() { tab.reset(); }

    [[nodiscard]] bool empty() const { return !tab || tab->rows.empty(); }

    [[nodiscard]] bool at(double mjd, double latRad, double lonRad, double hEllM,
                          GPT3Values &v) const;

    [[nodiscard]] bool gridHeight(double latRad, double lonRad, double &hEllM) const;

private:
    static void nodeWeights(double latRad, double lonRad, int idx[4], double w[4], int &nPt) ;

    // GPT3 是版本型格网（一装几年），同一份文件只解析一次，各 solver 实例共享
    // （IoCache 按 path+mtime+size 认）。2592 行 × 64 列 = 1.3 MB，比缓存的键还便宜。
    struct Table {
        std::vector<std::array<double, kCols> > rows;
    };

    std::shared_ptr<const Table> tab;
};
