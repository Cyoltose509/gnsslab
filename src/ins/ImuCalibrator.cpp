#include "ImuCalibrator.h"
#include "solver/SolverLSQ.h"



Eigen::Vector3d ImuCalibrator::posForce(const Pos p, const double g) {
    switch (p) {
        case XUP: return { g, 0, 0};
        case XDN: return {-g, 0, 0};
        case YUP: return { 0, g, 0};
        case YDN: return { 0,-g, 0};
        case ZUP: return { 0, 0, g};
        case ZDN: return { 0, 0,-g};
    }
    return Eigen::Vector3d::Zero();
}

void ImuCalibrator::rawMean(const ImuLog &log, Eigen::Vector3d &acc, Eigen::Vector3d &gyr) {
    acc.setZero(); gyr.setZero();
    if (log.empty()) return;
    for (const auto &s : log) {
        acc += Eigen::Vector3d(s.ax, s.ay, s.az);
        gyr += Eigen::Vector3d(s.gx, s.gy, s.gz);
    }
    const auto n = static_cast<double>(log.size());
    acc /= n; gyr /= n;
}

double ImuCalibrator::medianDt(const ImuLog &log, const double fallback) {
    if (log.size() < 3) return fallback;
    std::vector<double> d;
    d.reserve(log.size() - 1);
    for (size_t i = 1; i < log.size(); ++i) d.push_back(log[i].tow - log[i - 1].tow);
    std::sort(d.begin(), d.end());
    return d[d.size() / 2];
}

void ImuCalibrator::solveLinear(const std::vector<Eigen::Vector3d> &raw, const std::vector<Eigen::Vector3d> &f,
                                const std::vector<double> &w,
                                Eigen::Matrix3d &M, Eigen::Vector3d &b, double &rmsResid,
                                Eigen::Vector3d *scaleSigma) {
    M.setZero();
    b.setZero();
    rmsResid = 0.0;
    if (scaleSigma) scaleSigma->setZero();
    const size_t n = raw.size();
    if (n == 0 || f.size() != n || w.size() != n) return;

    // 逐输出轴解一个 4 参数加权最小二乘，直接复用 GNSS 解算链的 SolverLSQ
    // （同一数学：N=HᵀWH、b=HᵀWy、LDLT 分解），ins 侧不再另写一套正规方程：
    //   raw_i(k) ≈ M(k,0)·f_i(0) + M(k,1)·f_i(1) + M(k,2)·f_i(2) + b(k)
    // 未知数借用既有 Parameter 枚举作内部标签（不对外暴露，仅本函数可见）：
    // dX/dY/dZ = 该轴对 X/Y/Z 方向比力的灵敏度（比例因子矩阵的一行），cdt = 常数项（零偏），
    // 与伪距方程「dX·dx + dY·dy + dZ·dz + cdt」的结构同构。每轴独立求解 ⇒ 每个 Parameter
    // 在各自 varSet 中恰出现一次，getSolution 按类型取回无歧义。
    static const std::string kStation = "imu_calib"; // EquSys::station 是 string_view，须指向长活对象
    const Variable vSx(kStation, Parameter::dX), vSy(kStation, Parameter::dY),
                   vSz(kStation, Parameter::dZ), vB(kStation, Parameter::cdt);
    SolverLSQ lsq;
    for (int k = 0; k < 3; ++k) {
        EquSys equ;
        equ.station = kStation;
        equ.varSet = {vSx, vSy, vSz, vB};
        for (size_t i = 0; i < n; ++i) {
            if (w[i] <= 0.0) continue;
            EquData d;
            d.prefit = raw[i](k);
            d.weight = w[i];
            d.varCoeffData = {{vSx, f[i](0)}, {vSy, f[i](1)}, {vSz, f[i](2)}, {vB, 1.0}};
            equ.obsEquData[EquID(SatID('P', static_cast<int>(i)), "acc")] = d; // 'P'+段序号仅作 EquSys 内键
        }
        lsq.solve(equ);
        M(k, 0) = lsq.getSolution(Parameter::dX);
        M(k, 1) = lsq.getSolution(Parameter::dY);
        M(k, 2) = lsq.getSolution(Parameter::dZ);
        b(k) = lsq.getSolution(Parameter::cdt);
        // 比例因子 1σ：varSet 顺序 {dX,dY,dZ,cdt}，该轴比例因子是列 0。
        // covMatrix = N⁻¹（单位权重），乘后验方差因子 sigma0² 得参数协方差。
        if (scaleSigma && lsq.covMatrix.rows() == 4) {
            (*scaleSigma)(k) = std::sqrt(std::max(0.0, lsq.covMatrix(0, 0))) * lsq.sigma0;
        }
    }
    double ss = 0.0;
    double wtot = 0.0;
    for (size_t i = 0; i < n; ++i) wtot += w[i];
    for (int k = 0; k < 3; ++k) {
        for (size_t i = 0; i < n; ++i) {
            if (w[i] <= 0.0) continue;
            const double pred = f[i](0) * M(k, 0) + f[i](1) * M(k, 1) + f[i](2) * M(k, 2) + b(k);
            ss += w[i] * (raw[i](k) - pred) * (raw[i](k) - pred);
        }
    }
    rmsResid = std::sqrt(ss / (3.0 * wtot));
}


namespace {
    const char *kPosName[6] = {"X上", "X下", "Y上", "Y下", "Z上", "Z下"};
    const char *kRotName[6] = {"X正转", "X逆转", "Y正转", "Y逆转", "Z正转", "Z逆转"};
} 

// 完整标定（可变段数）。覆盖度不足时 ok=false 并填 errorMsg，不解出奇异矩阵。
CalibReport ImuCalibrator::calibrate(const CalibSegments &seg, double turnAngle, double g) {
    CalibReport r;
    r.ok = false;

    int nPos[6] = {0, 0, 0, 0, 0, 0};
    int nRot[6] = {0, 0, 0, 0, 0, 0};
    for (const auto &[fst, snd] : seg.pos) nPos[static_cast<int>(fst)]++;
    for (const auto &[fst, snd] : seg.rot) nRot[static_cast<int>(fst)]++;

    std::string miss;
    for (int p = 0; p < 6; ++p)
        if (nPos[p] == 0) miss += std::string(" 位置") + kPosName[p];
    for (int ax = 0; ax < 3; ++ax) {
        if (nRot[2 * ax] == 0) miss += std::string(" 旋转") + kRotName[2 * ax];
        if (nRot[2 * ax + 1] == 0) miss += std::string(" 旋转") + kRotName[2 * ax + 1];
    }
    if (!miss.empty()) {
        r.errorMsg = "覆盖不足，缺少：" + miss;
        return r;
    }

    // ---- 加表：堆叠所有静置段做最小二乘，行权重 = 段样本数 ----
    std::vector<Eigen::Vector3d> raw, f;
    std::vector<double> w;
    raw.reserve(seg.pos.size());
    f.reserve(seg.pos.size());
    w.reserve(seg.pos.size());
    for (const auto &[fst, samples] : seg.pos) {
        Eigen::Vector3d acc, gyr;
        rawMean(samples, acc, gyr);
        raw.push_back(acc);
        f.push_back(posForce(fst, g));
        w.push_back(static_cast<double>(samples.size()));
        r.nPos[static_cast<int>(fst)] += static_cast<int>(samples.size());
    }
    solveLinear(raw, f, w, r.calib.accelM, r.calib.accelB, r.accelRms, &r.accelScaleSigma);
    r.accelScaleCounts = r.calib.accelM.diagonal();
    r.accelScaleG = r.accelScaleCounts * g;

    // ---- 陀螺零偏：两位置法 b = (ω_上 + ω_下)/2，按轴从静置段均值取 ----
    // 同位置多段时按段样本数加权池化（= 逆方差加权）：短段的噪声均值不会因「等权平均」被放大，
    // 长段自然主导。故无需设「最短静置段」阈值——短段只管按自己的信息量贡献，换 IMU/采样率都自适应。
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();
    for (int ax = 0; ax < 3; ++ax) {
        double sumUp = 0.0, sumDn = 0.0, wUp = 0.0, wDn = 0.0;
        int nUp = 0, nDn = 0;
        for (const auto &[p, samples] : seg.pos) {
            Eigen::Vector3d acc, gyr;
            rawMean(samples, acc, gyr);
            const auto wt = static_cast<double>(samples.size());
            if (p == 2 * ax) { sumUp += gyr(ax) * wt; wUp += wt; ++nUp; }
            else if (p == 2 * ax + 1) { sumDn += gyr(ax) * wt; wDn += wt; ++nDn; }
        }
        if (wUp > 0.0 && wDn > 0.0)
            gyroBias(ax) = 0.5 * (sumUp / wUp + sumDn / wDn);
    }

    // ---- 陀螺比例因子：角位置法 K_eff = (Σ正 - Σ反)/(α·N段)，各旋转段扣零偏后累加 ----
    // 约定每个旋转段转过一整圈（±turnAngle）。同一方向出现多段（重复实验、分段录制）时
    // 必须按段数归一化：否则「两段正转」的计数和会被当成「一段转了两圈」，K 直接翻倍
    // （合成压力测试 S16：重复两遍时旧公式 gyroKeff 恰好 +100%）。
    for (int ax = 0; ax < 3; ++ax) {
        const double b = gyroBias(ax);
        double s1 = 0.0, s2 = 0.0;
        int nSeg = 0;
        for (const auto &[rr, samples] : seg.rot) {
            if (rr != 2 * ax && rr != 2 * ax + 1) continue;
            ++nSeg;
            for (const auto &s : samples) {
                const int32_t v[3] = {s.gx, s.gy, s.gz};
                const double val = v[ax] - b;
                if (rr == 2 * ax) s1 += val;
                else s2 += val;
            }
            r.nRot[rr] += static_cast<int>(samples.size());
        }
        r.gyroKeff(ax) = nSeg > 0 ? (s1 - s2) / (turnAngle * nSeg) : 0.0;
    }
    r.calib.gyroBias = gyroBias;

    r.gyroKmean = r.gyroKeff.mean();
    if (r.gyroKmean != 0.0) r.gyroDeltaS = r.gyroKeff / r.gyroKmean - Eigen::Vector3d::Ones();
    r.calib.gyroScale = r.gyroKeff * r.dt;

    // dt：优先取首个静置段，否则取首个旋转段
    if (!seg.pos.empty()) r.dt = medianDt(seg.pos.front().second);
    else if (!seg.rot.empty()) r.dt = medianDt(seg.rot.front().second);

    r.ok = true;
    return r;
}


