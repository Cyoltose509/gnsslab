
#include <cmath>
#include <algorithm>
#include <numeric>

using namespace std;
// 物理常数
const double c = 299792458.0;       // 光速 (m/s)
const double RE = 6378137.0;        // 地球半径 (m)
const double GM = 3.986005e14;      // 地球引力常数
const double PI = 3.141592653589793;

// ====================== 辅助函数 ======================
// 1. 无电离层组合伪距计算
double calcIFCombination(double P1, double P2, double f1 = 1575.42e6, double f2 = 1227.60e6)
{
    double f1sq = f1 * f1;
    double f2sq = f2 * f2;
    return (f1sq * P1 - f2sq * P2) / (f1sq - f2sq); // 无电离层组合公式
}

// 2. 对流层延迟计算
double calcTroposphereDelay(double elev)
{
    if (elev < 5.0 * PI / 180.0) elev = 5.0 * PI / 180.0; // 高度角≥5°
    double sinE = sin(elev);
    double dry = 2.303 / sinE;    // 干延迟
    double wet = 0.12 / sinE;     // 湿延迟
    return (dry + wet) / 1000.0;  // 单位转换为m
}

// 3. 高度角计算
double calcElevation(double sx, double sy, double sz, double rx, double ry, double rz)
{
    double dx = sx - rx;
    double dy = sy - ry;
    double dz = sz - rz;
    double r = sqrt(dx * dx + dy * dy + dz * dz);

    // 地心夹角
    double cosZ = (dx * rx + dy * ry + dz * rz) / (r * sqrt(rx * rx + ry * ry + rz * rz));
    double z = acos(cosZ);
    return PI / 2.0 - z; // 高度角 = 90° - 天顶角
}

// 4. 找星历
const GPSEphem* findGpsEph(const vector<GPSEphem>& ephs, int prn)
{
    for (const auto& e : ephs) if (e.PRN == prn) return &e;
    return nullptr;
}
const BDSEphem* findBdsEph(const vector<BDSEphem>& ephs, int prn)
{
    for (const auto& e : ephs) if (e.PRN == prn) return &e;
    return nullptr;
}

// 5. 矩阵求逆
vector<vector<double>> inv4x4(const vector<vector<double>>& mat)
{
    vector<vector<double>> res(4, vector<double>(4, 0));
    double det = mat[0][0] * (mat[1][1] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) - mat[1][2] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) + mat[1][3] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]))
        - mat[0][1] * (mat[1][0] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) - mat[1][2] * (mat[2][0] * mat[3][3] - mat[2][3] * mat[3][0]) + mat[1][3] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0]))
        + mat[0][2] * (mat[1][0] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) - mat[1][1] * (mat[2][0] * mat[3][3] - mat[2][3] * mat[3][0]) + mat[1][3] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0]))
        - mat[0][3] * (mat[1][0] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]) - mat[1][1] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0]) + mat[1][2] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0]));

    if (fabs(det) < 1e-10) return res;
    double invDet = 1.0 / det;

    // 伴随矩阵
    res[0][0] = (mat[1][1] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) - mat[1][2] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) + mat[1][3] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1])) * invDet;
    res[0][1] = -(mat[0][1] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) - mat[0][2] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) + mat[0][3] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1])) * invDet;
    res[0][2] = (mat[0][1] * (mat[1][2] * mat[3][3] - mat[1][3] * mat[3][2]) - mat[0][2] * (mat[1][1] * mat[3][3] - mat[1][3] * mat[3][1]) + mat[0][3] * (mat[1][1] * mat[3][2] - mat[1][2] * mat[3][1])) * invDet;
    res[0][3] = -(mat[0][1] * (mat[1][2] * mat[2][3] - mat[1][3] * mat[2][2]) - mat[0][2] * (mat[1][1] * mat[2][3] - mat[1][3] * mat[2][1]) + mat[0][3] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])) * invDet;

    res[1][0] = -(mat[1][0] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) - mat[1][2] * (mat[2][0] * mat[3][3] - mat[2][3] * mat[3][0]) + mat[1][3] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0])) * invDet;
    res[1][1] = (mat[0][0] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) - mat[0][2] * (mat[2][0] * mat[3][3] - mat[2][3] * mat[3][0]) + mat[0][3] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0])) * invDet;
    res[1][2] = -(mat[0][0] * (mat[1][2] * mat[3][3] - mat[1][3] * mat[3][2]) - mat[0][2] * (mat[1][0] * mat[3][3] - mat[1][3] * mat[3][0]) + mat[0][3] * (mat[1][0] * mat[3][2] - mat[1][2] * mat[3][0])) * invDet;
    res[1][3] = (mat[0][0] * (mat[1][2] * mat[2][3] - mat[1][3] * mat[2][2]) - mat[0][2] * (mat[1][0] * mat[2][3] - mat[1][3] * mat[2][0]) + mat[0][3] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])) * invDet;

    res[2][0] = (mat[1][0] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) - mat[1][1] * (mat[2][0] * mat[3][3] - mat[2][3] * mat[3][0]) + mat[1][3] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0])) * invDet;
    res[2][1] = -(mat[0][0] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) - mat[0][1] * (mat[2][0] * mat[3][3] - mat[2][3] * mat[3][0]) + mat[0][3] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0])) * invDet;
    res[2][2] = (mat[0][0] * (mat[1][1] * mat[3][3] - mat[1][3] * mat[3][1]) - mat[0][1] * (mat[1][0] * mat[3][3] - mat[1][3] * mat[3][0]) + mat[0][3] * (mat[1][0] * mat[3][1] - mat[1][1] * mat[3][0])) * invDet;
    res[2][3] = -(mat[0][0] * (mat[1][1] * mat[2][3] - mat[1][3] * mat[2][1]) - mat[0][1] * (mat[1][0] * mat[2][3] - mat[1][3] * mat[2][0]) + mat[0][3] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0])) * invDet;

    res[3][0] = -(mat[1][0] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]) - mat[1][1] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0]) + mat[1][2] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0])) * invDet;
    res[3][1] = (mat[0][0] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]) - mat[0][1] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0]) + mat[0][2] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0])) * invDet;
    res[3][2] = -(mat[0][0] * (mat[1][1] * mat[3][2] - mat[1][2] * mat[3][1]) - mat[0][1] * (mat[1][0] * mat[3][2] - mat[1][2] * mat[3][0]) + mat[0][2] * (mat[1][0] * mat[3][1] - mat[1][1] * mat[3][0])) * invDet;
    res[3][3] = (mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0])) * invDet;

    return res;
}

// ====================== 单点定位主函数 ======================
SPPResult SinglePointPositioning(
    const map<int, SatObs>& obsMap,
    const vector<GPSEphem>& gpsEphs,
    const vector<BDSEphem>& bdsEphs,
    const CommenTime& ct)
{
    SPPResult res;

    // 设置初始位置（WGS84近似值，也可设为0,0,0）
    double X = 0.0, Y = 0.0, Z = 0.0;
    double dtr = 0.0; // 接收机钟差初始值

    // 迭代次数<10次，收敛条件10^-6
    const int MAX_ITER = 10;
    const double CONVERGE_THRESH = 1e-6;
    bool converged = false;

	//获取参与解算的卫星列表
    vector<int> validPrns;
    for (const auto& pair : obsMap)
    {
        validPrns.push_back(pair.first);
    }

    // 至少4颗卫星才解算
    if (validPrns.size() < 4)
    {
        res.usedSat = 0;
        return res;
    }

    // 迭代解算（最多10次）
    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        vector<vector<double>> B;    // 观测系数矩阵
        vector<double> w;            // 残差向量
        vector<double> weights;      // 权矩阵P（高度角加权）

        for (int prn : validPrns)
        {
            const SatObs& sat = obsMap.at(prn);
            double P_IF = 0.0;       // 无电离层组合伪距
            double sv_clk = 0.0;     // 卫星钟差
            double sx = 0, sy = 0, sz = 0; // 卫星位置

            // 计算信号发射时刻的卫星位置+钟差
            if (sat.sys == 0) // GPS
            {
                const GPSEphem* eph = findGpsEph(gpsEphs, prn);
                if (!eph) continue;

                // 计算信号传播时间（近似）
                double tk_approx = TimeDiff_GPS(ct, *eph);
                vector<double> sv = GPS_P(*eph, tk_approx);
                sx = sv[0], sy = sv[1], sz = sv[2];
                sv_clk = sv[6];

                // 无电离层组合
                P_IF = calcIFCombination(sat.gps_l1.psr, sat.gps_l2.psr);
            }
            else if (sat.sys == 4) // BDS
            {
                const BDSEphem* eph = findBdsEph(bdsEphs, prn);
                if (!eph) continue;

                double tk_approx = TimeDiff_BDS(ct, *eph);
                vector<double> sv = BDS_P(*eph, tk_approx);
                sx = sv[0], sy = sv[1], sz = sv[2];
                sv_clk = sv[6];

                // 无电离层组合（北斗B1I/B3I频率）
                P_IF = calcIFCombination(sat.bds_b1i.psr, sat.bds_b3i.psr, 1561.098e6, 1268.52e6);
            }

            // 修正信号发射时刻（更精确）
            double rho = sqrt(pow(sx - X, 2) + pow(sy - Y, 2) + pow(sz - Z, 2));
            double delta_t = rho / c; // 信号传播时间
            double tk = TimeDiff_GPS(ct, *findGpsEph(gpsEphs, prn)) - delta_t;

            // 重新计算发射时刻的卫星位置
            if (sat.sys == 0)
            {
                vector<double> sv = GPS_P(*findGpsEph(gpsEphs, prn), tk);
                sx = sv[0], sy = sv[1], sz = sv[2];
                sv_clk = sv[6];
            }
            else if (sat.sys == 4)
            {
                vector<double> sv = BDS_P(*findBdsEph(bdsEphs, prn), tk);
                sx = sv[0], sy = sv[1], sz = sv[2];
                sv_clk = sv[6];
            }

            // 对流层延迟
            double elev = calcElevation(sx, sy, sz, X, Y, Z);
            double T = calcTroposphereDelay(elev);

            // 线性化观测方程
            double rho_new = sqrt(pow(sx - X, 2) + pow(sy - Y, 2) + pow(sz - Z, 2));
            double dx = X - sx;
            double dy = Y - sy;
            double dz = Z - sz;

            // 构建B矩阵行：[-dx/rho, -dy/rho, -dz/rho, 1]
            B.push_back({ -dx / rho_new, -dy / rho_new, -dz / rho_new, 1.0 });

            // 构建w向量：P_IF - rho - c*sv_clk + c*dtr - T
            w.push_back(P_IF - rho_new - c * sv_clk + c * dtr - T);

            // 权矩阵：高度角加权（sin²(elev)）
            weights.push_back(pow(sin(elev), 2));
        }

        // 重构矩阵（如果卫星数不足，直接跳过）
        if (B.size() < 4) break;

        // 带权最小二乘求解 x = (B^TPB)^-1 B^TPw
        // 1. 构建B^TP
        vector<vector<double>> BtP(4, vector<double>(B.size(), 0));
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < B.size(); ++j)
                BtP[i][j] = B[j][i] * weights[j];

        // 2. 构建B^TPB
        vector<vector<double>> BtPB(4, vector<double>(4, 0));
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < B.size(); ++k)
                    BtPB[i][j] += BtP[i][k] * B[k][j];

        // 3. 构建B^TPw
        vector<double> BtPw(4, 0);
        for (int i = 0; i < 4; ++i)
            for (int k = 0; k < B.size(); ++k)
                BtPw[i] += BtP[i][k] * w[k];

        // 4. 矩阵求逆 + 求解改正数
        vector<vector<double>> invBtPB = inv4x4(BtPB);
        vector<double> dx(4, 0);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                dx[i] += invBtPB[i][j] * BtPw[j];

        // 收敛判断（改正数平方和 < 1e-6）
        double dx_sq_sum = dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2] + dx[3] * dx[3];
        if (dx_sq_sum < CONVERGE_THRESH)
        {
            converged = true;
            break;
        }

        // 更新初始位置
        X += dx[0];
        Y += dx[1];
        Z += dx[2];
        dtr += dx[3] / c; // 钟差单位转换
    }

    // 输出结果
    res.X = X;
    res.Y = Y;
    res.Z = Z;
    res.dtr = dtr;
    res.usedSat = (int)validPrns.size();
    res.PDOP = sqrt(X * X + Y * Y + Z * Z) / RE; // 简化PDOP计算
    res.isConverged = converged;  // 将迭代收敛状态赋值给结构体
    return res;
}

// ====================== 单点测速 ======================
SPVResult SinglePointVelocity(
    const map<int, SatObs>& obsMap,
    const vector<GPSEphem>& gpsEphs,
    const vector<BDSEphem>& bdsEphs,
    const CommenTime& ct,
    const SPPResult& pos)
{
    SPVResult res;
    vector<int> validPrns;

    //直接获取所有PRN
    vector<int> validPrns;
    for (const auto& pair : obsMap)
    {
        validPrns.push_back(pair.first);
    }

    if (validPrns.size() < 4) return res;

    vector<vector<double>> B;
    vector<double> w;

    for (int prn : validPrns)
    {
        const SatObs& sat = obsMap.at(prn);
        double dop_IF = 0.0; // 多普勒无电离层组合
        double sx = 0, sy = 0, sz = 0, vx = 0, vy = 0, vz = 0;

        if (sat.sys == 0)
        {
            const GPSEphem* eph = findGpsEph(gpsEphs, prn);
            if (!eph) continue;
            double tk = TimeDiff_GPS(ct, *eph);
            vector<double> sv = GPS_P(*eph, tk);
            sx = sv[0], sy = sv[1], sz = sv[2], vx = sv[3], vy = sv[4], vz = sv[5];
            dop_IF = calcIFCombination(sat.gps_l1.doppler, sat.gps_l2.doppler);
        }
        else if (sat.sys == 4)
        {
            const BDSEphem* eph = findBdsEph(bdsEphs, prn);
            if (!eph) continue;
            double tk = TimeDiff_BDS(ct, *eph);
            vector<double> sv = BDS_P(*eph, tk);
            sx = sv[0], sy = sv[1], sz = sv[2], vx = sv[3], vy = sv[4], vz = sv[5];
            dop_IF = calcIFCombination(sat.bds_b1i.doppler, sat.bds_b3i.doppler, 1561.098e6, 1268.52e6);
        }

        // 构建测速观测方程
        double rho = sqrt(pow(sx - pos.X, 2) + pow(sy - pos.Y, 2) + pow(sz - pos.Z, 2));
        double dx = pos.X - sx;
        double dy = pos.Y - sy;
        double dz = pos.Z - sz;

        B.push_back({ dx / rho, dy / rho, dz / rho, 1.0 });
        w.push_back(dop_IF * c - (vx * dx + vy * dy + vz * dz) / rho);
    }

    // 最小二乘求解速度
    vector<vector<double>> Bt(4, vector<double>(B.size(), 0));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < B.size(); ++j)
            Bt[i][j] = B[j][i];

    vector<vector<double>> BtB(4, vector<double>(4, 0));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < B.size(); ++k)
                BtB[i][j] += Bt[i][k] * B[k][j];

    vector<double> Btw(4, 0);
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < B.size(); ++k)
            Btw[i] += Bt[i][k] * w[k];

    vector<vector<double>> invBtB = inv4x4(BtB);
    vector<double> dv(4, 0);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            dv[i] += invBtB[i][j] * Btw[j];

    res.vX = dv[0];
    res.vY = dv[1];
    res.vZ = dv[2];
    res.dtr_dot = dv[3] / c;

    return res;
}