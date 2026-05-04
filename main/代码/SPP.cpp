#include<iostream>
using namespace std;
#include "Decode.h"
#include "Print.h"
#include "coord_Sate_postion.h"
#include "operaVM.h"
#include<cmath>
#include<limits>


//构建结果类
struct Sppresult
{
    bool success;
    double x, y, z;
    double dt;             // 接收机钟差 (秒)
    int used_sats;          // 使用的卫星数
    double pdop;            // 位置精度因子
    double sigma0;          // 验后单位权中误差 (m)
    double B, L, H;     // 纬度、经度(度)、椭球高(米)
};
//构建SPP解算类
class SppSolver_single
{
public:
    //参数
    const double C_LIGHT = 299792458.0;
    const double OMEGA_E = 7.2921151467e-5;   // 地球自转角速度 (rad/s)
    const double M_PI = 3.14159265358979323846;

    // 当前估计状态
    double x0_, y0_, z0_, dt0_;
    SppSolver_single()
    {
        x0_ = 0.0;
        y0_ = 0.0;
        z0_ = 0.0;
        dt0_ = 0.0;
    }
    // 设置初始位置和钟差（用于加快收敛）
    void setInitialState(double x, double y, double z, double dt = 0.0)
    {
        x0_ = x;
        y0_ = y;
        z0_ = z;
        dt0_ = dt;
    }
    // 获取卫星在信号发射时刻的位置和钟差（含地球自转改正）
    bool getSatStateAtEmission(const SatObservation& sat,
        int rcvWeek, double rcvTow,
        double xr, double yr, double zr,
        double& xs, double& ys, double& zs, double& svClock,
        const unordered_map<int, GpsEphemeris>& gpsMap)
    {
        if (sat.system != 0) return false; // 仅支持GPS
        auto it = gpsMap.find(sat.prn);
        if (it == gpsMap.end()) return false;

        double tau = 0.0;
        CoordinateCalculator calc;

        for(int i=0;i<3;i++)
        {
            // 发射时刻 = 接收时刻 - 传播时间
            double t_T = rcvTow - tau;
            int week = rcvWeek;
            if (t_T < 0) 
            {
                t_T += 604800;
                week--; 
            }
            else if (t_T >= 604800) 
            {
                t_T -= 604800; 
                week++; 
            }

            // 计算卫星位置和钟差
            calc.calculateGpsPosition(it->second, week, t_T, xs, ys, zs, &svClock);

            // 地球自转改正（Sagnac效应）
            double omega_tau = OMEGA_E * tau;
            double x_rot = xs * cos(omega_tau) + ys * sin(omega_tau);
            double y_rot = -xs * sin(omega_tau) + ys * cos(omega_tau);
            xs = x_rot; ys = y_rot; // z 不变

            // 计算新的传播时间
            double dx = xs - xr;
            double dy = ys - yr;
            double dz = zs - zr;
            double new_tau = sqrt(dx * dx + dy * dy + dz * dz) / C_LIGHT;

            if (fabs(new_tau - tau) < 1e-9) break;
            tau = new_tau;
        }
        return true;
    }

    // 最小二乘迭代求解
    bool leastSquares(vector<SatObservation>& sats,
        int rcvWeek, double rcvTow,
        double& xr, double& yr, double& zr,
        double& dt,const unordered_map<int, GpsEphemeris> gpsMap,
        int& usedSats, double& pdop, double& sigma0)
    {
        int n = sats.size();
        if (n < 4)
        {
            return false;
        }
        double c = C_LIGHT;//光速

        double x = xr, y = yr, z = zr, cdt = c * dt;
        //存储每颗卫星的发射时刻（初始为接收时刻）
        vector<double> t_T_tow(n, rcvTow);
        vector<int> t_T_week(n, rcvWeek);

        vector<double> satX(n), satY(n), satZ(n), svClock(n);
        Matrix HtH_inv; // 保存最后一次的逆矩阵

        double B, L, h;
        ecefToGeodetic(x, y, z, B, L, h);
        double tropo = 0;
        double iono = 0;
        double elev;
        for (int iter = 0; iter < 10; ++iter)
        {
            // 1. 计算卫星位置和钟差
            
            for (int i = 0; i < n; ++i)
            {
                if (!getSatStateAtEmission(sats[i], t_T_week[i], t_T_tow[i],
                    x, y, z,
                    satX[i], satY[i], satZ[i], svClock[i],
                    gpsMap))
                    return false;
            }

            // 2. 构建设计矩阵 H (n×4) 和残差向量 y (n×1)
            Matrix H(n, vector<double>(4, 0.0));
            vector<double> y_vec(n, 0.0);

            for (int i = 0; i < n; ++i)
            {
                double dx = satX[i] - x;
                double dy = satY[i] - y;
                double dz = satZ[i] - z;
                double range = sqrt(dx * dx + dy * dy + dz * dz);
                double elev = atan2(dz, sqrt(dx * dx + dy * dy));   // 高度角（弧度）
                tropo = tropoCorrection(B, h, elev);
                iono=ionoCorrection(elev);
                //cout << tropo << endl;

                H[i][0] = -dx / range;
                H[i][1] = -dy / range;
                H[i][2] = -dz / range;
                H[i][3] = 1.0;

                double pr_corrected = sats[i].pseudoRange1 + c * svClock[i];
                y_vec[i] = pr_corrected - range - cdt - tropo - iono;
            }

            // 3. 求解法方程: (H^T H) * dx = H^T y
            Matrix Ht;
            matrix_T(H, Ht);                 // Ht = H^T (4×n)
            Matrix HtH;                      // 4×4
            matrix_multiply(Ht, H, HtH);     // HtH = H^T * H

            // 计算 Ht_y = H^T * y (4×1)
            vector<double> Ht_y(4, 0.0);
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < n; ++j)
                {
                    Ht_y[i] += Ht[i][j] * y_vec[j];
                }
            }

            if (!matrix_inverse(HtH, HtH_inv))
            {
                return false;   // 奇异矩阵
            }

            // 计算改正数 dx = HtH_inv * Ht_y
            vector<double> dx(4, 0.0);
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    dx[i] += HtH_inv[i][j] * Ht_y[j];
                }
            }

            // 更新状态
            x += dx[0];
            y += dx[1];
            z += dx[2];
            cdt += dx[3];
            dt = cdt / c;
            // 更新发射时刻
            for (int i = 0; i < n; ++i)
            {
                double dx_ = satX[i] - x;
                double dy_ = satY[i] - y;
                double dz_ = satZ[i] - z;
                double range = sqrt(dx_ * dx_ + dy_ * dy_ + dz_ * dz_);
                double tau = range / c;
                double t_T = rcvTow - tau;
                int week = rcvWeek;
                if (t_T < 0) 
                {
                    t_T += 604800; week--; 
                }
                else if (t_T >= 604800) 
                {
                    t_T -= 604800; week++; 
                }
                t_T_week[i] = week;
                t_T_tow[i] = t_T;
            }
            ecefToGeodetic(x, y, z, B, L, h);
            
            // 检查收敛
            double pos_change = sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);
            if (pos_change < 1e-4) break;
        }
        // 计算 sigma0 和 pdop
        double sum_v2 = 0.0;
        for (int i = 0; i < n; ++i)
        {
            double dx = satX[i] - x;
            double dy = satY[i] - y;
            double dz = satZ[i] - z;
            double range = sqrt(dx * dx + dy * dy + dz * dz);
            double elev = atan2(dz, sqrt(dx * dx + dy * dy));   // 高度角（弧度）
            tropo = tropoCorrection(B, h, elev);
            iono = ionoCorrection(elev);

            double pr_corrected = sats[i].pseudoRange1 + c * svClock[i];
            double v = pr_corrected - range - cdt - tropo - iono;
            sum_v2 += v * v;
        }
        sigma0 = sqrt(sum_v2 / (n - 4));
        pdop = sqrt(HtH_inv[0][0] + HtH_inv[1][1] + HtH_inv[2][2]);

        // 输出结果
        xr = x; yr = y; zr = z;
        usedSats = n;
        return true;
    }

    // ECEF 转经纬高
    void ecefToGeodetic(double x, double y, double z, double& B, double& L, double& H)
    {
        double a = 6378137.0;
        double f = 1.0 / 298.257223563;
        double e2 = 2 * f - f * f;
        L = atan2(y, x);
        double p = sqrt(x * x + y * y);
        double theta = atan2(z * a, p * (a * (1 - e2)));
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        B = atan2(z + e2 * a * sin_theta * sin_theta * sin_theta,
            p - e2 * a * cos_theta * cos_theta * cos_theta);
        double sin_lat = sin(B);
        double N = a / sqrt(1 - e2 * sin_lat * sin_lat);
        H = p / cos(B) - N;
        B = B * 180.0 / M_PI;
        L = L * 180.0 / M_PI;
    }

    // 主解算函数
    bool solve(const EpochData& epoch, unordered_map<int, GpsEphemeris>& gpsMap,
        Sppresult& result)
    {
        // 筛选GPS卫星（系统标识为0）
        vector<SatObservation> gpsSats;
        for (const auto& sat : epoch.sats) 
        {
            if (sat.system == 0 && sat.cn0 >= 30.0) 
            { // 简单质量筛选
                if (gpsMap.find(sat.prn) != gpsMap.end()) 
                {
                    gpsSats.push_back(sat);
                }
            }
        }

        if (gpsSats.size() < 4) return false;

        double xr = x0_, yr = y0_, zr = z0_;
        double dt = dt0_;
        int usedSats;
        double pdop, sigma0;

        bool ok = leastSquares(gpsSats, epoch.week, epoch.tow,
            xr, yr, zr, dt,
            gpsMap, usedSats, pdop, sigma0);
        if (!ok) return false;

        result.success = true;
        result.x = xr; result.y = yr; result.z = zr;
        result.dt = dt;
        result.used_sats = usedSats;
        result.pdop = pdop;
        result.sigma0 = sigma0;
        ecefToGeodetic(xr, yr, zr, result.B, result.L, result.H);

        // 更新初始状态，为下一历元提供初值
        x0_ = xr; y0_ = yr; z0_ = zr;
        dt0_ = dt;

        return true;
    }
    //对流层误差改正: Hopefield模型
    double tropoCorrection(double B, double H, double elev)
    {
        // 高度角限制（只对大于 10 度的卫星进行改正）
        double min_elev = 10.0 * M_PI / 180.0;
        if (elev < min_elev) return 0.0;
        

        // 标准大气参数
        double H0 = 0.0;
        double T0 = 15 + 273.16;
        double P0 = 1013.25;
        double RH0 = 0.5;

        double B_rad = B * M_PI / 180.0;
        double P = P0 * pow(1 - 0.0000226 * H, 5.2561);
        double T = T0 - 0.0065 * (H - H0);
        // 饱和水汽压 (hPa) - 使用 Tetens 公式
        double es = 6.112 * exp(17.67 * (T - 273.15) / (T - 29.65));
        double e = RH0 * es; // 水汽压，单位Pa
        // Hopefield 模型参数
        
        // 天顶干延迟 (米)
        double zh_d = 0.002277 * P / (1 - 0.00266 * cos(2 * B_rad) - 0.00028 * H / 1000.0);
        // 天顶湿延迟 (米)
        double zh_w = 0.002277 * (1255.0 / T + 0.05) * e / (1 - 0.00266 * cos(2 * B_rad) - 0.00028 * H / 1000.0);
        double zh = zh_d + zh_w;

        double mf = 1 / sin(elev);
        double tropo = zh * mf;
        // 防止异常值
        if (isnan(tropo) || isinf(tropo) || tropo > 100.0) tropo = 0.0;
        return tropo;
    }
    // 经验电离层改正（天顶延迟取 5 米，可根据观测时段调整）
    double ionoCorrection(double elev) 
    {
        const double zenith_iono = 5.0;   // 天顶电离层延迟（米），白天可设 5~10，夜晚 0~2
        double mf = 1.0 / sin(elev);
        return zenith_iono * mf;
    }
};

class SppSolver_BDS_IF
{
public:
    const double C_LIGHT = 299792458.0;
    const double OMEGA_E = 7.2921150e-5;        // 北斗 CGCS2000 采用的地球自转角速度
    const double M_PI = 3.14159265358979323846;

    // 北斗 B1I / B3I 频率 (Hz)
    const double FREQ_BDS_B1I = 1561.098e6;
    const double FREQ_BDS_B3I = 1268.52e6;

    // 组合系数
    const double F1_SQ = FREQ_BDS_B1I * FREQ_BDS_B1I;
    const double F3_SQ = FREQ_BDS_B3I * FREQ_BDS_B3I;
    const double IF_COEF1 = F1_SQ / (F1_SQ - F3_SQ); 
    const double IF_COEF2 = -F3_SQ / (F1_SQ - F3_SQ);  

    double x0_, y0_, z0_, dt0_;

    SppSolver_BDS_IF() 
    {
        x0_ = y0_ = z0_ = dt0_ = 0.0; 
    }

    void setInitialState(double x, double y, double z, double dt = 0.0) 
    {
        x0_ = x; y0_ = y; z0_ = z; dt0_ = dt;
    }

    // 计算无电离层组合伪距
    double getIFPseudoRange(const SatObservation& sat)
    {
        return IF_COEF1 * sat.pseudoRange1 + IF_COEF2 * sat.pseudoRange2;
    }

    // 获取北斗卫星发射时刻的位置和钟差（含地球自转改正和 TGD 改正）
    bool getSatStateAtEmission(const SatObservation& sat,
        int rcvWeek, double rcvTow,
        double xr, double yr, double zr,
        double& xs, double& ys, double& zs, double& svClock,
        const unordered_map<int, BdsEphemeris>& bdsMap)
    {
        if (sat.system != 4) return false;   // 北斗系统
        auto it = bdsMap.find(sat.prn);
        if (it == bdsMap.end()) return false;

        double tau = 0.0;
        CoordinateCalculator calc;

        for (int i = 0; i < 3; i++) 
        {
            double t_T = rcvTow - tau;
            int week = rcvWeek;
            if (t_T < 0) { t_T += 604800; week--; }
            else if (t_T >= 604800) { t_T -= 604800; week++; }

            // 北斗卫星位置计算
            calc.calculateBdsPosition(it->second, week, t_T, xs, ys, zs, &svClock);

            // 地球自转改正
            double omega_tau = OMEGA_E * tau;
            double x_rot = xs * cos(omega_tau) + ys * sin(omega_tau);
            double y_rot = -xs * sin(omega_tau) + ys * cos(omega_tau);
            xs = x_rot; ys = y_rot;

            double dx = xs - xr;
            double dy = ys - yr;
            double dz = zs - zr;
            double new_tau = sqrt(dx * dx + dy * dy + dz * dz) / C_LIGHT;
            if (fabs(new_tau - tau) < 1e-9) break;
            tau = new_tau;
        }

        double tgd1 = it->second.tgd1;   // 单位：秒
        double tgd_if = IF_COEF1 * tgd1;  // IF 组合的 TGD 改正
        // BDS广播钟差需要减去 TGD 项
        svClock -= tgd_if;   // 卫星钟差 = 原始钟差 - TGD_IF

        return true;
    }

    bool leastSquares(vector<SatObservation>& sats,
        int rcvWeek, double rcvTow,
        double& xr, double& yr, double& zr, double& dt,
        const unordered_map<int, BdsEphemeris>& bdsMap,
        int& usedSats, double& pdop, double& sigma0)
    {
        int n = sats.size();
        if (n < 4) return false;

        const double c = C_LIGHT;

        double x = xr;
        double y = yr;
        double z = zr;
        double cdt = c * dt;

        vector<double> satX(n), satY(n), satZ(n), svClock(n);
        Matrix HtH_inv;

        double B, L, h;
        ecefToGeodetic(x, y, z, B, L, h);

        vector<int> finalValidIdx;

        for (int iter = 0; iter < 10; ++iter)
        {
            double true_rcvTow = rcvTow - dt;
            int true_rcvWeek = rcvWeek;

            if (true_rcvTow < 0.0)
            {
                true_rcvTow += 604800.0;
                true_rcvWeek--;
            }
            else if (true_rcvTow >= 604800.0)
            {
                true_rcvTow -= 604800.0;
                true_rcvWeek++;
            }

            // 1. 每轮都用“接收时刻”计算卫星发射时刻状态
            for (int i = 0; i < n; ++i)
            {
                if (!getSatStateAtEmission(sats[i],true_rcvWeek,true_rcvTow, x, y, z,
                    satX[i], satY[i], satZ[i], svClock[i],bdsMap))
                {
                    return false;
                }
            }

            // 2. 高度角筛选
            vector<int> validIdx;
            const double min_elev = 10.0 * M_PI / 180.0;

            for (int i = 0; i < n; ++i)
            {
                double elev = calcElevation(x, y, z, satX[i], satY[i], satZ[i]);
                if (elev >= min_elev)
                {
                    validIdx.push_back(i);
                }
            }

            int m = static_cast<int>(validIdx.size());
            if (m < 4) return false;

            Matrix H(m, vector<double>(4, 0.0));
            //vector<double> y_vec(m, 0.0);
            Matrix y_vec(m, vector<double>(1, 0.0)); // 改为矩阵形式

            for (int ii = 0; ii < m; ++ii)
            {
                int i = validIdx[ii];

                double dxs = satX[i] - x;
                double dys = satY[i] - y;
                double dzs = satZ[i] - z;
                double range = sqrt(dxs * dxs + dys * dys + dzs * dzs);

                double elev = calcElevation(x, y, z, satX[i], satY[i], satZ[i]);
                double tropo = tropoCorrection(B, h, elev);

                H[ii][0] = -dxs / range;
                H[ii][1] = -dys / range;
                H[ii][2] = -dzs / range;
                H[ii][3] = 1.0;

                double pr_if = getIFPseudoRange(sats[i]);

                // P + c * dts - rho - c * dtr - tropo
                double pr_corrected = pr_if + c * svClock[i];

                y_vec[ii][0] = pr_corrected - range - cdt - tropo;
            }

            Matrix Ht;
            matrix_T(H, Ht);

            Matrix HtH;
            matrix_multiply(Ht, H, HtH);

            if (!matrix_inverse(HtH, HtH_inv))
            {
                return false;
            }

            Matrix Ht_y;
            matrix_multiply(Ht, y_vec, Ht_y);

            Matrix delta;
            matrix_multiply(HtH_inv, Ht_y, delta);
            
            x += delta[0][0];
            y += delta[1][0];
            z += delta[2][0];
            cdt += delta[3][0];
            dt = cdt / c;

            ecefToGeodetic(x, y, z, B, L, h);

            finalValidIdx = validIdx;

            double pos_change = sqrt(
                delta[0][0] * delta[0][0] +
                delta[1][0] * delta[1][0] +
                delta[2][0] * delta[2][0]);

            if (pos_change < 1e-4)
            {
                break;
            }
        }

        // 3. 用最终坐标和最终钟差重新计算一次卫星位置、残差和 PDOP
        double true_rcvTow = rcvTow - dt;
        int true_rcvWeek = rcvWeek;

        if (true_rcvTow < 0.0)
        {
            true_rcvTow += 604800.0;
            true_rcvWeek--;
        }
        else if (true_rcvTow >= 604800.0)
        {
            true_rcvTow -= 604800.0;
            true_rcvWeek++;
        }

        for (int i = 0; i < n; ++i)
        {
            if (!getSatStateAtEmission(
                sats[i],
                true_rcvWeek,
                true_rcvTow,
                x, y, z,
                satX[i], satY[i], satZ[i], svClock[i],
                bdsMap))
            {
                return false;
            }
        }

        vector<int> validIdx;
        const double min_elev = 10.0 * M_PI / 180.0;

        for (int i = 0; i < n; ++i)
        {
            double elev = calcElevation(x, y, z, satX[i], satY[i], satZ[i]);
            if (elev >= min_elev)
            {
                validIdx.push_back(i);
            }
        }

        int m = static_cast<int>(validIdx.size());
        if (m < 4) return false;

        Matrix H_final(m, vector<double>(4, 0.0));
        vector<double> v_vec(m, 0.0);

        double sum_v2 = 0.0;

        for (int ii = 0; ii < m; ++ii)
        {
            int i = validIdx[ii];

            double dxs = satX[i] - x;
            double dys = satY[i] - y;
            double dzs = satZ[i] - z;
            double range = sqrt(dxs * dxs + dys * dys + dzs * dzs);

            double elev = calcElevation(x, y, z, satX[i], satY[i], satZ[i]);
            double tropo = tropoCorrection(B, h, elev);

            H_final[ii][0] = -dxs / range;
            H_final[ii][1] = -dys / range;
            H_final[ii][2] = -dzs / range;
            H_final[ii][3] = 1.0;

            double pr_if = getIFPseudoRange(sats[i]);
            double pr_corrected = pr_if + c * svClock[i];

            double v = pr_corrected - range - cdt - tropo;
            v_vec[ii] = v;
            sum_v2 += v * v;
        }

        Matrix Ht_final;
        matrix_T(H_final, Ht_final);

        Matrix HtH_final;
        matrix_multiply(Ht_final, H_final, HtH_final);

        if (!matrix_inverse(HtH_final, HtH_inv))
        {
            return false;
        }

        if (m > 4)
        {
            sigma0 = sqrt(sum_v2 / (m - 4));
        }
        else
        {
            sigma0 = std::numeric_limits<double>::quiet_NaN();
        }

        pdop = sqrt(HtH_inv[0][0] + HtH_inv[1][1] + HtH_inv[2][2]);

        xr = x;
        yr = y;
        zr = z;
        usedSats = m;

        return true;
    }

    // ECEF 转经纬高
    void ecefToGeodetic(double x, double y, double z, double& B, double& L, double& H) 
    {
        double a = 6378137.0;
        double f = 1.0 / 298.257222101;   // CGCS2000 扁率
        double e2 = 2 * f - f * f;
        L = atan2(y, x);
        double p = sqrt(x * x + y * y);
        double theta = atan2(z * a, p * (a * (1 - e2)));
        double sin_theta = sin(theta), cos_theta = cos(theta);
        B = atan2(z + e2 * a * sin_theta * sin_theta * sin_theta,
            p - e2 * a * cos_theta * cos_theta * cos_theta);
        double sin_lat = sin(B);
        double N = a / sqrt(1 - e2 * sin_lat * sin_lat);
        H = p / cos(B) - N;
        B = B * 180.0 / M_PI;
        L = L * 180.0 / M_PI;
    }

    double calcElevation(
        double xr, double yr, double zr,
        double xs, double ys, double zs)
    {
        double B_deg, L_deg, H_m;
        ecefToGeodetic(xr, yr, zr, B_deg, L_deg, H_m);

        double lat = B_deg * M_PI / 180.0;
        double lon = L_deg * M_PI / 180.0;

        double dx = xs - xr;
        double dy = ys - yr;
        double dz = zs - zr;

        double sinLat = sin(lat);
        double cosLat = cos(lat);
        double sinLon = sin(lon);
        double cosLon = cos(lon);

        double east = -sinLon * dx + cosLon * dy;
        double north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
        double up = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;

        return atan2(up, sqrt(east * east + north * north));
    }

    // 对流层改正（与 GPS 相同）
    double tropoCorrection(double B_deg, double H_m, double elev_rad) 
    {
        double min_elev = 10.0 * M_PI / 180.0;
        if (elev_rad < min_elev) return 0.0;
        double B_rad = B_deg * M_PI / 180.0;
        double P = 1013.25 * pow(1 - 0.0000226 * H_m, 5.2561);
        double T = (15 + 273.16) - 0.0065 * H_m;
        double es = 6.112 * exp(17.67 * (T - 273.15) / (T - 29.65));
        double e = 0.5 * es;
        double zh_d = 0.002277 * P / (1 - 0.00266 * cos(2 * B_rad) - 0.00028 * H_m / 1000.0);
        double zh_w = 0.002277 * (1255.0 / T + 0.05) * e / (1 - 0.00266 * cos(2 * B_rad) - 0.00028 * H_m / 1000.0);
        double tropo = (zh_d + zh_w) / sin(elev_rad);
        if (isnan(tropo) || isinf(tropo) || tropo > 100.0) tropo = 0.0;
        return tropo;
    }

    // 主解算接口
    bool solve(const EpochData& epoch, unordered_map<int, BdsEphemeris>& bdsMap,
        Sppresult& result)
    {
        vector<SatObservation> bdsSats;
        for (const auto& sat : epoch.sats) 
        {
            if (sat.system == 4 && sat.cn0 >= 30.0 &&
                sat.pseudoRange1 > 0.0 && sat.pseudoRange2 > 0.0) 
            {
                if (bdsMap.find(sat.prn) != bdsMap.end()) 
                {
                    bdsSats.push_back(sat);
                }
            }
        }
        if (bdsSats.size() < 4) return false;

        double xr = x0_, yr = y0_, zr = z0_;
        double dt = dt0_;
        int usedSats;
        double pdop, sigma0;

        bool ok = leastSquares(bdsSats, epoch.week, epoch.tow,
            xr, yr, zr, dt, bdsMap,
            usedSats, pdop, sigma0);
        if (!ok) return false;

        result.success = true;
        result.x = xr; result.y = yr; result.z = zr;
        result.dt = dt;
        result.used_sats = usedSats;
        result.pdop = pdop;
        result.sigma0 = sigma0;
        ecefToGeodetic(xr, yr, zr, result.B, result.L, result.H);

        x0_ = xr; y0_ = yr; z0_ = zr; dt0_ = dt;
        return true;
    }
};

class SppSolver_GPS_IF
{
public:
    const double C_LIGHT = 299792458.0;
    const double OMEGA_E = 7.2921151467e-5;   // GPS WGS84 地球自转角速度
    const double M_PI = 3.14159265358979323846;

    // GPS L1 / L2 频率 (Hz)
    const double FREQ_GPS_L1 = 1575.42e6;
    const double FREQ_GPS_L2 = 1227.60e6;

    // 组合系数
    const double F1_SQ = FREQ_GPS_L1 * FREQ_GPS_L1;
    const double F2_SQ = FREQ_GPS_L2 * FREQ_GPS_L2;
    const double IF_COEF1 = F1_SQ / (F1_SQ - F2_SQ);   // 约 2.546
    const double IF_COEF2 = -F2_SQ / (F1_SQ - F2_SQ);  // 约 -1.546

    double x0_, y0_, z0_, dt0_;

    SppSolver_GPS_IF()
    {
        x0_ = y0_ = z0_ = dt0_ = 0.0;
    }

    void setInitialState(double x, double y, double z, double dt = 0.0)
    {
        x0_ = x; y0_ = y; z0_ = z; dt0_ = dt;
    }

    // 计算无电离层组合伪距
    double getIFPseudoRange(const SatObservation& sat)
    {
        return IF_COEF1 * sat.pseudoRange1 + IF_COEF2 * sat.pseudoRange2;
    }

    // 获取 GPS 卫星发射时刻的位置和钟差（含地球自转改正）
    bool getSatStateAtEmission(const SatObservation& sat,
        int rcvWeek, double rcvTow,
        double xr, double yr, double zr,
        double& xs, double& ys, double& zs, double& svClock,
        const unordered_map<int, GpsEphemeris>& gpsMap)
    {
        if (sat.system != 0) return false;   // GPS 系统
        auto it = gpsMap.find(sat.prn);
        if (it == gpsMap.end()) return false;

        double tau = 0.0;
        CoordinateCalculator calc;

        for (int i = 0; i < 3; i++)
        {
            double t_T = rcvTow - tau;
            int week = rcvWeek;
            if (t_T < 0) { t_T += 604800; week--; }
            else if (t_T >= 604800) { t_T -= 604800; week++; }

            // GPS 卫星位置计算
            calc.calculateGpsPosition(it->second, week, t_T, xs, ys, zs, &svClock);

            // 地球自转改正
            double omega_tau = OMEGA_E * tau;
            double x_rot = xs * cos(omega_tau) + ys * sin(omega_tau);
            double y_rot = -xs * sin(omega_tau) + ys * cos(omega_tau);
            xs = x_rot; ys = y_rot;

            double dx = xs - xr;
            double dy = ys - yr;
            double dz = zs - zr;
            double new_tau = sqrt(dx * dx + dy * dy + dz * dz) / C_LIGHT;
            if (fabs(new_tau - tau) < 1e-9) break;
            tau = new_tau;
        }

        // GPS 广播钟差已经基于 L1/L2 IF 组合，无需额外 TGD 改正
        return true;
    }

    // 最小二乘迭代（GPS IF 组合）
    bool leastSquares(std::vector<SatObservation>& sats,
        int rcvWeek, double rcvTow,
        double& xr, double& yr, double& zr, double& dt,
        const std::unordered_map<int, GpsEphemeris>& gpsMap,
        int& usedSats, double& pdop, double& sigma0)
    {
        int n = sats.size();
        if (n < 4) return false;

        double c = C_LIGHT;
        double x = xr, y = yr, z = zr, cdt = c * dt;

        // 真实接收时刻
        double true_rcvTow = rcvTow - dt;

        vector<double> satX(n), satY(n), satZ(n), svClock(n);
        Matrix HtH_inv;

        double B, L, h;
        ecefToGeodetic(x, y, z, B, L, h);

        for (int iter = 0; iter < 10; ++iter)
        {
            for (int i = 0; i < n; ++i)
            {
                if (!getSatStateAtEmission(sats[i],rcvWeek,true_rcvTow,x, y, z,
                    satX[i], satY[i], satZ[i], svClock[i],gpsMap))
                {
                    return false;
                }
            }

            // 2. 构建设计矩阵和残差
            Matrix H(n, vector<double>(4, 0.0));
            vector<double> y_vec(n, 0.0);

            for (int i = 0; i < n; ++i)
            {
                double dx = satX[i] - x;
                double dy = satY[i] - y;
                double dz = satZ[i] - z;
                double range = sqrt(dx * dx + dy * dy + dz * dz);
                double elev = calcElevation(x, y, z, satX[i], satY[i], satZ[i]);
                if (elev < 10.0 * M_PI / 180.0)
                {
                    continue;
                }
                double tropo = tropoCorrection(B, h, elev);

                H[i][0] = -dx / range;
                H[i][1] = -dy / range;
                H[i][2] = -dz / range;
                H[i][3] = 1.0;

                double pr_if = getIFPseudoRange(sats[i]);
                double pr_corrected = pr_if + c * svClock[i];
                y_vec[i] = pr_corrected - range - cdt - tropo;
            }

            // 3. 最小二乘求解
            Matrix Ht; matrix_T(H, Ht);
            Matrix HtH; matrix_multiply(Ht, H, HtH);
            vector<double> Ht_y(4, 0.0);
            for (int i = 0; i < 4; ++i) 
            {
                for (int j = 0; j < n; ++j)
                {
                    Ht_y[i] += Ht[i][j] * y_vec[j];
                }
            }
            if (!matrix_inverse(HtH, HtH_inv)) return false;

            vector<double> d_x(4, 0.0);
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    d_x[i] += HtH_inv[i][j] * Ht_y[j];

            x += d_x[0]; y += d_x[1]; z += d_x[2]; cdt += d_x[3];
            dt = cdt / c;
            true_rcvTow = rcvTow - dt;   // 更新真实接收时刻

            ecefToGeodetic(x, y, z, B, L, h);
            double pos_change = sqrt(d_x[0] * d_x[0] + d_x[1] * d_x[1] + d_x[2] * d_x[2]);
            if (pos_change < 1e-4) break;
        }

        // 计算验后残差与精度指标
        double sum_v2 = 0.0;
        for (int i = 0; i < n; ++i)
        {
            double dx = satX[i] - x;
            double dy = satY[i] - y;
            double dz = satZ[i] - z;
            double range = sqrt(dx * dx + dy * dy + dz * dz);
            double elev = calcElevation(x, y, z, satX[i], satY[i], satZ[i]);
            if (elev < 10.0 * M_PI / 180.0)
            {
                continue;
            }
            double tropo = tropoCorrection(B, h, elev);
            double pr_if = getIFPseudoRange(sats[i]);
            double pr_corrected = pr_if + C_LIGHT * svClock[i];
            double v = pr_corrected - range - cdt - tropo;
            sum_v2 += v * v;
        }
        if (n > 4) sigma0 = sqrt(sum_v2 / (n - 4));
        else sigma0 = std::numeric_limits<double>::quiet_NaN();
        pdop = sqrt(HtH_inv[0][0] + HtH_inv[1][1] + HtH_inv[2][2]);

        xr = x; yr = y; zr = z;
        usedSats = n;
        return true;
    }

    // ECEF 转经纬高（WGS84）
    void ecefToGeodetic(double x, double y, double z, double& B, double& L, double& H)
    {
        double a = 6378137.0;
        double f = 1.0 / 298.257223563;
        double e2 = 2 * f - f * f;
        L = atan2(y, x);
        double p = sqrt(x * x + y * y);
        double theta = atan2(z * a, p * (a * (1 - e2)));
        double sin_theta = sin(theta), cos_theta = cos(theta);
        B = atan2(z + e2 * a * sin_theta * sin_theta * sin_theta,
            p - e2 * a * cos_theta * cos_theta * cos_theta);
        double sin_lat = sin(B);
        double N = a / sqrt(1 - e2 * sin_lat * sin_lat);
        H = p / cos(B) - N;
        B = B * 180.0 / M_PI;
        L = L * 180.0 / M_PI;
    }

    double calcElevation(
        double xr, double yr, double zr,
        double xs, double ys, double zs)
    {
        double B_deg, L_deg, H_m;
        ecefToGeodetic(xr, yr, zr, B_deg, L_deg, H_m);

        double lat = B_deg * M_PI / 180.0;
        double lon = L_deg * M_PI / 180.0;

        double dx = xs - xr;
        double dy = ys - yr;
        double dz = zs - zr;

        double sinLat = sin(lat);
        double cosLat = cos(lat);
        double sinLon = sin(lon);
        double cosLon = cos(lon);

        double east = -sinLon * dx + cosLon * dy;
        double north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
        double up = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;

        return atan2(up, sqrt(east * east + north * north));
    }

    double tropoCorrection(double B_deg, double H_m, double elev_rad)
    {
        const double min_elev = 10.0 * M_PI / 180.0;   // 截止高度角 10°
        if (elev_rad < min_elev) return 0.0;

        // 1. 海平面标准参数（老师给定）
        const double P0 = 1013.25;          // hPa
        const double T0 = 15.0 + 273.16;    // 288.16 K
        const double RH0 = 0.5;             // 50%

        // 2. 测站气象参数（标准大气模型）
        double T = T0 - 0.0065 * H_m;
        double P = P0 * pow(1.0 - 0.0000226 * H_m, 5.2561);
        // 饱和水汽压 (hPa)
        double es = 6.112 * exp(17.67 * (T - 273.15) / (T - 29.65));
        double e = RH0 * es;

        // 3. Hopfield 模型参数
        double Nd = 77.6 * P / T;                  // 干折射度 (无量纲)
        double Nw = 3.73e5 * e / (T * T);          // 湿折射度 (无量纲)
        double Hd = 40136.0 + 148.72 * (T - 273.16); // 干大气等效高度 (m)
        double Hw = 11000.0;                       // 湿大气等效高度 (m)

        // 4. 天顶延迟
        double ZHD = 1e-6 * Nd * (Hd - H_m) / 5.0; // 干天顶延迟 (m)
        double ZWD = 1e-6 * Nw * (Hw - H_m) / 5.0; // 湿天顶延迟 (m)
        double ZTD = ZHD + ZWD;

        // 5. 映射函数 (简单 1/sin(elev))
        double tropo = ZTD / sin(elev_rad);

        // 异常处理
        if (std::isnan(tropo) || std::isinf(tropo) || tropo > 100.0)
            tropo = 0.0;
        return tropo;
    
    }

    // 主解算接口
    bool solve(const EpochData& epoch, std::unordered_map<int, GpsEphemeris>& gpsMap,
        Sppresult& result)
    {
        std::vector<SatObservation> gpsSats;
        for (const auto& sat : epoch.sats)
        {
            if (sat.system == 0 && sat.cn0 >= 30.0 &&
                sat.pseudoRange1 > 0.0 && sat.pseudoRange2 > 0.0)
            {
                if (gpsMap.find(sat.prn) != gpsMap.end())
                {
                    gpsSats.push_back(sat);
                }
            }
        }
        if (gpsSats.size() < 4) return false;

        double xr = x0_, yr = y0_, zr = z0_;
        double dt = dt0_;
        int usedSats;
        double pdop, sigma0;

        bool ok = leastSquares(gpsSats, epoch.week, epoch.tow,
            xr, yr, zr, dt, gpsMap,
            usedSats, pdop, sigma0);
        if (!ok) return false;

        result.success = true;
        result.x = xr; result.y = yr; result.z = zr;
        result.dt = dt;
        result.used_sats = usedSats;
        result.pdop = pdop;
        result.sigma0 = sigma0;
        ecefToGeodetic(xr, yr, zr, result.B, result.L, result.H);

        x0_ = xr; y0_ = yr; z0_ = zr; dt0_ = dt;
        return true;
    }
};

int main()
{
    SppSolver_BDS_IF solver;
    EpochReader_double reader;
    if (!reader.open("NovatelOEM20211114-01.log"))
    {
        return 1;
    }
    EpochData epoch;
    Sppresult result;
    int epochCount = 0;
    while (reader.getNextEpoch(epoch)) 
    {
        epochCount++;
        unordered_map<int, BdsEphemeris> bds=reader.getlatestBds();
        bool ok = solver.solve(epoch, bds, result);

        // 打印结果
        std::cout << std::fixed << std::setprecision(3);
        if (ok) {
            std::cout << "Epoch " << std::setw(5) << epochCount
                << "  Week:" << epoch.week
                << "  ToW:" << std::setw(10) << std::setprecision(3) << epoch.tow
                << "  |  ECEF X:" << std::setw(14) << result.x
                << "  Y:" << std::setw(14) << result.y
                << "  Z:" << std::setw(14) << result.z
                << "  |  BLH B:" << std::setw(10) << std::setprecision(7) << result.B
                << "  L:" << std::setw(11) << result.L
                << "  H:" << std::setw(8) << std::setprecision(4) << result.H
                << "  |  dt:" << std::setw(9) << std::setprecision(6) << result.dt
                << "  sats:" << result.used_sats
                << "  PDOP:" << std::setw(6) << std::setprecision(2) << result.pdop
                << "  sigma0:" << std::setw(6) << result.sigma0
                << std::endl;
        }
        if (epochCount >= 200)break;
    }


    return 0;
}