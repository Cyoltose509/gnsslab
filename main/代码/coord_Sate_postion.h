#pragma once
#define _USE_MATH_DEFINES
#include<iostream>
#include"decode.h"
#include"time.h"
#include <cmath>
#include<fstream>
// 卫星坐标计算类
class CoordinateCalculator
{
public:
    // 地球引力常数 (m³/s²)
    static constexpr double GM_GPS = 3.986005e14;
    // 地球自转角速度(GPS) (rad/s)
    static constexpr double OMEGA_EARTH = 7.2921151467e-5;
    // 光速 (m/s)
    static constexpr double LIGHT_SPEED = 2.99792458e8;
    static constexpr double GM_BDS = 3.986004418e14;      // BDS地球引力常数 (m³/s²)
    static constexpr double OMEGA_E_BDS = 7.2921150e-5;   // BDS地球自转角速度 (rad/s)
    static constexpr double M_PI = 3.141592654;      // 圆周率

    //相对论效应常数
    static constexpr double F_REL = -4.442807633e-10;

    void calculateGpsPosition(const GpsEphemeris& eph, int gpsWeek, double gpsTow,
        double& x, double& y, double& z, double* svClock = nullptr, double* v_clock = nullptr, double* vx = nullptr,
        double* vy = nullptr, double* vz = nullptr)
    {
        // 1. 时间差 dt
        double t_obs = gpsWeek * 604800.0 + gpsTow;
        double t_toe = eph.week * 604800.0 + eph.toe;
        double dt = t_obs - t_toe;

        // 星历过期检查（可选）
        if (fabs(dt) > 7200.0) {
            // 可输出警告
        }

        // 2. 长半轴
        double a = eph.A;

        // 3. 平均角速度
        double n0 = sqrt(GM_GPS / (a * a * a));
        double n = n0 + eph.dN;   // 注意：结构体中使用 dN

        // 4. 平近点角
        double M = eph.M0 + n * dt;

        // 5. 偏近点角（迭代）
        double E = M;
        double E_old;
        for (int i = 0; i < 10; ++i) {
            E_old = E;
            E = M + eph.ecc * sin(E);
            if (fabs(E - E_old) < 1e-12) break;
        }

        // 6. 真近点角
        double nu = atan2(sqrt(1 - eph.ecc * eph.ecc) * sin(E), cos(E) - eph.ecc);

        // 7. 升交角距
        double phi = nu + eph.omega;

        // 8. 摄动改正
        double du = eph.cuc * cos(2 * phi) + eph.cus * sin(2 * phi);
        double dr = eph.crc * cos(2 * phi) + eph.crs * sin(2 * phi);
        double di = eph.cic * cos(2 * phi) + eph.cis * sin(2 * phi);

        // 9. 改正后的参数
        double u = phi + du;
        double r = a * (1 - eph.ecc * cos(E)) + dr;
        double i = eph.i0 + eph.idot * dt + di;

        // 10. 轨道平面坐标
        double x_prime = r * cos(u);
        double y_prime = r * sin(u);

        // 11. 升交点赤经
        double Omega = eph.omega0 + (eph.omegaDot - OMEGA_EARTH) * dt - OMEGA_EARTH * eph.toe;

        // 12. ECEF 坐标
        double cosOmega = cos(Omega);
        double sinOmega = sin(Omega);
        double cosi = cos(i);
        double sini = sin(i);
        x = x_prime * cosOmega - y_prime * cosi * sinOmega;
        y = x_prime * sinOmega + y_prime * cosi * cosOmega;
        z = y_prime * sini;
        //计算钟差
        double t_oc = eph.week * 604800.0 + eph.toc;
        double dt_1 = t_obs - t_oc;
        if (svClock)
        {
            double tr = F_REL * eph.ecc * sqrt(a) * sin(E);
            double clockRaw = eph.af0 + eph.af1 * dt_1 + eph.af2 * dt_1 * dt_1 + tr;
            *svClock = clockRaw;
        }
        //计算钟速
        double E_dot = n / (1.0 - eph.ecc * cos(E));
        if (v_clock)
        {
            double tr_dot = F_REL * eph.ecc * sqrt(a) * cos(E) * E_dot;
            *v_clock = eph.af1 + 2 * eph.af2 * dt_1 + tr_dot;
        }
        //速度计算
        if (vx && vy && vz)
        {
            // 1. 偏近点角变化率 E_dot
            double E_dot = n / (1.0 - eph.ecc * cos(E));

            // 2. 升交角距变化率 phi_dot（即真近点角变化率 nu_dot）
            double phi_dot = sqrt(1 - eph.ecc * eph.ecc) * E_dot / (1 - eph.ecc * cos(E));

            // 3. 摄动改正变化率
            double du_dot = 2.0 * phi_dot * (eph.cus * cos(2 * phi) - eph.cuc * sin(2 * phi));
            double dr_dot = 2.0 * phi_dot * (eph.crs * cos(2 * phi) - eph.crc * sin(2 * phi));
            double di_dot = 2.0 * phi_dot * (eph.cis * cos(2 * phi) - eph.cic * sin(2 * phi));

            // 4. 改正后参数变化率
            double u_dot = phi_dot + du_dot;
            double r_dot = a * eph.ecc * sin(E) * E_dot + dr_dot;
            double i_dot = eph.idot + di_dot;

            // 5. 轨道平面速度
            double xp_dot = r_dot * cos(u) - r * sin(u) * u_dot;
            double yp_dot = r_dot * sin(u) + r * cos(u) * u_dot;

            // 6. 升交点赤经变化率
            double Omega_dot = eph.omegaDot - OMEGA_EARTH;

            // 7. ECEF 速度
            *vx = (xp_dot * cosOmega - yp_dot * cosi * sinOmega)
                - (x_prime * sinOmega + y_prime * cosi * cosOmega) * Omega_dot
                + (y_prime * sini * sinOmega) * i_dot;
            *vy = (xp_dot * sinOmega + yp_dot * cosi * cosOmega)
                + (x_prime * cosOmega - y_prime * cosi * sinOmega) * Omega_dot
                - (y_prime * sini * cosOmega) * i_dot;
            *vz = yp_dot * sini + y_prime * cosi * i_dot;
        }
    }
    bool calculateBdsPosition(const BdsEphemeris& eph, int gpsWeek, double gpsTow,
        double& x, double& y, double& z, double* svClock = nullptr, double* v_clock = nullptr,
        double* vx = nullptr, double* vy = nullptr, double* vz = nullptr)
    {
        // 1. 转换为北斗时间
        int bdsWeek;
        double bdsTow;
        gpsTimeToBdsTime(gpsWeek, gpsTow, bdsWeek, bdsTow);

        // 2. 时间差 dt
        double t_obs = bdsWeek * 604800.0 + bdsTow;
        double t_toe = eph.week * 604800.0 + eph.toe;
        double dt = t_obs - t_toe;

        // 星历有效期检查（北斗通常 1 小时）
        if (fabs(dt) > 3600.0)
        {
            return false;
        }
        // 检查
        if (eph.rootA <= 0) return false;
        double a = eph.rootA * eph.rootA;
        if (a <= 0) return false;

        // 3. 长半轴
        //double a = eph.rootA * eph.rootA;

        // 4. 平均角速度
        double n0 = sqrt(GM_BDS / (a * a * a));
        double n = n0 + eph.dn;   // 北斗用 dn

        // 5. 平近点角
        double M = eph.M0 + n * dt;

        // 6. 偏近点角
        double E = M;
        double E_old;
        for (int i = 0; i < 10; ++i)
        {
            E_old = E;
            E = M + eph.e * sin(E);
            if (fabs(E - E_old) < 1e-12) break;
        }

        // 7. 真近点角
        double nu = atan2(sqrt(1 - eph.e * eph.e) * sin(E), cos(E) - eph.e);

        // 8. 升交角距
        double phi = nu + eph.omega;

        // 9. 摄动改正
        double sin2phi = sin(2.0 * phi);
        double cos2phi = cos(2.0 * phi);
        double du = eph.cus * sin2phi + eph.cuc * cos2phi;
        double dr = eph.crs * sin2phi + eph.crc * cos2phi;
        double di = eph.cis * sin2phi + eph.cic * cos2phi;

        // 10. 改正后的参数
        double u = phi + du;
        double r = a * (1.0 - eph.e * cos(E)) + dr;
        double i = eph.i0 + eph.idot * dt + di;

        // 11. 轨道平面坐标
        double x_prime = r * cos(u);
        double y_prime = r * sin(u);


        //北斗特殊处理：部分GEO卫星（PRN 1-5, 59-62)，使用特殊公式
        bool isGeo = (eph.prn >= 1 && eph.prn <= 5) || (eph.prn >= 59 && eph.prn <= 62);
        if (isGeo)
        {
            double Omega = eph.Omega0 + eph.OmegaDot * dt - OMEGA_E_BDS * eph.toe;
            // 13. ECEF 坐标
            double cosOmega = cos(Omega);
            double sinOmega = sin(Omega);
            double cosi = cos(i);
            double sini = sin(i);
            double xgk = x_prime * cosOmega - y_prime * cosi * sinOmega;
            double ygk = x_prime * sinOmega + y_prime * cosi * cosOmega;
            double zgk = y_prime * sini;
            double I_5 = -1 * 5.0 * M_PI / 180.0; // 5° 弧度
            double I_1 = OMEGA_E_BDS * dt;
            double cosI_1 = cos(I_1);
            double sinI_1 = sin(I_1);
            double cosI_5 = cos(I_5);
            double sinI_5 = sin(I_5);
            x = cosI_1 * xgk + sinI_1 * cosI_5 * ygk + sinI_1 * sinI_5 * zgk;
            y = -1 * sinI_1 * xgk + cosI_1 * cosI_5 * ygk + cosI_1 * sinI_5 * zgk;
            z = -1 * sinI_5 * ygk + cosI_5 * zgk;
            if (vx && vy && vz)
            {
                // 偏近点角变化率
                double E_dot = n / (1.0 - eph.e * cos(E));
                // 升交角距变化率
                double phi_dot = sqrt(1 - eph.e * eph.e) * E_dot / (1 - eph.e * cos(E));
                // 摄动改正变化率
                double du_dot = 2.0 * phi_dot * (eph.cus * cos2phi - eph.cuc * sin2phi);
                double dr_dot = 2.0 * phi_dot * (eph.crs * cos2phi - eph.crc * sin2phi);
                double di_dot = 2.0 * phi_dot * (eph.cis * cos2phi - eph.cic * sin2phi);
                // 改正后参数变化率
                double u_dot = phi_dot + du_dot;
                double r_dot = a * eph.e * sin(E) * E_dot + dr_dot;
                double i_dot = eph.idot + di_dot;
                // 轨道平面速度
                double xp_dot = r_dot * cos(u) - r * sin(u) * u_dot;
                double yp_dot = r_dot * sin(u) + r * cos(u) * u_dot;
                // 升交点赤经变化率
                double Omega_dot = eph.OmegaDot;
                double v1, v2, v3, x_1, y_1;
                v1 = (xp_dot * cosOmega - yp_dot * cosi * sinOmega)
                    - (x_prime * sinOmega + y_prime * cosi * cosOmega) * Omega_dot
                    + (y_prime * sini * sinOmega) * i_dot;
                v2 = (xp_dot * sinOmega + yp_dot * cosi * cosOmega)
                    + (x_prime * cosOmega - y_prime * cosi * sinOmega) * Omega_dot
                    - (y_prime * sini * cosOmega) * i_dot;
                v3 = yp_dot * sini + y_prime * cosi * i_dot;
                x_1 = OMEGA_E_BDS * (-1 * sinI_1 * xgk + cosI_1 * cosI_5 * ygk + cosI_1 * sinI_5 * zgk);
                y_1 = OMEGA_E_BDS * (-1 * cosI_1 * xgk - sinI_1 * cosI_5 * ygk - sinI_1 * sinI_5 * zgk);
                *vx = x_1 + cosI_1 * v1 + sinI_1 * cosI_5 * v2 + sinI_1 * sinI_5 * v3;
                *vy = y_1 - 1 * sinI_1 * v1 + cosI_1 * cosI_5 * v2 + cosI_1 * sinI_5 * v3;
                *vz = -1 * sinI_5 * v2 + cosI_5 * v3;
            }
        }
        else
        {
            // 12. 升交点赤经
            double Omega = eph.Omega0 + (eph.OmegaDot - OMEGA_E_BDS) * dt - OMEGA_E_BDS * eph.toe;

            // 13. ECEF 坐标
            double cosOmega = cos(Omega);
            double sinOmega = sin(Omega);
            double cosi = cos(i);
            double sini = sin(i);
            x = x_prime * cosOmega - y_prime * cosi * sinOmega;
            y = x_prime * sinOmega + y_prime * cosi * cosOmega;
            z = y_prime * sini;
            //速度计算
            if (vx && vy && vz)
            {
                // 偏近点角变化率
                double E_dot = n / (1.0 - eph.e * cos(E));
                // 升交角距变化率
                double phi_dot = sqrt(1 - eph.e * eph.e) * E_dot / (1 - eph.e * cos(E));
                // 摄动改正变化率
                double du_dot = 2.0 * phi_dot * (eph.cus * cos2phi - eph.cuc * sin2phi);
                double dr_dot = 2.0 * phi_dot * (eph.crs * cos2phi - eph.crc * sin2phi);
                double di_dot = 2.0 * phi_dot * (eph.cis * cos2phi - eph.cic * sin2phi);
                // 改正后参数变化率
                double u_dot = phi_dot + du_dot;
                double r_dot = a * eph.e * sin(E) * E_dot + dr_dot;
                double i_dot = eph.idot + di_dot;
                // 轨道平面速度
                double xp_dot = r_dot * cos(u) - r * sin(u) * u_dot;
                double yp_dot = r_dot * sin(u) + r * cos(u) * u_dot;
                // 升交点赤经变化率
                double Omega_dot = eph.OmegaDot - OMEGA_E_BDS;
                // ECEF 速度
                *vx = (xp_dot * cosOmega - yp_dot * cosi * sinOmega)
                    - (x_prime * sinOmega + y_prime * cosi * cosOmega) * Omega_dot
                    + (y_prime * sini * sinOmega) * i_dot;
                *vy = (xp_dot * sinOmega + yp_dot * cosi * cosOmega)
                    + (x_prime * cosOmega - y_prime * cosi * sinOmega) * Omega_dot
                    - (y_prime * sini * cosOmega) * i_dot;
                *vz = yp_dot * sini + y_prime * cosi * i_dot;
            }
        }
        //钟差计算
        double t_oc = eph.week * 604800.0 + eph.toc;
        double dt_clock = t_obs - t_oc;  //星历中读取 toc 计算
        if (svClock)
        {
            // 相对论改正项
            double tr = F_REL * eph.e * sqrt(a) * sin(E);
            double clockRaw = eph.a0 + eph.a1 * dt_clock + eph.a2 * dt_clock * dt_clock + tr;
            *svClock = clockRaw;
        }
        //计算钟速
        double E_dot = n / (1.0 - eph.e * cos(E));
        if (v_clock)
        {
            double tr_dot = F_REL * eph.e * sqrt(a) * cos(E) * E_dot;
            *v_clock = eph.a1 + 2 * eph.a2 * dt_clock + tr_dot;
        }
        return true;
    }
};