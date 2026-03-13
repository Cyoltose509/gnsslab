#include <iostream>
#include "TimeStruct.h"
#include "TimeConvert.h"
#include "CoordConvert.h"
#include "CoordStruct.h"
#include <Eigen/Dense>

namespace sample {
#define BDT_EPOCH_MJD 53736

    class BDTWeekSecond : public WeekSecond {
    public:
        /// Constructor.
        BDTWeekSecond(unsigned int w = 0,
                      double s = 0.,
                      TimeSystem ts = TimeSystem::BDT)
            : WeekSecond(w, s) { timeSystem = ts; }

        /// Constructor from CommonTime

        /// Destructor.
        ~BDTWeekSecond() {
        }

        /// Return the number of bits in the bitmask used to get the
        /// ModWeek from the full week.
        int Nbits(void) const {
            static const int n = 10;
            return n;
        }

        /// Return the bitmask used to get the ModWeek from the full week.
        int bitmask(void) const {
            static const int bm = 0x3FF;
            return bm;
        }

        /// Return the Modified Julian Date (MJD) of epoch for this system.
        long MJDEpoch(void) const {
            static const long e = BDT_EPOCH_MJD;
            return e;
        }

        inline bool operator==(const BDTWeekSecond &right) const {
            return WeekSecond::operator==(right);
        }

        inline bool operator!=(const BDTWeekSecond &right) const {
            return WeekSecond::operator!=(right);
        }

        inline bool operator<(const BDTWeekSecond &right) const {
            return WeekSecond::operator<(right);
        }

        inline bool operator>(const BDTWeekSecond &right) const {
            return WeekSecond::operator>(right);
        }

        inline bool operator<=(const BDTWeekSecond &right) const {
            return WeekSecond::operator<=(right);
        }

        inline bool operator>=(const BDTWeekSecond &right) const {
            return WeekSecond::operator>=(right);
        }
    }; // end class GPSWeekSecond

    constexpr double JD2020_EPOCH = 2458849.5;

    double CommonTime2JD2020(CommonTime &ct) {
        long mjd_day;
        double sod;
        ct.get(mjd_day, sod, ct.m_timeSystem);
        double jday = mjd_day + MJD_TO_JD;
        return static_cast<long double>(jday) + (static_cast<long double>(sod)) * DAY_PER_SEC - JD2020_EPOCH;
    }

    void exam21() {
        std::cout << std::endl << "2.1作业：" << std::endl;
        //2.1
        Eigen::MatrixXd m(2, 2); // MatrixXd 表示的是动态数组，初始化的时候指定数组的行数和列数

        m(0, 0) = 3; //m(i,j) 表示第i行第j列的值，这里对数组进行初始化
        m(1, 0) = 2.5;
        m(0, 1) = -1;
        m(1, 1) = m(1, 0) + m(0, 1);
        std::cout << "原矩阵：" << std::endl << m << std::endl;
        std::cout << "矩阵的二次方" << std::endl << m * m << std::endl;
        std::cout << "矩阵求逆" << std::endl << m.inverse() << std::endl;
    }

    void exam22() {
        std::cout << std::endl << "2.2作业：" << std::endl;
        //2.2
        CivilTime civilTime(1993, 7, 1, 0, 0, 7);
        CommonTime GPS = CivilTime2CommonTime(civilTime);
        CommonTime UTC = convertTimeSystem(GPS, TimeSystem::UTC);
        UTC.m_timeSystem = TimeSystem::UTC;
        std::cout << "gps is: " << civilTime << std::endl;
        std::cout << "utc is: " << CommonTime2CivilTime(UTC) << std::endl;
    }

    void exam23() {
        std::cout << std::endl << "2.3作业：" << std::endl;
        //2.3
        CommonTime bdt;
        BDTWeekSecond ws(991, 287986.000000);
        WeekSecond2CommonTime(ws, bdt);
        std::cout << "week second is:" << ws << std::endl;
        std::cout << "common time is:" << bdt << std::endl;
    }

    void exam25() {
        std::cout << std::endl << "2.5作业：" << std::endl;
        CivilTime civilTime2(2025, 1, 4, 9, 0, 0.0);
        CommonTime commonTime = CivilTime2CommonTime(civilTime2);
        cout << "CommonTime is:" << commonTime << endl;

        cout << "JD2020 is:" << CommonTime2JD2020(commonTime) << endl;
    }
    Eigen::Matrix3d computeRotationMatrix(double B, double L) {
        double sinB = std::sin(B);
        double cosB = std::cos(B);
        double sinL = std::sin(L);
        double cosL = std::cos(L);

        Eigen::Matrix3d R;
        R << -sinL,            cosL,           0.0,
             -sinB * cosL,     -sinB * sinL,   cosB,
              cosB * cosL,      cosB * sinL,   sinB;

        return R;
    }

    Eigen::Vector3d xyz2enu(const BLH& refBLH, const XYZ& targetXYZ, const XYZ& refXYZ) {
        Eigen::Vector3d diffXYZ;
        diffXYZ << targetXYZ.X() - refXYZ.X(),
                   targetXYZ.Y() - refXYZ.Y(),
                   targetXYZ.Z() - refXYZ.Z();

        // 2. 获取旋转矩阵
        Eigen::Matrix3d R = computeRotationMatrix(refBLH.B(), refBLH.L());

        // 3. 应用旋转: ENU = R * XYZ_vec
        return R * diffXYZ;
    }

    Eigen::Vector3d enu2xyz(const BLH& refBLH, const Eigen::Vector3d& enu, const XYZ& refXYZ) {
        // 1. 获取旋转矩阵
        Eigen::Matrix3d R = computeRotationMatrix(refBLH.B(), refBLH.L());

        // 2. 逆变换: XYZ_vec = R.transpose() * ENU (因为旋转矩阵是正交矩阵，逆=转置)
        Eigen::Vector3d diffXYZ = R.transpose() * enu;

        // 3. 加上参考点坐标
        Eigen::Vector3d resXYZ;
        resXYZ << refXYZ.X() + diffXYZ.x(),
                   refXYZ.Y() + diffXYZ.y(),
                   refXYZ.Z() + diffXYZ.z();

        return resXYZ;
    }

    void exam26() {
        std::cout << std::endl << "2.6作业：" << std::endl;

        const auto reference_frame = WGS84();

        // 定义点坐标
        XYZ xyz_north_pole(0.0, 0.0, 6356752.314);
        XYZ xyz_south_pole(0.0, 0.0, -6356752.314);
        XYZ xyz_normal(4081945.67, 2187689.34, 4767321.89);
        // 假设 cartesian 是站心（参考点）
        XYZ cartesian(4081945, 2187689, 4767321);

        // 1. 计算参考点（站心）的 BLH
        BLH refBLH = xyz2blh(cartesian, reference_frame);
        std::cout << "站心坐标： " << cartesian.X() << ", " << cartesian.Y() << ", " << cartesian.Z() << std::endl;
        std::cout << "站心BLH：" << refBLH.B() << ", " << refBLH.L() << ", " << refBLH.H() << std::endl;

        // 2. 测试 ECEF -> ENU 转换
        std::cout << "--- ECEF to ENU ---" << std::endl;

        // 转换北极点
        Eigen::Vector3d enu_north = xyz2enu(refBLH, xyz_north_pole, cartesian);
        std::cout << "北极点站心坐标: " << enu_north.transpose() << std::endl;
        // 验证：北极点应该在站心的正北方向，即 E分量接近0，N分量为正距离，U分量为正高差

        // 转换正常点
        Eigen::Vector3d enu_normal = xyz2enu(refBLH, xyz_normal, cartesian);
        std::cout << "普通点站心坐标: " << enu_normal.transpose() << std::endl;

        // 3. 测试 ENU -> ECEF 逆转换
        std::cout << "--- ENU to ECEF---" << std::endl;

        // 将刚才算出的 enu_normal 转回 XYZ
        XYZ xyz_recovered = enu2xyz(refBLH, enu_normal, cartesian);
        std::cout << "普通点转回XYZ: " << xyz_recovered.X() << ", " << xyz_recovered.Y() << ", " << xyz_recovered.Z() << std::endl;

        // 验证误差
        double err_x = xyz_recovered.X() - xyz_normal.X();
        double err_y = xyz_recovered.Y() - xyz_normal.Y();
        double err_z = xyz_recovered.Z() - xyz_normal.Z();
        std::cout << "差距: " << err_x << ", " << err_y << ", " << err_z << std::endl;
    }
}


int main() {
    sample::exam21();
   sample::exam22();
    sample::exam23();
    sample::exam25();
    sample::exam26();
}
