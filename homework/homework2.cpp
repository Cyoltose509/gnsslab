#include <iostream>
#include "TimeStruct.h"
#include "TimeConvert.h"
#include "CoordConvert.h"
#include "CoordStruct.h"
#include <Eigen/Dense>

namespace sample {

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
        JD2020 jd;
        MJD mjd;
        CommonTime2JD2020(commonTime,jd);
        CommonTime2MJD(commonTime,mjd);
        cout << "CommonTime is:" << commonTime << endl;

        cout << "MJD is:" << mjd << endl;
        cout << "JD2020 is:" << jd << endl;
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

        std::cout << "站心坐标： " << cartesian.X() << ", " << cartesian.Y() << ", " << cartesian.Z() << std::endl;

        // 2. 测试 ECEF -> ENU 转换
        std::cout << "--- ECEF to ENU ---" << std::endl;

        // 转换北极点
        Eigen::Vector3d enu_north = xyz2enu(xyz_north_pole, cartesian, reference_frame);
        std::cout << "北极点站心坐标: " << enu_north.transpose() << std::endl;
        // 验证：北极点应该在站心的正北方向，即 E分量接近0，N分量为正距离，U分量为正高差

        // 转换正常点
        Eigen::Vector3d enu_normal = xyz2enu(xyz_normal, cartesian, reference_frame);
        std::cout << "普通点站心坐标: " << enu_normal.transpose() << std::endl;

        // 3. 测试 ENU -> ECEF 逆转换
        std::cout << "--- ENU to ECEF---" << std::endl;

        // 将刚才算出的 enu_normal 转回 XYZ
        XYZ xyz_recovered = enu2xyz( enu_normal, cartesian, reference_frame);
        std::cout << "普通点转回XYZ: " << xyz_recovered.X() << ", " << xyz_recovered.Y() << ", " << xyz_recovered.Z() << std::endl;

        // 验证误差
        double err_x = xyz_recovered.X() - xyz_normal.X();
        double err_y = xyz_recovered.Y() - xyz_normal.Y();
        double err_z = xyz_recovered.Z() - xyz_normal.Z();
        std::cout << "差距: " << err_x << ", " << err_y << ", " << err_z << std::endl;
    }
}


int main() {
   // sample::exam21();
   //sample::exam22();
    //sample::exam23();
    //sample::exam25();
    sample::exam26();
}
