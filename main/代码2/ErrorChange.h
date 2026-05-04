#include<Eigen/Core>
#include<vector>
#include<cmath>		
#include<iostream>

#include"PositionTrans.h"
	
#define F -4.442807633e-10//地球扁率
#define PI 3.14159265358979323846
#define OMEe_dt_G 7.2921151467e-5
#define OMEe_dt_B 7.2921151467e-5
#define Vc 299792458//光速
#define u_G 3.9860050e14//地球引力常数
#define u_B 3.986004418e14
//测站坐标
struct ECEF {
	Eigen::Vector3d XYZ;
	bool exist = 0;
};
const ECEF Station0 = { Eigen::Vector3d(-2267749.0000,5009154.0000,3221290.0000),1 };//参考测站坐标
//与卫星相关参数	
struct SatelliteData {
	Eigen::Vector3d XYZ;
	Eigen::Vector3d XYZdot;
	double clk;//单位为秒
	double P;//伪距
	double P_std_2;//伪距方差
	bool health = 0;
	double week, second;
	bool exist = 0;
	int system;
};
void PrintSatelliteData(SatelliteData data) {
	cout << "P:" << data.XYZ.transpose() << " V:" << data.XYZdot.transpose();
	cout << " clk:" << data.clk << endl;cout << "P:" << data.P << " P_std_2:" << data.P_std_2 << " week:" << data.week << " second:" << data.second << endl;
}

//计算地球自传改正
void EarthRotation(SatelliteData& pvt) {
	double DeltaT = pvt.P / Vc+pvt.clk;
	double a = DeltaT * OMEe_dt_G;
	Eigen::MatrixXd Rz(3, 3);Rz << cos(a), sin(a), 0,
		-sin(a), cos(a), 0,
		0, 0, 1;
	pvt.XYZ = Rz * pvt.XYZ;
}


//计算电离层改正值
void Klobuchar() {
}
//计算高度角
double ElevationAngle(SatelliteData satellite,ECEF station0) {
	double dx = satellite.XYZ(0) - station0.XYZ(0);
	double dy = satellite.XYZ(1) - station0.XYZ(1);
	double dz = satellite.XYZ(2) - station0.XYZ(2);
	double r = sqrt(dx * dx + dy * dy + dz * dz);
	double cosE = (dx * station0.XYZ(0) + dy * station0.XYZ(1) + dz * station0.XYZ(2)) / (r * station0.XYZ.norm());
	double E = 90-acos(cosE)*180/PI;
	return E;
}
//计算测站高度
double StationHeight(ECEF station) {
	XYZ Ps;Ps.X = station.XYZ(0);Ps.Y = station.XYZ(1);Ps.Z = station.XYZ(2);
	double H = XYZToBLH(Ps).Height;
	return H;
}
//计算对流层改正值
double Hopfiled(double H, double E) {
    // Constants
    const double H0 = 0.0; // 海平面
    const double T0 = 15.0 + 273.16; // 参考温度（海面）
    const double p0 = 1013.25; //参考气压
    const double RH0 = 0.5; // 参考湿度
	const double hw = 11000;
	const double hd = 40136 + 148.72*(T0 - 273.16);
    //计算测站相关数据
	double T = T0 - 0.0065 * (H - H0);
    double P = p0 * pow((1 - 0.0000226 * (H - H0)), 5.225);
    double RH = RH0 * exp(-0.0006396 * (H - H0));
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

    double Kd = 155.2 * pow(10, -7) * P / T*(hd-H);
    double Kw = 155.2* pow(10, -7)*4810/T/T*e*(hw-H);

	double delta_trp = Kd / sin(sqrt((E * E + 6.25) * PI / 180)) + Kw / sin(sqrt((E * E + 2.25) * PI / 180));
    return delta_trp;
}
//对伪距进行改正
void GPSPsrCorrection(SatelliteData& pvt, ECEF station0,GPSEphem GPSData) {//参数为卫星PVT和测站参考坐标
	pvt.P = pvt.P + pvt.clk * Vc;//钟差改正
	double E = ElevationAngle(pvt, station0);//计算高度角
	double H = StationHeight(station0);//计算测站高度
	double delta_trp = Hopfiled(H, E);//计算对流层改正值
	//double delta_trp = Hopfiled(H, E);
	pvt.P = pvt.P - delta_trp;//对伪距进行改正
}