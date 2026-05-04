#include <Eigen/Dense>

#include"OEM7Read.h"
#include"ErrorChange.h"

const double f0 = 1.023 * 10e6;
const double L1_f = 157.542 * 10e6;//154 * f0;//L1频率
const double L2_f = 122.760 * 10e6;//120 * f0;//L2频率
const double B1I_f = 156.1098 * 10e6;//B1I频率
const double B3I_f = 126.852 * 10e6;//B3I频率

//单个GPS卫星的双频观测数据
struct DoubleFrequencyData {
	RangeData F1, F2;
};

//迭代法求偏近点角Ek
double C_Ek(double E0, double e) {
	double E = E0;
	double E_1;
	do
	{
		E_1 = E;
		E = E0 + e * sin(E_1);
	} while (abs(E_1 - E) > 0.0001);
	return E;

}
//计算卫星数据
void CalGPS(GPSEphem& data, double tk, SatelliteData& GPSPVTData) {
	double n0 = sqrt(u_G / pow(data.A, 3));//平均角速度0.000145859rad/s
	double toe = data.toe;//星历参考时刻
	double delta_n = data.dn;//平均角速度修正值
	double nA = n0 + delta_n;//改正平均角速度
	double e = data.e;//偏心率
	double M0 = data.M0;//参考时刻平近点角
	double Mk = M0 + nA * tk;
	double Ek = C_Ek(Mk, e);data.E = Ek;
	double vk = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(Ek / 2));//计算真近点角
	double omega = data.omega;//近地点角距
	double phi = vk + omega;//卫星与升交点间地心夹角
	double Cus = data.cus, Crs = data.crs, Cis = data.cis, Cuc = data.cuc, Crc = data.crc, Cic = data.cic;//卫星各周期震动的正弦项振幅
	double s2 = sin(2 * phi), c2 = cos(2 * phi);
	double Qu = Cus * s2 + Cuc * c2, Qr = Crs * s2 + Crc * c2, Qi = Cis * s2 + Cic * c2;//短周期摄动项
	double uk = phi + Qu, rk = data.A * (1 - e * cos(Ek)) + Qr, i0 = data.i0;//短周期摄动改正，ik的值不确定
	double ik = i0 + Qi + data.idot * tk;
	double x0 = rk * cos(uk), y0 = rk * sin(uk);
	double OME_dt = data.Omegadot, OMEk = data.Omega0 - OMEe_dt_G * toe + (OME_dt - OMEe_dt_G) * tk;//计算升交点经度
	double xk = x0 * cos(OMEk) - y0 * cos(ik) * sin(OMEk);
	double yk = x0 * sin(OMEk) + y0 * cos(ik) * cos(OMEk);
	double zk = y0 * sin(ik);
	Eigen::Vector3d XYZ(xk, yk, zk);
	//速度计算
	double Edot = nA / (1 - e * cos(Ek));//偏近点角速率
	double phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(Ek));//升交角距速率
	double rdot = data.A * e * sin(Ek) * Edot + 2 * (Crs * c2 - Crc * s2) * phidot;//轨道半径速率
	double ukdot = phidot + 2 * phidot * (Cus * c2 - Cuc * s2);
	double OMEkdot = OME_dt - OMEe_dt_G;//升交点速率
	double ikdot = 2 * phidot * (Cis * c2 - Cic * s2) + data.idot;
	double x0dot = rdot * cos(uk) - rk * ukdot * sin(uk);
	double y0dot = rdot * sin(uk) + rk * ukdot * cos(uk);//轨道平面内速度分量
	double cosik = cos(ik), sinik = sin(ik);
	double xkdot = x0dot * cos(OMEk) - x0 * sin(OMEk) * OMEkdot
		- (y0dot * cosik * sin(OMEk) + y0 * (-sinik * ikdot) * sin(OMEk) + y0 * cosik * cos(OMEk) * OMEkdot);
	double ykdot = x0dot * sin(OMEk) + x0 * cos(OMEk) * OMEkdot
		+ (y0dot * cosik * cos(OMEk) + y0 * (-sinik * ikdot) * cos(OMEk) - y0 * cosik * sin(OMEk) * OMEkdot);
	double zkdot = y0dot * sinik + y0 * cosik * ikdot;
	Eigen::Vector3d XYZdot(xkdot, ykdot, zkdot);
	//钟差计算
	double delta_tr = F * data.e * sqrt(data.A) * sin(Ek);
	double delta_tsv = data.a0 + data.a1 * tk + data.a2 * tk * tk + delta_tr;
	GPSPVTData.XYZ = XYZ;GPSPVTData.XYZdot = XYZdot;GPSPVTData.clk = delta_tsv;GPSPVTData.health = data.health;GPSPVTData.week = data.week;GPSPVTData.second = data.toe + tk;
	if (GPSPVTData.second > 604800) {
		GPSPVTData.week++;
		GPSPVTData.second -= 604800;
	}
	return;
}
void CalBDS(BDSEphem& data, double tk, SatelliteData& GPSPVTData) {
	double A = data.RootA * data.RootA;//计算轨道长半径
	double n0 = sqrt(u_B / pow(A, 3));//平均角速度
	double toe = data.toe;//星历参考时刻
	double delta_n = data.dn;//平均角速度修正值
	double nA = n0 + delta_n;//改正平均角速度
	double e = data.e;//偏心率
	double M0 = data.M0;//参考时刻平近点角
	double Mk = M0 + nA * tk;
	double Ek = C_Ek(Mk, e);
	double vk = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(Ek / 2));//计算真近点角
	double omega = data.omega;//近地点角距
	double phi = vk + omega;//卫星与升交点间地心夹角
	double Cus = data.cus, Crs = data.crs, Cis = data.cis, Cuc = data.cuc, Crc = data.crc, Cic = data.cic;//卫星各周期震动的正弦项振幅
	double s2 = sin(2 * phi), c2 = cos(2 * phi);
	double Qu = Cus * s2 + Cuc * c2, Qr = Crs * s2 + Crc * c2, Qi = Cis * s2 + Cic * c2;//短周期摄动项
	double uk = phi + Qu, rk = A * (1 - e * cos(Ek)) + Qr, i0 = data.i0;//短周期摄动改正，ik的值不确定
	double ik = i0 + Qi + data.idot * tk;
	double x0 = rk * cos(uk), y0 = rk * sin(uk);
	double OME_dt = data.Omegadot, OMEk, OMEkdot;//计算升交点经度
	vector<double>res(3);
	if (data.PRN < 6 || data.PRN > 58) {//GEO卫星
		OMEk = data.Omega0 - OMEe_dt_B * toe + OME_dt * tk;
		OMEkdot = OME_dt;
	}
	else {
		OMEk = data.Omega0 - OMEe_dt_B * toe + (OME_dt - OMEe_dt_B) * tk;
		OMEkdot = OME_dt - OMEe_dt_B;//升交点速率
	}
	double xk = x0 * cos(OMEk) - y0 * cos(ik) * sin(OMEk);
	double yk = x0 * sin(OMEk) + y0 * cos(ik) * cos(OMEk);
	double zk = y0 * sin(ik);
	Eigen::Vector3d XYZ(xk, yk, zk);
	//速度计算
	double Edot = nA / (1 - e * cos(Ek));//偏近点角速率
	double phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(Ek));//升交角距速率
	double rdot = A * e * sin(Ek) * Edot + 2 * (Crs * c2 - Crc * s2) * phidot;//轨道半径速率
	double ukdot = phidot + 2 * phidot * (Cus * c2 - Cuc * s2);
	double ikdot = 2 * phidot * (Cis * c2 - Cic * s2) + data.idot;
	double x0dot = rdot * cos(uk) - rk * ukdot * sin(uk);
	double y0dot = rdot * sin(uk) + rk * ukdot * cos(uk);//轨道平面内速度分量
	double cosik = cos(ik), sinik = sin(ik);
	double xkdot = x0dot * cos(OMEk) - x0 * sin(OMEk) * OMEkdot
		- (y0dot * cosik * sin(OMEk) + y0 * (-sinik * ikdot) * sin(OMEk) + y0 * cosik * cos(OMEk) * OMEkdot);
	double ykdot = x0dot * sin(OMEk) + x0 * cos(OMEk) * OMEkdot
		+ (y0dot * cosik * cos(OMEk) + y0 * (-sinik * ikdot) * cos(OMEk) - y0 * cosik * sin(OMEk) * OMEkdot);
	double zkdot = y0dot * sinik + y0 * cosik * ikdot;
	Eigen::Vector3d XYZdot;
	if (data.PRN < 6 || data.PRN > 58) {
		OMEk = data.Omega0 - OMEe_dt_B * toe + OME_dt * tk;
		double CosGEO = cos(-5.0 / 180 * PI), SinGEO = sin(-5.0 / 180 * PI);
		Eigen::MatrixXd Rx(3, 3);Rx << 1, 0, 0, 0, CosGEO, SinGEO, 0, -SinGEO, CosGEO;
		double CosT = cos(OMEe_dt_B * tk), SinT = sin(OMEe_dt_B * tk);
		Eigen::MatrixXd Rz(3, 3);Rz << CosT, SinT, 0, -SinT, CosT, 0, 0, 0, 1;
		Eigen::Vector3d Pgk(xk, yk, zk);
		Eigen::Vector3d P = Rz * Rx * Pgk;
		XYZ = Eigen::Vector3d(P(0), P(1), P(2));
		Eigen::Vector3d Vgk(xkdot, ykdot, zkdot);
		Eigen::MatrixXd Rzdot(3, 3);Rzdot << -sin(OMEe_dt_B * tk), cos(OMEe_dt_B * tk), 0, -cos(OMEe_dt_B * tk), -sin(OMEe_dt_B * tk), 0, 0, 0, 0;Rzdot = Rzdot * OMEe_dt_B;
		Eigen::Vector3d V = Rzdot * Rx * Pgk + Rz * Rx * Vgk;
		XYZdot = Eigen::Vector3d(V(0), V(1), V(2));
	}
	else {
		XYZdot = Eigen::Vector3d(xkdot, ykdot, zkdot);
	}
	//钟差计算
	double delta_tr = F * data.e * data.RootA * sin(Ek);
	double delta_tsv = data.a0 + data.a1 * tk + data.a2 * tk * tk + delta_tr;
	GPSPVTData.XYZ = XYZ;GPSPVTData.XYZdot = XYZdot;GPSPVTData.clk = delta_tsv;GPSPVTData.health = data.health;GPSPVTData.week = data.week;GPSPVTData.second = data.toe + tk;
	if (GPSPVTData.second > 604800) {
		GPSPVTData.week++;
		GPSPVTData.second -= 604800;
	}
}

//计算双频组合的无电离层延迟
void IF(DoubleFrequencyData GPSData, SatelliteData& GPSPVTData) {
	double delta = pow(L1_f, 2) - pow(L2_f, 2);
	GPSPVTData.P = (pow(L1_f, 2) * GPSData.F1.psr - pow(L2_f, 2) * GPSData.F2.psr) / delta + 16;
	//误差传播定律计算组合方差
	GPSPVTData.P_std_2 = pow(L1_f, 2) * pow(GPSData.F1.psr_std, 2) / delta
		+ pow(L2_f, 2) * pow(GPSData.F2.psr_std, 2) / delta;
	return;
}

double GPSClk(GPSEphem data, double tk) {
	double n0 = sqrt(u_G / pow(data.A, 3));//平均角速度0.000145859rad/s
	double delta_n = data.dn;//平均角速度修正值
	double nA = n0 + delta_n;//改正平均角速度
	double e = data.e;//偏心率
	double M0 = data.M0;//参考时刻平近点角
	double Mk = M0 + nA * tk;
	double Ek = C_Ek(Mk, e);
	//钟差计算
	double delta_tr = F * data.e * sqrt(data.A) * sin(Ek);
	double delta_tsv = data.a0 + data.a1 * tk + data.a2 * tk * tk + delta_tr;
	return delta_tsv;
}
double BDSClk(BDSEphem data, double tk) {
	double A = data.RootA * data.RootA;//计算轨道长半径
	double n0 = sqrt(u_B / pow(A, 3));//平均角速度0.000145859rad/s
	double delta_n = data.dn;//平均角速度修正值
	double nA = n0 + delta_n;//改正平均角速度
	double e = data.e;//偏心率
	double M0 = data.M0;//参考时刻平近点角
	double Mk = M0 + nA * tk;
	double Ek = C_Ek(Mk, e);
	//钟差计算
	double delta_tr = F * data.e * sqrt(A) * sin(Ek);
	double delta_tsv = data.a0 + data.a1 * tk + data.a2 * tk * tk + delta_tr;
	return delta_tsv;
}

//根据观测时间获取信号发射时间(还需伪距，卫星钟差)
GPSTime GPSCalSTime(GPSTime RTime, SatelliteData& GPSData, GPSEphem GPSphem) {
	GPSTime res = RTime;double res0, tk;
	res.Seconds = RTime.Seconds - GPSData.P / Vc - GPSphem.clk;
	if (res.Seconds < 0) {
		res.Weeks--;
		res.Seconds += 604800;
	}
	GPSTime GTime;GTime.Weeks = GPSphem.week;GTime.Seconds = GPSphem.toe;
	res0 = GPSData.clk;
	tk = (res.Weeks - GTime.Weeks) * 604800 + res.Seconds - GTime.Seconds;
	GPSData.clk = GPSClk(GPSphem, tk);
	res.Seconds = RTime.Seconds - GPSData.P / Vc - GPSData.clk;
	return res;
}


//赋值单个观测值
void ValRangeData(RangeData& res, vector<unsigned char> buf, int& off) {
	res.PRN = U2(&buf[off + 0]);
	res.psr = R8(&buf[off + 4]);
	res.psr_std = R4(&buf[off + 12]);
	res.adr = R8(&buf[off + 16]);
	res.adr_std = R4(&buf[off + 24]);
	res.doppler = R4(&buf[off + 28]);
	res.locktime = R4(&buf[off + 36]);
	res.CN0 = R4(&buf[off + 32]);
	off += 44;
}
//最小二乘解算
ECEF CalPosition(SatelliteData GPSPVTData[], SatelliteData BDSPVTData[]) {
	Eigen::VectorXd Z(0, 1);//观测值矩阵（伪距）
	vector<SatelliteData>Satellite;//整合已知GPS数据
	int num = 0;
	for (int i = 0;i < 32;i++) {
		if (GPSPVTData[i].exist) {
			Z.conservativeResize(Z.rows() + 1, Z.cols());
			Z(num, 0) = GPSPVTData[i].P;
			Satellite.push_back(GPSPVTData[i]);
			//cout <<Satellite[num].P - sqrt(pow(Satellite[num].XYZ(0) - Station0.XYZ(0), 2) + pow(Satellite[num].XYZ(1) - Station0.XYZ(1), 2) + pow(Satellite[num].XYZ(2) - Station0.XYZ(2), 2)) << endl;
			num++;
		}
	}
	cout << num;
	if (num > 4) {
		Eigen::MatrixXd X(4, 1);X << 0, 0, 0, 0;//未知数矩阵（接收机位置,最后一项为接收机钟差导致的路径改正t*Vc，被当成未知数解算)
		Eigen::MatrixXd X0 = X;//迭代中的X_k-1
		Eigen::MatrixXd H(num, 4);//设计矩阵
		Eigen::MatrixXd W = Eigen::MatrixXd::Zero(num, num);//权阵
		for (int i = 0; i < num; i++)
		{
			W(i, i) = 1.00 / sqrt(Satellite[i].P_std_2);
		}
		Eigen::VectorXd S(num);
		// 写入计算距离
		int j = 0;
		do {
			X0 = X;
			for (int i = 0;i < num;i++) {
				S[i] = sqrt(pow(Satellite[i].XYZ(0) - X(0), 2) + pow(Satellite[i].XYZ(1) - X(1), 2) + pow(Satellite[i].XYZ(2) - X(2), 2));
				S[i] = S[i] + X(3);
				H(i, 3) = 1;
				Z(i, 0) = Satellite[i].P - S[i];
			}
			for (int i = 0;i < num;i++) {
				H(i, 0) = -(Satellite[i].XYZ(0) - X(0)) / S[i];
				H(i, 1) = -(Satellite[i].XYZ(1) - X(1)) / S[i];
				H(i, 2) = -(Satellite[i].XYZ(2) - X(2)) / S[i];
			}
			//参数估计:X_L=(H_T*W*H)_1*H_T*W*Z
			X = X + (H.transpose() * W * H).inverse() * H.transpose() * W * Z;
			j++;
		} while (abs(X(0) - X0(0)) > 0.01);
		ECEF res;res.exist = 1;
		res.XYZ = X.block(0, 0, 3, 1);
		std::cout << setprecision(10) << "Position: " << res.XYZ.transpose() << endl;
		//BLH S1= XYZToBLH(S0);cout<<S1.Latitude*180/PI<<" "<<S1.Longitude*180/PI<<" "<<S1.Height<<endl;//BLH坐标输出
		return res;
	}
}
//单点定位与测速函数
ECEF SPP(vector<unsigned char> buf, header Header, GPSEphem GPS[], BDSEphem BDS[]) {
	//cout << dec << Header.BodyLength << endl;
	//获取卫星信号观测值总数
	RangeData Rdata, F1, F2;
	GPSTime RTime;RTime.Weeks = Header.week;RTime.Seconds = Header.ms * 1e-3;
	SatelliteData GPSPVTData[32]; SatelliteData BDSPVTData[62];
	SatelliteData PVT;PVT.exist = 1;
	double tk;
	int status;
	int off = 28;
	int num = I4(&buf[off]);off += 4;
	int Freq;
	for (int i = 0;i < num;i++) {
		//状态字解析
		status = I4(&buf[off + 40]);
		RangeDataStatus Status = GetStatus(status);
		//cout << "System: " << Status.sys << " Type: " << Status.type << " Track: " << Status.track << " Plock: " << Status.plock << " Parity: " << Status.parity << " Clock: " << Status.clock << " Halfc: " << Status.halfc << endl;
		ValRangeData(Rdata, buf, off);
		if (Status.sys == 0)
		{
			switch (Status.type) //本次实习使用的是F1C和F2P(Y)
			{
			case 0:  Freq = 0; break;   // L1C/A
			case 9:  Freq = 1; break;   // L2P(Y),semi-codeless
			default: Freq = -1; break;
			}
			if (Freq == 0) {
				F1 = Rdata;
			}
			if (Freq == 1) {
				int n = F1.PRN - 1;
				F2 = Rdata;if (F2.PRN != F1.PRN || !GPS[n].exist)continue;//双频观测数据必须来自同一颗卫星
				DoubleFrequencyData GPSData;GPSData.F1 = F1;GPSData.F2 = F2;
				IF(GPSData, PVT);//计算消电离层组合伪距
				//计算信号发射时间，并根据发射时间和卫星星历计算卫星位置和钟差
				GPSTime STime;STime = GPSCalSTime(RTime, PVT, GPS[n]);
				tk = (STime.Weeks - GPS[n].week) * 604800 + STime.Seconds - GPS[n].toe;
				if (abs(tk) > 7200)continue;
				CalGPS(GPS[n], tk, PVT);
				GPSPsrCorrection(PVT, Station0, GPS[n]);//伪距改正
				EarthRotation(PVT);//地球自转改正
				GPSPVTData[n] = PVT;//将tk时刻计算结果存入GPSPVTData数组
			}
		}
		else if (Status.sys == 4)
		{
			switch (Status.type)  //本次实习使用的是B1I和B3I
			{
			case 0: Freq = 0; break;   // B1I D1
			case 2: Freq = 1; break;   // B3I D1
			case 4: Freq = 0; break;   // B1I D2
			case 6: Freq = 1; break;   // B3I D2
			default: Freq = -1; break;
			}
			if (Freq == 0) {
				F1 = Rdata;
			}
			if (Freq == 1) {
				int n = F1.PRN - 1;
				F2 = Rdata;if (F2.PRN != F1.PRN || !BDS[n].exist)continue;//双频观测数据必须来自同一颗卫星
				DoubleFrequencyData BDSData;BDSData.F1 = F1;BDSData.F2 = F2;
				IF(BDSData, PVT);//计算消电离层组合伪距
				CalBDS(BDS[n], tk, PVT);
				BDSPVTData[n] = PVT;//将tk时刻计算结果存入BDSPVTData数组
			}
		}

	}
	return  CalPosition(GPSPVTData, BDSPVTData);//计算接收机位置
}

int main() {
	string filename = "NovatelOEM20211114-01.log";
	// 以二进制方式打开文件用于读取
	ifstream datafile(filename, ios::binary);
	if (!datafile) {
		cerr << "Failed to open file: " << filename << '\n';
		return 1;
	}
	header Head;
	GPSEphem GPS[32];BDSEphem BDS[62];
	GPSEphem GPSData;
	BDSEphem BDSData;
	size_t BodyLength;
	vector<unsigned char> buf;
	vector<unsigned char> body;
	ECEF res;
	/*	ofstream RangeFile("RangeFile.txt");//双频观测数据文件
	ofstream PVTFile("PVTFile.txt");//广播星历计算结果
	PVTFile << "PRN POS_X(m) POS_Y(m) POS_Z(m) VEL_X(m/s) VEL_Y(m/s) VEL_Z(m/s) CLK(us) CLK_VEL(us/sec)\n ";
	PVTFile << "2021 11 14  7 25\n";
*/
//while (datafile.peek() != std::char_traits<char>::eof()) {
	for (int i = 0;i < 300;i++) {
		ReadBytes(datafile, buf, 28); //读取消息头
		Head = HeaderData(buf);
		// 读取消息主体，并将它们放在buf的消息头数据之后
		BodyLength = ReadBytes(datafile, body, Head.BodyLength);
		buf.resize(28 + BodyLength);
		memcpy(buf.data() + 28, body.data(), BodyLength);
		if (!CRCExam(datafile, buf)) {
			cout << "CRCExam wrong" << endl;
			continue;
		}
		switch (Head.type) {
		case ID_RANGE:
			cout << i << ":" << Head.week << " " << Head.ms / 1000 << endl;
			//GTime.Weeks = Head.week;GTime.Seconds = Head.ms * 1e-3;	cout << i<<" "<<GTime.Weeks << " " << GTime.Seconds << endl;	/*RangeCTime = GPSTtoCommonT(GTime);RangeFile << RangeCTime.Years << " " << setw(2) << setfill('0') << RangeCTime.Months << " " << setw(2) << setfill('0') << RangeCTime.Days << " " << setw(2) << setfill('0') << RangeCTime.Hours << " " << setw(2) << setfill('0') << RangeCTime.Minutes << " " << fixed << setprecision(5) << RangeCTime.Seconds << endl;RangeRead(buf, Head, RangeFile);*/
			res = SPP(buf, Head, GPS, BDS);
			break;
		case ID_GPSEPHEM:
			GPSData = GPSEphemRead(buf, Head);GPSData.exist = 1;
			GPSData.clk = GPSClk(GPSData, 0);
			GPS[GPSData.PRN - 1] = GPSData;//将星历数据存入GPS数组，PRN从1开始
			break;
		case ID_BDSEPHEMRIS:
			BDSData = BDSEphemRead(buf, Head);BDSData.exist = 1;
			BDSData.clk = BDSClk(BDSData, 0);
			BDS[BDSData.PRN - 1] = BDSData;//将星历数据存入BDS数组，PRN从1开始
			break;
		default:
			break;
		}
	}
}

