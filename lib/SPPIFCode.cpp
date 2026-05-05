

#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

#define debug 1

void SPPIFCode::solve(ObsData &obsData) {
    //----------------------
    // 去掉通道号，C1W, C1C => C1;
    // 后面computeSatPos里用与通道号无关的观测值计算卫星发射时刻位置
    //----------------------
    convertObsType(obsData);

    if(debug)
    {
        cout << "after convertObsType" << endl;
        cout << obsData << endl;
    }

    // 计算IF组合
    computeIF(obsData);

    // 计算发射时刻卫星位置（参考框架为时刻的）
    satXvtTransTime = computeSatPos(obsData);
    if(debug)
    {
        cout << "satXvtTransTime" << CommonTime2CivilTime(obsData.epoch) << endl;
        for(auto sx: satXvtTransTime)
        {
            cout << sx.first  << endl;
            cout << sx.second << endl;
        };
    }

    //----------------------
    // 得到卫星发射时刻位置和钟差、相对论和TGD后，改正观测值延迟，并更新C1/C2等观测值
    //----------------------
    // todo:
    // correctTGD(obsData);

    xyz = obsData.antennaPosition;
    dxyz = {100, 100, 100};

    int iter(0);
    while (true) {

        satXvtRecTime = earthRotation(xyz, satXvtTransTime);

        if(debug)
        {
            cout << "satXvtRecTime" << endl;
            for(auto sx: satXvtRecTime)
            {
                cout << sx.first << " xvt:" << endl;
                cout << sx.second << endl;
            };
        }

        // step 1: 确定观测值和未知参数的纬数
        // 根据数据结构中已经有的satTypePrefitData, satTypeVarCoeffData;
        // 得到numObs, numUnk的数值
        int numSats = obsData.satTypeValueData.size();

        // 这里应该抛出异常，而不是break，因为无法解算，所以后续rtk也不能算，
        // 所以在rtk的主程序里捕获这个异常，然后再continue下一个历元；
        // 如果break了，就不知道问题在哪里了
        if (numSats < 4 ) {
            SVNumException e("num of satellites is less than 4");
            throw(e);
        }


        // 地球表面才计算高度角和大气改正
        if(std::abs(xyz.norm() - RadiusEarth) < 100000.0)
        {
            satElevData.clear();
            satAzimData.clear();
            if(debug)
                cout << "computeElevAzim" << endl;

            computeElevAzim(xyz, satXvtRecTime,satElevData,satAzimData);

            if(debug)
            {
                cout << "satElevData:" << endl;
                cout << satElevData << endl;
            }

            // todo:
            // computeIonoDelay();
            // computeTropDealy();
        }

        equSys = linearize(xyz, satXvtRecTime, satElevData, obsData);

        if(debug)
            cout << "afte linearize:" << endl;

        // 如果是基准站，完成线性化后就退出
        // 因为基准站位置是准确的
        if(!isRover)
            break;

        solverLsq.solve(equSys);
        dxyz = solverLsq.getdxyz();

        xyz += dxyz;
        cout << "iteration:" << iter
        << "dxyz:" << dxyz.transpose()
        << "xyz:" << xyz.transpose() << endl;

        // convergence threshold
        if (dxyz.norm() < 0.1) {
            break;
        }

        if (iter > 10) {
            InvalidSolver e("too many iterations");
            throw(e);
        }
        iter++;

    }

    result.xyz = xyz;
}


std::map<SatID, PVT> SPPIFCode::computeSatPos(ObsData &obsData) {
    std::map<SatID, PVT> satXvtData;
    SatIDSet satRejectedSet;
    CommonTime time = obsData.epoch;
    // Loop through all the satellites
    for (auto stv: obsData.satTypeValueData) {
        SatID sat(stv.first);
        PVT xvt;
        // compute satellite ephemeris at transmitting time
        // Scalar to hold temporal value
        double obs(0.0);
        std::string_view codeType;
        if (sat.system == 'G') {
            codeType = "C1";
        }
        // todo
        // 请增加bds或其他系统的观测值选择
        else {
            satRejectedSet.insert(sat);
            continue;
        }

        // code obs
        try {
            obs = stv.second.at(codeType);
            if(debug)
                cout << "sat:" << sat << "obs:" << codeType << "value:" << obs << endl;
        }
        catch (...) {
            satRejectedSet.insert(sat);
            continue;
        }

        // now, compute xvt
        try {
            xvt = computeAtTransmitTime(time, obs, sat);
        }
        catch (InvalidRequest &e) {
            satRejectedSet.insert(sat);
            continue;
        }
        satXvtData[sat] = xvt;
    }

    // remove bad sat;
    for (auto sat: satRejectedSet) {
        obsData.satTypeValueData.erase(sat);
    }

    return satXvtData;

};

PVT SPPIFCode::computeAtTransmitTime(const CommonTime &tr,
                                     const double &pr,
                                     const SatID &sat)
noexcept(false) {
    PVT xvt;

    CommonTime tt;
    CommonTime transmit = tr;

    transmit -= pr / C_MPS;
    tt = transmit;

    // 这里也可以用while循环来替换这里的迭代次数
    for (int i = 0; i < 2; i++) {
        if (pEphStore != NULL) {
            xvt = pEphStore->getXvt(sat, tt);

        }
        tt = transmit;
        tt -= xvt.clockBias + xvt.relativityCorrection;
    }
    return xvt;
};

void SPPIFCode::convertObsType(ObsData &obsData) {

    SatTypeValueMap stvData;
    for (const auto& sd: obsData.satTypeValueData) {
        TypeValueMap tvData;
        for (auto td: sd.second) {
            tvData[td.first.substr(0, 2)] = td.second;
        }
        stvData[sd.first] = tvData;
    }

    // 替代
    obsData.satTypeValueData = stvData;
};

void SPPIFCode::computeIF(ObsData &obsData) {
    SatIDSet satRejectedSet;
    // Loop through all the satellites
    for (auto &stv: obsData.satTypeValueData) {
        char sys = stv.first.system;
        // get type for current system
        std::pair<string, string> ifPair;
        try {
            ifPair = ifCodeTypes.at(sys);
        }
        catch (...) {
            satRejectedSet.insert(stv.first);
        }

        // if组合的具体公式为：
        // if12 = (f1^2*P1 - f2^2*P2)/(f1^2-f2^2);
        cout << "computeIF:" << "sys:"
        << sys << "type1:"
        << ifPair.first
        << "type2:"
        << ifPair.second << endl;

        double f1 = getFreq(sys, ifPair.first);
        double f2 = getFreq(sys, ifPair.second);
        if(debug)
        {
            cout << "f1:" << f1 << "f2" << f2 << endl;
        }

        // 提取观测值
        // TypeID type1, type2;
        double value1, value2, ifValue;

        try {
            value1 = stv.second.at(ifPair.first);
            value2 = stv.second.at(ifPair.second);
            ifValue = (f1 * f1 * value1 - f2 * f2 * value2) / (f1 * f1 - f2 * f2);

            if (debug) {
                cout << "value1:" << value1 << endl;
                cout << "value2:" << value2 << endl;
                cout << "ifValue:" << ifValue << endl;
            }
            string ifCodeStr = "CC" + ifPair.first.substr(1, 1) + ifPair.second.substr(1, 1);

            stv.second[ifCodeStr] = ifValue;
        }
        catch (...) {
            satRejectedSet.insert(stv.first);
        }
    }

    // remove bad sat;
    for (auto sat: satRejectedSet) {
        obsData.satTypeValueData.erase(sat);
    }


};

std::map<SatID, PVT> SPPIFCode::earthRotation(Eigen::Vector3d &xyz,
                                              std::map<SatID, PVT> &satXvtTransTime) {

    std::map<SatID, PVT> satXvtRecTime;
    for(auto stv: satXvtTransTime) {
        SatID sat = stv.first;
        XYZ xyzSat(stv.second.p);
        double dt = (xyzSat - xyz).norm() / C_MPS;

        double wt(0.0);
        wt = OMEGA_EARTH * dt;

        // todo:
        // Eigen中Vector3d是不是支持坐标旋转？
        // 请查询并修改

        double xSat, ySat, zSat;
        xSat = stv.second.p[0];
        ySat = stv.second.p[1];
        zSat = stv.second.p[2];

        double xSatRot(0.0), ySatRot(0.0);
        xSatRot = +std::cos(wt) * xSat + std::sin(wt) * ySat;
        ySatRot = -std::sin(wt) * xSat + std::cos(wt) * ySat;

        XYZ xyzRecTime;
        xyzRecTime[0] = xSatRot;
        xyzRecTime[1] = ySatRot;
        xyzRecTime[2] = zSat; // z轴不变

        double vxSat, vySat, vzSat;
        vxSat = stv.second.v[0];
        vySat = stv.second.v[1];
        vzSat = stv.second.v[2];

        double vxSatRot(0.0), vySatRot(0.0);
        vxSatRot = +std::cos(wt) * vxSat + std::sin(wt) * vySat;
        vySatRot = -std::sin(wt) * vxSat + std::cos(wt) * vySat;

        XYZ velRecTime;
        velRecTime[0] = vxSatRot;
        velRecTime[1] = vySatRot;
        velRecTime[2] = vzSat; // 不变

        // 替换位置和速度，得到旋转后的卫星产品
        PVT xvtRecTime = stv.second;
        xvtRecTime.p = xyzRecTime;
        xvtRecTime.v = velRecTime;

        satXvtRecTime[sat] = xvtRecTime;
    };

    return satXvtRecTime;
};

void SPPIFCode::computeElevAzim(Eigen::Vector3d& xyz,
                                std::map<SatID,PVT> & satXvt,
                                SatValueMap& tempElevData,
                                SatValueMap& tempAzimData
                                 )
{

    for(auto sx: satXvt)
    {
        SatID sat = sx.first;

        XYZ satXYZ = sx.second.p;

        // elevation
        double elev(0.0);
        double azim(0.0);
        elev = elevation(xyz, satXYZ);
        azim = azimuth(xyz, satXYZ);


        tempElevData[sat] = elev;
        tempAzimData[sat] = azim;
    }
};


EquSys SPPIFCode::linearize(Eigen::Vector3d& xyz,
                                  std::map<SatID,PVT>& satXvtRecTime,
                                  SatValueMap& satElevData,
                                  ObsData &obsData) {
    EquSys equSysTemp;
    equSysTemp.station = obsData.station;

    VariableSet varSetTemp;
    for (auto stv: obsData.satTypeValueData) {
        SatID sat = stv.first;

        double elev = satElevData.at(sat);
        double elevRad = elev*DEG_TO_RAD;

        // 跳过这颗卫星，不形成观测方程和未知参数数据
        if(elev < cutOffElev)
        {
            continue;
        }

        // 这里卫星的位置，应该是地球自转以后的卫星位置
        XYZ satXYZ;
        satXYZ = satXvtRecTime[sat].p;

        // rho
        double rho(0.0);
        rho = ( satXYZ - xyz).norm();
        if(debug)
        {
            cout << "Sat:" << sat << endl;
            cout << "xyz:" << xyz << endl;
            cout << "satXYZ:" << satXYZ << endl;
        }

        // convert unit form second to meter
        double clkBias = satXvtRecTime.at(sat).clockBias * C_MPS;
        double relCorr = satXvtRecTime.at(sat).relativityCorrection * C_MPS;



        double slantTrop(0.0);
        // to do
        // extract slant trop

        // partials
        Eigen::Vector3d cosines;
        cosines[0] = (xyz.x() - satXYZ[0]) / rho;
        cosines[1] = (xyz.y() - satXYZ[1]) / rho;
        cosines[2] = (xyz.z() - satXYZ[2]) / rho;

        // todo
        // 请补充rhoDot，用于后续的单点测速

        // 首先定义所有可能的未知参数
        Variable dx(obsData.station, Parameter::dX);
        Variable dy(obsData.station, Parameter::dY);
        Variable dz(obsData.station, Parameter::dZ);
        Variable cdtGPS(obsData.station, Parameter::cdt);

        // 对每个观测值，都需要存储对应的未知参数及其偏导数
        for (auto tv: stv.second)
        {
            if (sat.system=='G' && tv.first == "CC12" )
            {
                EquID equID = EquID(sat, tv.first);

                //>> 先验残差
                double prefit;
                double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                prefit = tv.second - computedObs;

                if (debug) {
                    cout << "obs:" << tv.second << endl;
                    cout << "rho:" << rho << endl;
                    cout << "clkBias:" << clkBias << endl;
                    cout << "relCorr:" << relCorr << endl;
                    cout << "slantTrop" << slantTrop << endl;
                }

                equSysTemp.obsEquData[equID].prefit = prefit;
                equSysTemp.obsEquData[equID].varCoeffData[dx] = cosines[0];
                equSysTemp.obsEquData[equID].varCoeffData[dy] = cosines[1];
                equSysTemp.obsEquData[equID].varCoeffData[dz] = cosines[2];
                equSysTemp.obsEquData[equID].varCoeffData[cdtGPS] = 1.0;


                // Compute the weight according to elevation
                double weight;
                if(elev >= 30){
                    weight = 1.0 / (sigIFCode * sigIFCode);
                }
                else
                {
                    weight = 1.0 / (sigIFCode * sigIFCode) * std::pow(std::sin(elevRad), 2);
                }

                equSysTemp.obsEquData[equID].weight = weight; // IF组合方差为1.0m

                // 把当前观测方程未知参数插入到总体的未知参数
                varSetTemp.insert(dx);
                varSetTemp.insert(dy);
                varSetTemp.insert(dz);
                varSetTemp.insert(cdtGPS);
            }
        }
    }

    equSysTemp.varSet = varSetTemp;

    return equSysTemp;
};