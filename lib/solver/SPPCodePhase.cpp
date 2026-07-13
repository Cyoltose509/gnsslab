#include "SPPCodePhase.h"
#include "CoordConvert.h"
#include "Const.h"

void SPPCodePhase::solve(ObsData &obsData) {
    if (useKalman_) solveKalman(obsData);
    else solveLSQ(obsData);
}

// 用干净的 IF-code 单历元解作为可靠参考（位置+钟差）
bool SPPCodePhase::computeCodeReference(const ObsData &rawObs, XYZ &outXyz, std::map<char, double> &outCdt) {
    SPPIFCode ifSpp;
    ifSpp.setIFCodeTypes(ifCodeTypes);
    ifSpp.setEphemerisTable(ephTable);
    ifSpp.setFrame(frame);
    ObsData obsIF = rawObs;                 // 未 strip/normalize 的原始观测
    if (obsIF.antennaPosition.norm() < 1000.0 && lastGoodXyz_.norm() > 1000.0)
        obsIF.antennaPosition = lastGoodXyz_;  // 冷启动时给 IF-code 一个合理初值
    try {
        ifSpp.preprocess(obsIF);
        ifSpp.solve(obsIF);
        if (ifSpp.result.numSats >= 4) {
            outXyz = ifSpp.result.xyz;
            outCdt['G'] = ifSpp.getClockBias('G');
            outCdt['C'] = ifSpp.getClockBias('C');
            return true;
        }
    } catch (...) {}
    return false;
}

void SPPCodePhase::solveLSQ(ObsData &obsData) {
    result.reset(); lastWStats_.clear(); rClockBias.clear();
    vel = Vector3d(0,0,0); rClockDrift = 0.0;

    xyz = obsData.antennaPosition;
    stripObsType(obsData);
    normalizePhase(obsData);
    // strip 后的 obsData 仍含 C1/C2/... 双频伪距，可直接供 IF-code 参考解使用（无需额外整份拷贝）
    // 无先验坐标时（如 OEM7 前几个历元），用 IF-code / 上一历元作初值
    XYZ xyz_code; std::map<char, double> cdt_code; bool haveCode = false;
    if (xyz.norm() < 1000.0) {
        haveCode = computeCodeReference(obsData, xyz_code, cdt_code);
        if (haveCode) xyz = xyz_code;
        else if (lastGoodXyz_.norm() > 1000.0) xyz = lastGoodXyz_;
    }
    int iter(0);
    while (iter < 3) {
        computeSatPos(obsData); earthRotation();
        if (obsData.satTypeValueData.size() < 4) throw SVNumException("sats<4");
        satElevData.clear(); satAzimData.clear(); computeElevAzim();
        linearize(obsData, iter);
        if (posEquations.obsEquData.size() < 4) throw SVNumException("obs<4");
        posSolver.solve(posEquations);
        Vector3d d{posSolver.getSolution(Parameter::dX),posSolver.getSolution(Parameter::dY),posSolver.getSolution(Parameter::dZ)};
        xyz += d;
        for (char s:activeSystems) if (auto it=sysCdtParam.find(s);it!=sysCdtParam.end()) rClockBias[s]+=posSolver.getSolution(it->second);
        if (iter>=1 && d.norm()<1e-1) break;
        buildResidualMap(); iter++;
    }
    if (iter>=3) throw InvalidSolver("too many iterations");
    // 安全网：单历元浮点模糊度对周跳/粗差敏感，可能大幅发散。
    // 触发判据用“时间连续性”——与上一历元成功位置比较（与坐标框架无关，静止/缓动时几乎不触发），
    // 仅在位置突跳超过阈值时才做一次干净的 IF-code 解作回退。
    // 这样正常历元零额外开销，只有极少数发散历元才付出 IF-code 计算成本。
    if (lastGoodXyz_.norm() > 1000.0) {
        if ((xyz - lastGoodXyz_).norm() > FALLBACK_THRESH) {
            if (!haveCode) haveCode = computeCodeReference(obsData, xyz_code, cdt_code);
            if (haveCode && (xyz - xyz_code).norm() > FALLBACK_THRESH) {
                xyz = xyz_code; rClockBias = cdt_code;
            }
        }
    } else {
        // 首历元尚无历史位置：用 BESTPOS/近似坐标兜底（仅一次）
        const XYZ ref = obsData.antennaPosition;
        if (ref.norm() > 1000.0 && (xyz - ref).norm() > FALLBACK_THRESH) {
            if (!haveCode) haveCode = computeCodeReference(obsData, xyz_code, cdt_code);
            if (haveCode && (xyz - xyz_code).norm() > FALLBACK_THRESH) {
                xyz = xyz_code; rClockBias = cdt_code;
            }
        }
    }
    if (xyz.norm() > 1000.0) lastGoodXyz_ = xyz;
    if (velEquations.obsEquData.size()>=4) {
        velSolver.solve(velEquations);
        vel += Vector3d(velSolver.getSolution(Parameter::dVX),velSolver.getSolution(Parameter::dVY),velSolver.getSolution(Parameter::dVZ));
        rClockDrift += velSolver.getSolution(Parameter::cdtr_dot);
    }
    if (xyz.norm() > 1000.0) lastGoodXyz_ = xyz;
    getResult();
}

void SPPCodePhase::solveKalman(ObsData &obsData) {
    result.reset(); lastWStats_.clear(); rClockBias.clear();
    vel = Vector3d(0,0,0); rClockDrift = 0.0;

    xyz = obsData.antennaPosition;
    stripObsType(obsData);
    normalizePhase(obsData);
    // 无先验坐标时用 IF-code / 上一历元作初值（strip 后的伪距可直接供 IF-code 参考解）
    XYZ xyz_code; std::map<char, double> cdt_code; bool haveCode = false;
    if (xyz.norm() < 1000.0) {
        haveCode = computeCodeReference(obsData, xyz_code, cdt_code);
        if (haveCode) xyz = xyz_code;
        else if (lastGoodXyz_.norm() > 1000.0) xyz = lastGoodXyz_;
    }
    satRejected.clear(); lastWStats_.clear();
    int iter(0);
    while(iter<3) {
        computeSatPos(obsData); earthRotation();
        satElevData.clear(); satAzimData.clear(); computeElevAzim();
        linearize(obsData, iter);
        if (posEquations.obsEquData.size()<4) throw SVNumException("obs<4");
        VariableDataMap emptyCs;
        if (iter==0) solverKalman_.timeUpdate(posEquations.varSet, &emptyCs);
        solverKalman_.measUpdate(posEquations);
        Vector3d d = solverKalman_.getdxyz(); xyz += d;
        for (char s:activeSystems) if (auto it=sysCdtParam.find(s);it!=sysCdtParam.end())
            rClockBias[s] += solverKalman_.getSolution(it->second, solverKalman_.currentUnkSet, solverKalman_.getState());
        if (d.norm()<1e-2) break;
        buildResidualMap(); iter++;
    }
    // 安全网：用“时间连续性”检测单历元浮点解是否发散（与上一历元成功位置比较，
    // 与坐标框架无关，静止/缓动时几乎不触发）；仅在位置突跳超过阈值时才做一次
    // 干净的 IF-code 解作回退，正常历元零额外开销。
    // 一旦检测到突跳，立即重置卡尔曼滤波器内部状态，否则其位置状态会持续携带旧偏置，
    // 下一历元又会重新发散（表现为反复 ~8m 跳变）。
    if (lastGoodXyz_.norm() > 1000.0) {
        if ((xyz - lastGoodXyz_).norm() > FALLBACK_THRESH) {
            if (!haveCode) haveCode = computeCodeReference(obsData, xyz_code, cdt_code);
            if (haveCode && (xyz - xyz_code).norm() > FALLBACK_THRESH) {
                xyz = xyz_code; rClockBias = cdt_code;
            }
            solverKalman_.reset();
        }
    } else {
        const XYZ ref = obsData.antennaPosition;
        if (ref.norm() > 1000.0 && (xyz - ref).norm() > FALLBACK_THRESH) {
            if (!haveCode) haveCode = computeCodeReference(obsData, xyz_code, cdt_code);
            if (haveCode && (xyz - xyz_code).norm() > FALLBACK_THRESH) {
                xyz = xyz_code; rClockBias = cdt_code;
            }
        }
    }
    if (xyz.norm() > 1000.0) lastGoodXyz_ = xyz;
    if (velEquations.obsEquData.size()>=4) {
        velSolver.solve(velEquations);
        vel += Vector3d(velSolver.getSolution(Parameter::dVX),velSolver.getSolution(Parameter::dVY),velSolver.getSolution(Parameter::dVZ));
        rClockDrift += velSolver.getSolution(Parameter::cdtr_dot);
    }
    getResult();
}

void SPPCodePhase::getResult() {
    result.blh = XYZtoBLH(xyz, frame);
    if (useKalman_) {
        auto &P = solverKalman_.getCovMatrix();
        result.xyz=xyz; result.vel=vel;
        result.numSats=(int)satPVTRecTime.size();
        if (P.rows() >= 3) {
            result.sigmaXYZ = Eigen::Vector3d(sqrt(P(0,0)), sqrt(P(1,1)), sqrt(P(2,2)));
            result.sigmaP = sqrt(P(0,0)+P(1,1)+P(2,2));
        }
        computeDOP();
        int io=0; auto &pr = solverKalman_.getPostfitResidual();
        for(auto &[eid,ed]:posEquations.obsEquData) {
            if(io<(int)pr.size()){
                double v = pr[io++];
                // 仅保留伪距(码)方程残差：相位残差被浮点模糊度吸收≈0，无 QC 意义
                if (!eid.obsType.empty() && eid.obsType[0] == 'C') result.postRes[eid.sat] = v;
            } else io++;
        }
    } else {
        auto &pD=posSolver.covMatrix; result.xyz=xyz; result.vel=vel;
        result.numSats=(int)satPVTRecTime.size();
        if (pD.rows() >= 3) {
            result.sigmaXYZ = Eigen::Vector3d(sqrt(pD(0,0)), sqrt(pD(1,1)), sqrt(pD(2,2)));
            result.sigmaP = sqrt(pD(0,0)+pD(1,1)+pD(2,2));
        }
        computeDOP();
        int io=0; for(auto &[eid,ed]:posEquations.obsEquData){
            double v = posSolver.v[io++];
            // 仅保留伪距(码)方程残差：相位残差被浮点模糊度吸收≈0，无 QC 意义
            if (!eid.obsType.empty() && eid.obsType[0] == 'C') result.postRes[eid.sat] = v;
        }
    }
    if (result.vel.norm() > 1e-12 && velSolver.covMatrix.rows() >= 4) {
        auto& vD = velSolver.covMatrix;
        result.sigmaVel = Eigen::Vector3d(sqrt(vD(0,0)), sqrt(vD(1,1)), sqrt(vD(2,2)));
        result.sigmaV = result.sigmaVel.norm();
    }
}

void SPPCodePhase::computeSatPos(ObsData &obsData) {
    satPVTTransTime.clear();
    for(auto &[sat,cl]:obsData.satTypeValueData){
        if(sat.system!='G'&&sat.system!='C')continue;
        Ephemeris *eph=ephTable?ephTable->find(sat,obsData.epoch):nullptr;
        if(!eph){auto it=ephMap.find(sat);if(it==ephMap.end())continue;eph=it->second;}
        double pr=0; for(auto&[k,v]:cl)if(k.size()>=2&&k[0]=='C'&&k[1]>='1'&&k[1]<='9'&&v>0){pr=v;break;}
        if(pr<=0)continue;
        double t=(pr-rClockBias[sat.system])/C_MPS;
        CommonTime te=obsData.epoch; te.m_sod-=t;
        satPVTTransTime[sat]=eph->svPVT(std::move(te));
    }
}

void SPPCodePhase::stripObsType(ObsData &obsData){
    SatTypeValueMap nm; for(auto&[s,t]:obsData.satTypeValueData){TypeValueMap nt; for(auto&[k,v]:t)if(k.size()>=2)nt[k.substr(0,2)]=v; nm[s]=std::move(nt);}
    obsData.satTypeValueData=std::move(nm);
}

void SPPCodePhase::normalizePhase(ObsData &obsData){
    for(auto&[sat,tv]:obsData.satTypeValueData){
        for(auto&[key,val]:tv){
            if(key.size()>=2&&key[0]=='L'&&val!=0){
                double f=getFreq(sat.system,key);
                if(f>0){ double cyc=std::abs(val); val=cyc*C_MPS/f; }
            }
        }
    }
}

void SPPCodePhase::linearize(ObsData &obsData, int iter){
    posEquations.reset(); velEquations.reset();
    posEquations.station = velEquations.station = obsData.station;
    Variable dx(obsData.station, Parameter::dX), dy(obsData.station, Parameter::dY), dz(obsData.station, Parameter::dZ);
    std::map<char, Variable> cv;
    for (auto &[s, p] : sysCdtParam) cv.try_emplace(s, obsData.station, p);

    // ---- 基于上轮 LSQ 残差的 Baarda w-检验剔除粗差/周跳异常的卫星 ----
    SatID outlierSat; double worstW = 0.0;
    if (iter >= 2) {
        for (auto const &[sat, cl] : obsData.satTypeValueData) {
            if (satRejected.count(sat)) continue;
            if (auto itW = lastWStats_.find(sat); itW != lastWStats_.end() && itW->second > worstW) {
                worstW = itW->second;
                outlierSat = sat;
            }
        }
    }
    const bool rejectOutlier = worstW > W_THRESHOLD;

    double rh = 0;
    if (xyz.norm() > 1000.0) rh = XYZtoBLH(xyz, frame)[2];
    activeSystems.clear();

    for (auto &[sat, cl] : obsData.satTypeValueData) {
        if (sat.system != 'G' && sat.system != 'C') continue;
        if (satRejected.count(sat)) continue;
        if (rejectOutlier && sat == outlierSat) { satRejected.insert(sat); continue; }
        if (!satPVTRecTime.count(sat)) { satRejected.insert(sat); continue; }
        auto &pvt = satPVTRecTime.at(sat);
        double r = pvt.p.norm();
        if (r < 15e6 || r > 45e6) { satRejected.insert(sat); continue; }
        double elev = PI * 0.5;
        if (auto ie = satElevData.find(sat); ie != satElevData.end()) elev = ie->second;
        if (xyz.norm() > 1000.0 && elev < cutOffElev) { satRejected.insert(sat); continue; }
        auto has = [&](const std::string &k) { auto i = cl.find(k); return i != cl.end() && i->second != 0.0; };
        std::string c1, c2, p1, p2;
        if (sat.system == 'G') {
            if (has("C1") && has("C2") && has("L1") && has("L2")) { c1 = "C1"; c2 = "C2"; p1 = "L1"; p2 = "L2"; }
            else continue;
        } else {
            if (has("C1") && has("C6") && has("L1") && has("L6")) { c1 = "C1"; c2 = "C6"; p1 = "L1"; p2 = "L6"; }
            else if (has("C2") && has("C6") && has("L2") && has("L6")) { c1 = "C2"; c2 = "C6"; p1 = "L2"; p2 = "L6"; }
            else continue;
        }
        if (!has(c1) || !has(c2) || !has(p1) || !has(p2)) continue;

        double f1 = getFreq(sat.system, c1), f2 = getFreq(sat.system, c2);
        double alpha = f1 * f1 / (f1 * f1 - f2 * f2);
        double beta = -f2 * f2 / (f1 * f1 - f2 * f2);
        double lambda_IF = C_MPS / (alpha * f1 + beta * f2);

        double C1v = cl.at(c1), C2v = cl.at(c2);
        double L1v = cl.at(p1), L2v = cl.at(p2);
        double C_IF = alpha * C1v + beta * C2v;
        double L_IF = alpha * L1v + beta * L2v;

        double rho = (pvt.p - xyz).norm(); if (rho < 1.0) rho = 2e7;
        Vector3d los = {-(pvt.p[0] - xyz[0]) / rho, -(pvt.p[1] - xyz[1]) / rho, -(pvt.p[2] - xyz[2]) / rho};
        double trop = (rh > 0) ? tropoHopfield(rh, elev) : 0;
        double dts = pvt.clockBias * C_MPS, rel = pvt.relativityCorrection * C_MPS;
        double range = rho + rClockBias[sat.system] - dts - rel + trop;

        // IF 模糊度 B_IF = α·λ1·N1 + β·λ2·N2 是实数（α,β 非整数），不是整数。
        // 由 L_IF = range + B_IF 且 C_IF ≈ range 可得 B_IF ≈ L_IF - C_IF，
        // 故用 (L_IF - C_IF)/λ_IF 取整作为浮点模糊度初值（注意符号！）。
        double initN = std::round((L_IF - C_IF) / lambda_IF);
        if (!std::isfinite(initN)) initN = 0;

        Variable amb(obsData.station, sat, Parameter::ambiguity, ObsID(std::string(1, sat.system), "IF"));
        auto cw = [&]() { double w = 1.0 / (sigCode * sigCode); if (elev < PI / 6) w *= sin(elev) * sin(elev); return w; };
        auto pw = [&]() { double w = 1.0 / (sigPhase * sigPhase); if (elev < PI / 6) w *= sin(elev) * sin(elev); return w; };

        EquID eidC(sat, "C_IF"); EquData edC;
        edC.prefit = C_IF - range;
        edC.varCoeffData[dx] = los[0]; edC.varCoeffData[dy] = los[1]; edC.varCoeffData[dz] = los[2];
        edC.varCoeffData[cv[sat.system]] = 1.0;
        edC.weight = cw(); posEquations.obsEquData[eidC] = edC;

        EquID eidL(sat, "L_IF"); EquData edL;
        edL.prefit = L_IF - range - initN * lambda_IF;
        edL.varCoeffData[dx] = los[0]; edL.varCoeffData[dy] = los[1]; edL.varCoeffData[dz] = los[2];
        edL.varCoeffData[cv[sat.system]] = 1.0;
        edL.varCoeffData[amb] = lambda_IF;
        edL.weight = pw(); posEquations.obsEquData[eidL] = edL;

        posEquations.varSet.insert(dx); posEquations.varSet.insert(dy); posEquations.varSet.insert(dz);
        posEquations.varSet.insert(cv[sat.system]); posEquations.varSet.insert(amb);
        activeSystems.insert(sat.system);

        if (auto dit = cl.find("D1"); dit != cl.end() && dit->second != 0) {
            double lambdaV = C_MPS / f1, rhoDotObs = -lambdaV * dit->second;
            double rhoDotMod = (vel - pvt.v).dot(los) + rClockDrift;
            Variable dvx(obsData.station, Parameter::dVX), dvy(obsData.station, Parameter::dVY), dvz(obsData.station, Parameter::dVZ), dcdt(obsData.station, Parameter::cdtr_dot);
            EquID eidVel(sat, "D1"); EquData ed;
            ed.prefit = rhoDotObs - rhoDotMod;
            ed.varCoeffData[dvx] = los[0]; ed.varCoeffData[dvy] = los[1]; ed.varCoeffData[dvz] = los[2]; ed.varCoeffData[dcdt] = 1.0;
            double vw = 1.0 / (0.01 * 0.01); if (elev < PI / 6) vw *= sin(elev) * sin(elev);
            ed.weight = vw; velEquations.obsEquData[eidVel] = ed;
            velEquations.varSet.insert(dvx); velEquations.varSet.insert(dvy); velEquations.varSet.insert(dvz); velEquations.varSet.insert(dcdt);
        }
    }
}

void SPPCodePhase::computeDOP(){
    std::map<char,int> clockIdx; int nCol=3;
    for(auto&[sys,p]:sysCdtParam){clockIdx[sys]=nCol++;}
    MatrixXd N=MatrixXd::Zero(nCol,nCol);
    int ns=0;
    for(auto&[eid,ed]:posEquations.obsEquData){
        VectorXd h=VectorXd::Zero(nCol); bool hasAny=false;
        for(auto&[var,coeff]:ed.varCoeffData){
            auto p=var.getParaType();
            if(p==Parameter::dX){h(0)=coeff;hasAny=true;}
            else if(p==Parameter::dY){h(1)=coeff;hasAny=true;}
            else if(p==Parameter::dZ){h(2)=coeff;hasAny=true;}
            else if(p==Parameter::cdt){auto it=clockIdx.find('G');if(it!=clockIdx.end()){h(it->second)=coeff;hasAny=true;}}
            else if(p==Parameter::cdt2){auto it=clockIdx.find('C');if(it!=clockIdx.end()){h(it->second)=coeff;hasAny=true;}}
        }
        if(hasAny){ns++; N+=h*h.transpose();}
    }
    if(ns>=4){
        auto ldlt=N.ldlt();
        if(ldlt.info()==Eigen::Success){
            MatrixXd Q=ldlt.solve(MatrixXd::Identity(nCol,nCol));
            result.pdop=sqrt(Q(0,0)+Q(1,1)+Q(2,2));
            double g2=Q(0,0)+Q(1,1)+Q(2,2), t2=0;
            for(auto&[sys,idx]:clockIdx){g2+=Q(idx,idx); t2+=Q(idx,idx);}
            result.gdop=sqrt(g2); result.tdop=sqrt(t2);
            result.hdop=sqrt(Q(0,0)+Q(1,1)); result.vdop=sqrt(Q(2,2));
        }else{result.pdop=result.gdop=result.tdop=result.hdop=result.vdop=99.0;}
    }else{result.pdop=result.gdop=result.tdop=result.hdop=result.vdop=99.0;}
}
// force
