
#include <algorithm>
#include <cmath>

#include "Log.h"
#include "GnssEKF.h"

GnssEKF::GnssEKF() {
    firstTime = true;
}

void GnssEKF::configure(const double posSigma, const bool hasTropo, const double ambSigma, const double clockSigma) {
    posProcSigma = posSigma;
    useTropo = hasTropo;
    ambProcSigma = ambSigma;
    clockProcSigma = clockSigma;
}

void GnssEKF::reset() {
    firstTime = true;
    motionDt = 0.0;
    mCVModel = false;
    solution = Eigen::VectorXd();
    Q = Eigen::MatrixXd();
    postfitResidual = Eigen::VectorXd();
    currentUnkSet.clear();
    currentIndexData.clear();
    posIdx = {-1, -1, -1};
    velIdx = {-1, -1, -1};
    Xhat = Eigen::VectorXd();
    Dx = Eigen::MatrixXd();
    oldUnkSet.clear();
    oldIndexData.clear();
    kalmanFilter = SolverKalman{};
    savedXminus = Eigen::VectorXd();
    savedDp = Eigen::MatrixXd();
    hasSaved = false;
}

void GnssEKF::createIndex(const VariableSet &varSet) {
    // 建立索引
    currentIndexData.clear();
    posIdx = {-1, -1, -1};
    velIdx = {-1, -1, -1};
    int index = 0;
    for (const auto &var: varSet) {
        if (const int p = var.par.name(); p >= Parameter::dX && p <= Parameter::dZ) posIdx[p - Parameter::dX] = index;
        else if (p >= Parameter::dVX && p <= Parameter::dVZ) velIdx[p - Parameter::dVX] = index;
        currentIndexData[var] = index++;
    }
}

int GnssEKF::indexOf(const Variable &v) const {
    const auto it = currentIndexData.find(v);
    return it == currentIndexData.end() ? -1 : it->second;
}

void GnssEKF::setPosition(const Eigen::Vector3d &p, const double posVar) {
    if (currentUnkSet.empty()) return; // timeUpdate 尚未建索引时无效
    for (int k = 0; k < 3; ++k) {
        const int idx = posIdx[k];
        if (idx < 0) continue;
        Xhat(idx) = p[k];
        kalmanFilter.Xhat(idx) = p[k];
        kalmanFilter.Xminus(idx) = p[k];
        if (posVar > 0.0) {
            kalmanFilter.Dp(idx, idx) = posVar;
            kalmanFilter.Dx(idx, idx) = posVar;
            Dx(idx, idx) = posVar;
        }
    }
}

void GnssEKF::setStates(const std::map<Variable, double> &values,
                        const std::map<Variable, double> *vars) {
    if (currentUnkSet.empty()) return;
    for (const auto &[var, value]: values) {
        if (const int idx = indexOf(var); idx >= 0) {
            kalmanFilter.Xhat(idx) = value;
            kalmanFilter.Xminus(idx) = value;
            double v = -1.0;
            if (vars) {
                if (const auto vit = vars->find(var); vit != vars->end()) v = vit->second;
            }
            if (v > 0.0) {
                kalmanFilter.Dp(idx, idx) = v;
                kalmanFilter.Dx(idx, idx) = v;
                Dx(idx, idx) = v;
            }
        }
    }
}

void GnssEKF::setAmbiguity(const std::string &station, const std::map<SatID, double> &amb,
                           const std::vector<AmbReset> *resets) {
    if (currentUnkSet.empty()) return;
    for (const auto &[sat, value]: amb) {
        const Variable vamb = Variable::makeAmbiguity(station, sat, "IF");
        if (resets && std::none_of(resets->begin(), resets->end(),
                                   [&](const AmbReset &r) { return r.var == vamb; })) continue;
        if (const int idx = indexOf(vamb); idx >= 0) {
            kalmanFilter.Xhat(idx) = value;
            kalmanFilter.Xminus(idx) = value;
        }
    }
}

void GnssEKF::shiftState(const Variable &var, const double delta) {
    if (currentUnkSet.empty()) return;
    const int idx = indexOf(var);
    if (idx < 0) return;
    kalmanFilter.Xhat(idx) += delta;
    kalmanFilter.Xminus(idx) += delta;
}

void GnssEKF::timeUpdate(const VariableSet &varSet, const std::vector<AmbReset> &resets) {
    currentUnkSet = varSet;
    const int n = static_cast<int>(currentUnkSet.size());

    std::map<Variable, AmbReset> resetMap;
    for (const auto &r: resets) resetMap[r.var] = r;

    createIndex(currentUnkSet);
    const auto [init, proc] = varianceTables();

    if (oldUnkSet.empty()) {
        // 无先前状态可映射：按方差表全新引导
        Xhat = Eigen::VectorXd::Zero(n);
        Dx = Eigen::MatrixXd::Zero(n, n);
        for (const auto &[var, idx]: currentIndexData) {
            Dx(idx, idx) = init[static_cast<int>(var.par.name())];
            if (var.par == Parameter::ztd) Xhat(idx) = ztdInitValue;
        }
        firstTime = false;
    } else {
        // 新旧变量映射
        Eigen::VectorXd newXhat = Eigen::VectorXd::Zero(n);
        Eigen::MatrixXd newDx = Eigen::MatrixXd::Zero(n, n);

        for (const auto &[var, ci]: currentIndexData) {
            // 周跳软重置：值用指令里的初值
            if (var.par == Parameter::ambiguity) {
                if (const auto it = resetMap.find(var);
                    it != resetMap.end() && it->second.kind == AmbReset::Kind::Slip) {
                    newXhat(ci) = it->second.initValue;
                    newDx(ci, ci) = ambInitSigma * ambInitSigma;
                    continue;
                }
            }
            if (oldUnkSet.find(var) != oldUnkSet.end()) {
                const int oi = oldIndexData[var];
                newXhat(ci) = solution(oi);
                for (const auto &[v2, c2]: currentIndexData) {
                    if (oldUnkSet.find(v2) != oldUnkSet.end()) {
                        const int o2 = oldIndexData[v2];
                        newDx(ci, c2) = Q(oi, o2);
                    }
                }
            } else {
                newXhat(ci) = var.par == Parameter::ztd ? ztdInitValue : 0.0;
                newDx(ci, ci) = init[static_cast<int>(var.par.name())];
            }
        }
        Xhat = newXhat;
        Dx = newDx;
    }

    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd Dw = Eigen::MatrixXd::Zero(n, n);

    // 速度耦合模型：CV 模型，位置状态被速度预测耦合
    const bool cvModel = accProcSigma > 0.0 && motionDt > 0.0 &&
                         posIdx[0] >= 0 && posIdx[1] >= 0 && posIdx[2] >= 0 &&
                         velIdx[0] >= 0 && velIdx[1] >= 0 && velIdx[2] >= 0;
    mCVModel = cvModel;
    if (cvModel) {
        const double acc = accProcSigma;
        const double dt = motionDt;
        const double dp = 0.5 * acc * dt * dt;            // 位置增量 σ = ½·a·Δt²
        const double dv = acc * dt;                       // 速度增量 σ = a·Δt
        const double dpv = 0.5 * acc * acc * dt * dt * dt; // 位置-速度协方差 = ½·a²·Δt³
        for (int k = 0; k < 3; ++k) {
            Phi(posIdx[k], velIdx[k]) = dt;
            Dw(posIdx[k], posIdx[k]) = dp * dp;
            Dw(velIdx[k], velIdx[k]) = dv * dv;
            Dw(posIdx[k], velIdx[k]) = dpv;
            Dw(velIdx[k], posIdx[k]) = dpv;
        }
    }

    for (const auto &[var, ii]: currentIndexData) {
        const int p = var.par.name();
        if (init[p] < 0.0 || proc[p] < 0.0)
            throw InvalidRequest("GnssEKF::timeUpdate: variance table missing parameter type");
        double q = proc[p];
        if (p >= Parameter::dX && p <= Parameter::dVZ && cvModel) {
            q = Dw(ii, ii); // 用 CV 噪声
        } else if (var.par.isClock() && q == 0.0) {
            for (int j = 0; j < n; ++j) {
                Dx(ii, j) = 0.0;
                Dx(j, ii) = 0.0;
            }
            Dx(ii, ii) = init[p];
        } else if (p == Parameter::ambiguity && resetMap.count(var)) {
            q = init[p];
        }
        Dw(ii, ii) = q; // 其他噪声
    }

    kalmanFilter.reset(Xhat, Dx);
    kalmanFilter.predict(Phi, Dw);
}

/// 按当前 σ 配置一次列出全部参数类型的初始方差与过程噪声方差
GnssEKF::VarianceTable GnssEKF::varianceTables() const {
    VarianceTable v{};
    v.init.fill(kVarianceUnset);
    v.proc.fill(kVarianceUnset);
    const auto put = [&v](const int p, const double initVar, const double procVar) {
        v.init[p] = initVar;
        v.proc[p] = procVar;
    };
    const double posProcVar = posProcSigma * posProcSigma;
    const double posInitVar = posProcSigma > 0.0 ? posProcVar : 100.0 * 100.0;
    const bool isbOn = isbInitSigma > 0.0;
    const double clockInitVar = clockInitSigma * clockInitSigma;
    const double isbInitVar = isbInitSigma * isbInitSigma;
    const double clockProcVar = clockProcSigma * clockProcSigma;
    const double isbProcVar = isbProcSigma * isbProcSigma;
    const double velInitVar = velInitSigma * velInitSigma;

    put(Parameter::Unknown, 1.0E4, 0.0);
    put(Parameter::dX, posInitVar, posProcVar);
    put(Parameter::dY, posInitVar, posProcVar);
    put(Parameter::dZ, posInitVar, posProcVar);
    put(Parameter::dVX, velInitVar, 0.0);
    put(Parameter::dVY, velInitVar, 0.0);
    put(Parameter::dVZ, velInitVar, 0.0);
    put(Parameter::cdt1, clockInitVar, clockProcVar);
    put(Parameter::cdt2, isbOn ? isbInitVar : clockInitVar, isbOn ? isbProcVar : clockProcVar);
    put(Parameter::cdt3, isbOn ? isbInitVar : clockInitVar, isbOn ? isbProcVar : clockProcVar);
    put(Parameter::cdt4, isbOn ? isbInitVar : clockInitVar, isbOn ? isbProcVar : clockProcVar);
    put(Parameter::ifb, gloIfcbInitSigma * gloIfcbInitSigma, gloIfcbProcSigma * gloIfcbProcSigma);
    put(Parameter::iono, 1.0E4, 1.0);
    put(Parameter::ambiguity, ambInitSigma * ambInitSigma, ambProcSigma * ambProcSigma);
    put(Parameter::ztd, ztdInitSigma * ztdInitSigma, ztdProcSigma * ztdProcSigma);
    put(Parameter::dOX, 10.0, 1.0E-10);
    put(Parameter::dOY, 10.0, 1.0E-10);
    put(Parameter::dOZ, 10.0, 1.0E-10);
    put(Parameter::codeBias, codeBiasInitSigma * codeBiasInitSigma, codeBiasProcSigma * codeBiasProcSigma);
    return v;
}

void GnssEKF::measUpdate(EquSys &equSys, const Eigen::MatrixXd &W) {
    const auto numObs = static_cast<int>(equSys.obsEquData.size());
    const auto numUnk = static_cast<int>(currentUnkSet.size());

    Eigen::VectorXd prefit = Eigen::VectorXd::Zero(numObs);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(numObs, numUnk);

    int iobs = 0;
    for (const auto &[eid, ed]: equSys.obsEquData) {
        prefit(iobs) = ed.prefit;
        for (const auto &[var, coeff]: ed.varCoefficient) {
            const int idx = currentIndexData.at(var);
            H(iobs, idx) = coeff;
        }
        iobs++;
    }

    if (const int rc = kalmanFilter.correct(prefit, H, W); rc != 0) {
        std::cerr << "correct skipped: singular geometry (rc=" << rc << ")\n";
        return;
    }
    solution = kalmanFilter.Xhat;
    Q = kalmanFilter.Dx;
    Xhat = kalmanFilter.Xhat;
    Dx = kalmanFilter.Dx;
    postfitResidual = kalmanFilter.postfitResidual;


    // 位置后验 σ 下限(各向异性)
    if (const bool useVecFloor = posFloorSigmaVec.x() > 0.0 || posFloorSigmaVec.y() > 0.0 || posFloorSigmaVec.z() > 0.0;
        posFloorSigma > 0.0 || useVecFloor) {
        for (int k = 0; k < 3; ++k) {
            const int idx = posIdx[k];
            if (idx < 0) continue;
            const double fl = useVecFloor ? posFloorSigmaVec[k] : posFloorSigma;
            if (const double flVar = fl * fl; fl > 0.0 && Dx(idx, idx) < flVar) {
                const double add = flVar - Dx(idx, idx);
                Dx(idx, idx) += add;
                kalmanFilter.Dx(idx, idx) += add;
                Q(idx, idx) += add;
            }
        }
    }

    oldUnkSet = currentUnkSet;
    oldIndexData = currentIndexData;
}

void GnssEKF::updateVelocity(const Eigen::Vector3d &vel, const Eigen::Vector3d &sigma) {
    if (velIdx[0] < 0 || velIdx[1] < 0 || velIdx[2] < 0) return;

    const int numUnk = static_cast<int>(kalmanFilter.Xhat.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, numUnk);
    Eigen::VectorXd z(3);
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    double chi2 = 0.0;
    for (int k = 0; k < 3; ++k) {
        const double s = (std::max)(sigma[k], kMinVeloSigma);
        const double sSq = s * s;
        H(k, velIdx[k]) = 1.0;
        z(k) = vel[k] - kalmanFilter.Xhat(velIdx[k]);
        chi2 += z(k) * z(k) / (kalmanFilter.Dx(velIdx[k], velIdx[k]) + sSq);
        W(k, k) = 1.0 / sSq;
    }
    if (chi2 > kVeloGate * kVeloGate) W /= chi2 / (kVeloGate * kVeloGate);
    const Eigen::VectorXd posPostfit = postfitResidual;
    kalmanFilter.Xminus = kalmanFilter.Xhat;
    kalmanFilter.Dp = kalmanFilter.Dx;
    if (kalmanFilter.correct(z, H, W) != 0) return;
    postfitResidual = posPostfit;
    solution = kalmanFilter.Xhat;
    Q = kalmanFilter.Dx;
    Xhat = kalmanFilter.Xhat;
    Dx = kalmanFilter.Dx;
}

void GnssEKF::savePrior() {
    savedXminus = kalmanFilter.Xminus;
    savedDp = kalmanFilter.Dp;
    hasSaved = true;
}

void GnssEKF::reparameterize(const VariableSet &newOrder, const Eigen::MatrixXd &T) {
    if (const int n = static_cast<int>(solution.size()); n <= 0 || T.cols() != n || T.rows() != static_cast<int>(newOrder.size())) return;

    const Eigen::VectorXd xNew = T * solution;
    const Eigen::MatrixXd pNew = T * Q * T.transpose();

    currentUnkSet = newOrder;
    createIndex(currentUnkSet);
    solution = xNew;
    Q = pNew;
    Xhat = xNew;
    Dx = pNew;
    kalmanFilter.Xhat = xNew;
    kalmanFilter.Dx = pNew;
    kalmanFilter.Xminus = xNew;
    kalmanFilter.Dp = pNew;
    oldUnkSet = currentUnkSet;
    oldIndexData = currentIndexData;
}

double GnssEKF::getSolution(const Parameter &type,
                            const VariableSet &currentUnkSet,
                            const Eigen::VectorXd &stateVec) {
    double out = 0.0;
    if (!tryGetSolution(type, currentUnkSet, stateVec, out))
        throw InvalidRequest("GnssEKF::getSolution: type not found");
    return out;
}

bool GnssEKF::tryGetSolution(const Parameter &type, const VariableSet &currentUnkSet,
                             const Eigen::VectorXd &stateVec, double &out) {
    auto it = currentUnkSet.begin();
    int idx = 0;
    while (it != currentUnkSet.end()) {
        if (it->par == type) {
            out = stateVec(idx);
            return true;
        }
        idx++;
        ++it;
    }
    return false;
}

// SPP 式读取：按 Variable 取回状态分量（set 序索引由 currentIndexData 给出）。
double GnssEKF::getSolution(const Variable &var) const {
    const int idx = indexOf(var);
    if (idx < 0 || idx >= solution.size()) return 0.0;
    return solution(idx);
}

// 获取先验状态分量
double GnssEKF::getMinus(const Variable &var) const {
    const int idx = indexOf(var);
    if (idx < 0 || idx >= kalmanFilter.Xminus.size()) return 0.0;
    return kalmanFilter.Xminus(idx);
}

bool GnssEKF::posteriorVelocity(const std::string &station, Eigen::Vector3d &vel,
                                Eigen::Vector3d &sigma) const {
    const Variable vx(station, Parameter::dVX), vy(station, Parameter::dVY), vz(station, Parameter::dVZ);
    const Variable *vars[3] = {&vx, &vy, &vz};
    for (int k = 0; k < 3; ++k) {
        const int idx = indexOf(*vars[k]);
        if (idx < 0 || idx >= solution.size()) return false;
        vel[k] = solution(idx);
        sigma[k] = std::sqrt((std::max)(Q(idx, idx), 0.0));
    }
    return true;
}

// 获取先验残差
Eigen::VectorXd GnssEKF::getMinusResiduals(const EquSys &equSys) const {
    const int n = static_cast<int>(equSys.obsEquData.size());
    Eigen::VectorXd residuals = Eigen::VectorXd::Zero(n);
    const int dim = static_cast<int>(kalmanFilter.Dp.rows());
    int i = 0;
    for (const auto &[eid, ed]: equSys.obsEquData) {
        double v = 0.0;
        for (const auto &[va, ca]: ed.varCoefficient) {
            const int ia = indexOf(va);
            if (ia < 0 || ia >= dim) continue;
            for (const auto &[vb, cb]: ed.varCoefficient) {
                const int ib = indexOf(vb);
                if (ib < 0 || ib >= dim) continue;
                v += ca * cb * kalmanFilter.Dp(ia, ib);
            }
        }
        residuals(i++) = std::sqrt((std::max)(v, 0.0));
    }
    return residuals;
}
