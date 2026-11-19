#pragma once


#include <array>
#include <vector>

#include "core/equ/EquStruct.h"
#include "SolverKalman.h"

/// 模糊度重置指令
struct AmbReset {
    enum class Kind {
        NewArc, // 新弧段，放大方差
        Slip // 周跳，给出初值
    };
    Variable var;
    Kind kind = Kind::NewArc;
    double initValue = 0.0;
};

class GnssEKF {

public:
    /// 冷启动/重锚定的位置初值方差(m^2)：σ=10 m，松到让首历元量测主导
    static constexpr double kInitPosVar = 10.0 * 10.0;
    static constexpr double kMinVeloSigma = 0.01;  // m/s：多普勒测速 σ 下限，防止残余估出 1e-4 量级后变成无穷权
    static constexpr double kVeloGate = 4.0;       // 速度新息 χ² 门限（σ 倍数）
    static constexpr double kVarianceUnset = -1.0; // 方差表未配置哨兵：新增参数类型漏填表时显式失败

    GnssEKF();

    void configure(double posSigma, bool hasTropo, double ambSigma = 1.0E-8, double clockSigma = 0.0);

    void reset();

    void createIndex(const VariableSet &varSet);

    [[nodiscard]] int indexOf(const Variable &v) const;

    /// 设置位置状态：需 timeUpdate 建索引后调用，否则无效
    void setPosition(const Eigen::Vector3d &p, double posVar = 0.0);

    /// 设置钟差/ZTD 状态
    void setStates(const std::map<Variable, double> &values,
                   const std::map<Variable, double> *vars = nullptr);

    /// 设置模糊度状态；空指针写全部，否则只写指令覆盖的变量
    void setAmbiguity(const std::string &station, const std::map<SatID, double> &amb,
                      const std::vector<AmbReset> *resets = nullptr);

    void shiftState(const Variable &var, double delta);

    void timeUpdate(const VariableSet &varSet, const std::vector<AmbReset> &resets);

    void measUpdate(EquSys &equSys, const Eigen::MatrixXd &W);

    void updateVelocity(const Eigen::Vector3d &vel, const Eigen::Vector3d &sigma);

    void savePrior();

    void reparameterize(const VariableSet &newOrder, const Eigen::MatrixXd &T);

    static double getSolution(const Parameter &type,
                              const VariableSet &currentUnkSet,
                              const Eigen::VectorXd &stateVec);

    static bool tryGetSolution(const Parameter &type, const VariableSet &currentUnkSet,
                               const Eigen::VectorXd &stateVec, double &out);

    [[nodiscard]] double getSolution(const Variable &var) const;

    [[nodiscard]] double getMinus(const Variable &var) const;

    [[nodiscard]] bool posteriorVelocity(const std::string &station, Eigen::Vector3d &vel,
                                         Eigen::Vector3d &sigma) const;

    [[nodiscard]] Eigen::VectorXd getMinusResiduals(const EquSys &equSys) const;

    virtual ~GnssEKF() = default;


    VariableSet currentUnkSet;
    VariableIntMap currentIndexData;

    double ambProcSigma = 1.0E-4;
    double ambInitSigma = 10.0;
    double clockProcSigma = 0.0;
    double ztdProcSigma = 1.0E-3;
    double posFloorSigma = 0.0;
    Eigen::Vector3d posFloorSigmaVec = Eigen::Vector3d(0.0, 0.0, 0.0);
    double ztdInitSigma = 0.2;
    double ztdInitValue = 0.0;
    double clockInitSigma = 100.0;
    double accProcSigma = 0.0; // 加速度过程 σ (m/s²)
    double velInitSigma = 10.0; // 速度状态初始 σ (m/s)
    double motionDt = 0.0; // 与上一历元的间隔 (s)，0 = 本历元不预测（首历元/断档）
    bool mCVModel = false; // 本历元位置是否被速度预测耦合（＝常速度模型生效），速度量测经此影响位置
    Eigen::VectorXd solution; // 后验状态向量 x
    Eigen::MatrixXd Q; // 后验协方差
    Eigen::VectorXd postfitResidual;

    double codeBiasProcSigma = 1.0E-4;
    double codeBiasInitSigma = 1.0;
    double isbProcSigma = 0.0;
    double isbInitSigma = 0.0;

    double gloIfcbProcSigma = 1.0E-4;
    double gloIfcbInitSigma = 1.0;

    std::array<int, 3> posIdx = {-1, -1, -1};
    std::array<int, 3> velIdx = {-1, -1, -1};
private:
    struct VarianceTable {
        std::array<double, Parameter::count> init;
        std::array<double, Parameter::count> proc;
    };


    [[nodiscard]] VarianceTable varianceTables() const;



    bool firstTime;
    bool useTropo = true;
    double posProcSigma = 0.0;

    Eigen::VectorXd Xhat;
    Eigen::MatrixXd Dx;
    VariableSet oldUnkSet;
    VariableIntMap oldIndexData;

    SolverKalman kalmanFilter;

    Eigen::VectorXd savedXminus;
    Eigen::MatrixXd savedDp;
    bool hasSaved = false;
};
