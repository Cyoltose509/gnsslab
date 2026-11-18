#pragma once

#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "core/geom/CoordStruct.h"
#include "core/obs/ObsStruct.h"
#include "rinex/RINEXFileBase.h"

struct RINEXHeader {
    std::string station;
    double version;
    std::string antType;
    XYZ antennaPosition;
    Eigen::Vector3d antDeltaENU{0, 0, 0};
    std::map<SatSystem, std::vector<std::string> > mapObsTypes;

    std::map<std::string, double> phaseShift;

    RINEXHeader() : version(0) {
    }
};

class RINEXObsReader : public RINEXFileBase {
public:
    RINEXObsReader() = default;

    ~RINEXObsReader() override = default;

    void parseRINEXHeader();

    ObsData parseRINEXObs();

    void fastForwardToEpoch(const CommonTime &target) const;

    [[nodiscard]] const RINEXHeader &getHeader() const { return rinexHeader; }

    [[nodiscard]] double version() const { return rinexHeader.version; }

    /// 头里 GLONASS SLOT / FRQ # 不全时用它补槽位（PRN→频率号），由调用方从广播星历取；
    /// 头里已有的以头为准。必须在 parseRINEXHeader 之前赋值。
    std::map<std::string, int> gloSlots;

    ObsData parseRINEXObs(const CommonTime &syncEpoch) {
        const std::streampos sp(pStream->tellg());
        ObsData obsData;
        while (pStream->peek() != EOF) {
            obsData = parseRINEXObs();
            if (obsData.epoch >= syncEpoch) break;
        }
        if (obsData.epoch > syncEpoch + 0.001) {
            pStream->seekg(sp);
            throw SyncException("can't synchronize the obs!");
        }
        return obsData;
    }

    void chooseObs(ObsData &obsData);

    RINEXHeader rinexHeader;
    std::map<SatSystem, std::set<std::string> > sysTypes;
    std::map<std::string, int> mGloChan;
    bool isHeaderRead = false;

    virtual void readHeaderLabel(const std::string &label, const std::string &line) = 0;

    [[nodiscard]] virtual bool isEpochLine(const std::string &line) const = 0;

    [[nodiscard]] virtual CommonTime epochTime(const std::string &line) const = 0;

    virtual void readEpochHead(std::string &line, int &flag, int &numSats) = 0;

    virtual void afterHeader() {
    }

    virtual void readSatObs(SatTypeValueMap &stvData, int numSats, SatTypeLLIMap &lliOut);

    void decodeObsBlock(const SatID &sat, const std::string &buf, SatTypeValueMap &out,
                        SatTypeLLIMap &lliOut) const;
};

class RINEX2ObsReader : public RINEXObsReader {
public:
    static CommonTime parseTime(const std::string &line);

    void readHeaderLabel(const std::string &label, const std::string &line) override;

    [[nodiscard]] bool isEpochLine(const std::string &line) const override;

    [[nodiscard]] CommonTime epochTime(const std::string &line) const override;

    void readEpochHead(std::string &line, int &flag, int &numSats) override;

    void readSatObs(SatTypeValueMap &stvData, int numSats, SatTypeLLIMap &lliOut) override;

    void afterHeader() override;

private:
    std::vector<std::string> mTypes2;
    std::vector<std::string> mSatIds;
    int mObsCount = 0;
};

class RINEX3ObsReader : public RINEXObsReader {
public:
    static CommonTime parseTime(const std::string &line);

    void readHeaderLabel(const std::string &label, const std::string &line) override;

    [[nodiscard]] bool isEpochLine(const std::string &line) const override;

    [[nodiscard]] CommonTime epochTime(const std::string &line) const override;

    void readEpochHead(std::string &line, int &flag, int &numSats) override;

private:
    std::map<SatSystem, std::vector<std::string> > mObsTypes;
};
