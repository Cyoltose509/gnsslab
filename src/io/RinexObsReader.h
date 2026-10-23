
#pragma once
#include <fstream>
#include "GnssStruct.h"
#include "RinexFileBase.h"

class RinexObsReader : public RinexFileBase {
public:
    RinexObsReader()
    : isHeaderRead(false)
    {}

    std::fstream* pFileStream = nullptr;   // 观测文件流；由调用方直接赋值（替代 setFileStream）

    void parseRinexHeader();
    ObsData parseRinexObs();

    void fastForwardToEpoch(const CommonTime &target) const;

    /// 返回已解析的 RINEX header（含 mapObsTypes 可用于自动检测 IF 类型）
    [[nodiscard]] const RinexHeader &getHeader() const { return rinexHeader; }

    ObsData parseRinexObs(const CommonTime& syncEpoch)
    {
        const streampos sp( pFileStream->tellg() );
        ObsData obsData;
        while(true){
            if( pFileStream->peek() == EOF ){
                break;
            }
            obsData = parseRinexObs();
            if(obsData.epoch >=syncEpoch)
            {
                break;
            }
        }
        if(obsData.epoch > syncEpoch + 0.001 )
        {
            pFileStream->seekg( sp );
            throw SyncException("Rx3ObsData::can't synchronize the obs!");
        }

        return obsData;
    }

    static CommonTime parseTime(const string &line);
    void chooseObs(ObsData &obsData);


private:
    RinexHeader rinexHeader;
    std::map<char, std::set<string>> sysTypes;
    /// GLONASS 频率通道号 k：键为卫星标签(R01..R24)，来自 RINEX "GLONASS SLOT / FRQ #"
    std::map<std::string, int> mGloChan;
    bool isHeaderRead;

    /// obs 流由调用方经 pFileStream 注入；同步到基类 pStream 供 nextLine/scanLabeledHeader 使用。
    void syncStream() { pStream = pFileStream; }
};



