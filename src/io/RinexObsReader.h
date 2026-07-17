
#pragma once
#include <fstream>
#include "GnssStruct.h"

class RinexObsReader {
public:
    RinexObsReader()
    : pFileStream(nullptr), isHeaderRead(false)
    {}

    void setFileStream(std::fstream* pStream)
    {
        pFileStream = pStream;
    }

    void setSelectedTypes(const std::map<char, std::set<string>>& systemTypes)
    {
        sysTypes = systemTypes;
    }

    void parseRinexHeader();
    ObsData parseRinexObs();

    /// 返回已解析的 RINEX header（含 mapObsTypes 可用于自动检测 IF 类型）
    [[nodiscard]] const RinexHeader &getHeader() const { return rinexHeader; }

    ObsData parseRinexObs(const CommonTime& syncEpoch)
    {
        // store current stream pos;
        const streampos sp( pFileStream->tellg() );
        ObsData obsData;
        while(true){
            if( pFileStream->peek() == EOF ){
                break;
            }
            // read a record from current strm;
            obsData = parseRinexObs();

            // 首先寻找大于等于参考时刻的历元
            // 只要大于等于，就意味着时间是同步的或者是超过了给定参考时刻的
            if(obsData.epoch >=syncEpoch)
            {
                break;
            }
        }
        // 如果流动站的时刻大于给定的参考时刻+容许的误差，则说明流动站观测值超前了，
        // 此时，同步失败，且要把流动站的流重置到文件开头，以实现下一个历元的同步。
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
    std::fstream* pFileStream;
    RinexHeader rinexHeader;
    std::map<char, std::set<string>> sysTypes;
    bool isHeaderRead;
};



