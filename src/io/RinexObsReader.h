
#pragma once
#include <fstream>
#include "GnssStruct.h"
#include "RinexFileBase.h"

class RinexObsReader : public RinexFileBase {
public:
    RinexObsReader()
    : isHeaderRead(false)
    {}

    std::fstream* pFileStream = nullptr;

    void parseRinexHeader();
    ObsData parseRinexObs();

    void fastForwardToEpoch(const CommonTime &target) const;


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

    std::map<std::string, int> mGloChan;
    bool isHeaderRead;


    void syncStream() { pStream = pFileStream; }
};



