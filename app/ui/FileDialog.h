#pragma once

#include <string>

#include "Gui.h"
#include "StringUtils.h"

namespace FileDialog {
    inline GuiFileFilter fAll{L"所有文件", {L"*"}};

    inline GuiFileFilter fNovAtel{L"NovAtel 日志", {L"dat", L"log", L"asc", L"gps", L"gnss"}};

    inline GuiFileFilter fClk[] = {{L"精密钟差 CLK", {L"CLK"}}, fAll};
    inline GuiFileFilter fOsb[] = {{L"码偏差 OSB/BIA", {L"BIA", L"OSB"}}, fAll};
    inline GuiFileFilter fAtx[] = {{L"天线文件 ATX", {L"atx"}}, fAll};
    inline GuiFileFilter fSp3[] = {{L"精密轨道 SP3", {L"SP3"}}, fAll};
    inline GuiFileFilter fRnx[] = {{L"广播星历", {L"??N", L"??G", L"??C", L"nav", L"rnx"}}, fAll};
    inline GuiFileFilter fObs[] = {{L"RINEX 观测", {L"??O", L"rnx", L"obs"}}, fNovAtel, fAll};
    inline GuiFileFilter fHdr[] = {{L"杠杆臂", {L"HDR"}}, fAll};
    inline GuiFileFilter fCsv[] = {{L"CSV", {L"csv"}}, fAll};
    inline GuiFileFilter fPng[] = {{L"PNG 文件", {L"png"}}, fAll};
    inline GuiFileFilter fErp[] = {{L"地球自转参数 ERP", {L"ERP"}}, fAll};
    inline GuiFileFilter fTro[] = {{L"对流层 TRO", {L"TRO"}}, fAll};
    inline GuiFileFilter fIonex[] = {{L"电离层 IONEX", {L"ION", L"IONEX"}}, fAll};
    inline GuiFileFilter fVmf[] = {{L"映射函数格网 VMF", {L"H00", L"H06", L"H12", L"H18", L"grd"}}, fAll};
    inline GuiFileFilter fAtt[] = {{L"卫星姿态 ORBEX", {L"OBX"}}, fAll};
    inline GuiFileFilter fBia[] = {{L"偏差 SINEX", {L"BIA", L"OSB"}}, fAll};
    inline GuiFileFilter fProd[] = {
        {
            L"GNSS 产品文件",
            {
                L"SP3", L"CLK", L"ERP", L"BIA", L"OSB", L"TRO", L"INX",
                L"ION", L"OBX", L"rnx", L"H00", L"H06", L"H12", L"H18", L"grd"
            }
        },
        fAll
    };
    inline GuiFileFilter fIns[] = {fNovAtel, fAll};

    inline bool save(const std::string &defaultName, std::wstring &outPath,
                     const GuiFileFilter *filters, const int nFilters, const wchar_t *defExt) {
        const std::wstring wDef = Str::ACPToWide(defaultName);
        return ShowSaveFileDialog(outPath, filters, nFilters, wDef.c_str(), defExt);
    }

    inline bool saveCSV(const std::string &defaultName, std::wstring &outPath) {
        return save(defaultName, outPath, fCsv, 2, L"csv");
    }
}
