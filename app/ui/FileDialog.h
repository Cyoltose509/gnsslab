#pragma once

#include <string>

#include "Gui.h"
#include "StringUtils.h"

// 通用文件选择对话框：各文件类型的过滤器常量 + 保存对话框包装。
namespace FileDialog {
    inline constexpr GuiFileFilter fAll = {L"所有文件 (*.*)", L"*.*"};
    inline constexpr GuiFileFilter fNovAtel = {
        L"NovAtel 日志 (*.dat;*.log;*.asc;*.gps;.*gnss)", L"*.dat;*.log;*.asc;*.gps;.*gnss"
    };

    inline constexpr GuiFileFilter fClk[] = {
        {L"精密钟差 CLK (*.CLK;*.clk)", L"*.CLK;*.clk"},
        fAll
    };
    inline constexpr GuiFileFilter fOsb[] = {
        {L"码偏差 OSB/BIA (*.BIA;*.bia;*.OSB)", L"*.BIA;*.bia;*.OSB"},
        fAll
    };
    inline constexpr GuiFileFilter fAtx[] = {
        {L"天线文件 ATX (*.atx;*.ATX)", L"*.atx;*.ATX"},
        fAll
    };
    inline constexpr GuiFileFilter fSp3[] = {
        {L"精密轨道 SP3 (*.SP3;*.sp3)", L"*.SP3;*.sp3"},
        fAll
    };
    inline constexpr GuiFileFilter fRnx[] = {
        {L"广播星历 (*.??N;*.??G;*.??C;*.nav;*.rnx)", L"*.??N;*.??G;*.??C;*.nav;*.rnx"},
        fAll
    };
    inline constexpr GuiFileFilter fObs[] = {
        {L"RINEX 观测 (*.??O;*.rnx;*.obs)", L"*.??O;*.rnx;*.obs"},
        fNovAtel,
        fAll
    };
    inline constexpr GuiFileFilter fHdr[] = {
        {L"杠杆臂 (*.HDR;*.hdr)", L"*.HDR;*.hdr"},
        fAll
    };
    inline constexpr GuiFileFilter fCsv[] = {
        {L"CSV (*.csv)", L"*.csv"},
        fAll
    };
    inline constexpr GuiFileFilter fPng[] = {
        {L"PNG 文件 (*.png)", L"*.png"},
        fAll
    };
    inline constexpr GuiFileFilter fErp[] = {
        {L"地球自转参数 ERP (*.ERP;*.erp)", L"*.ERP;*.erp"},
        fAll
    };
    inline constexpr GuiFileFilter fTro[] = {
        {L"对流层 TRO (*.TRO;*.tro)", L"*.TRO;*.tro"},
        fAll
    };
    inline constexpr GuiFileFilter fIonex[] = {
        {L"电离层 IONEX (*.ION;*.ion;*.IONEX;*.ionex)", L"*.ION;*.ion;*.IONEX;*.ionex"},
        fAll
    };
    inline constexpr GuiFileFilter fVmf[] = {
        {L"映射函数格网 VMF (*.H00;*.H06;*.H12;*.H18;*.grd)", L"*.H00;*.H06;*.H12;*.H18;*.grd"},
        fAll
    };
    inline constexpr GuiFileFilter fAtt[] = {
        {L"卫星姿态 ORBEX (*.OBX;*.obx)", L"*.OBX;*.obx"},
        fAll
    };
    inline constexpr GuiFileFilter fMet[] = {
        {L"气象文件 MET (*.rnx;*.??m;*.csv)", L"*.rnx;*.??m;*.csv"},
        fAll
    };
    inline constexpr GuiFileFilter fBia[] = {
        {L"偏差 SINEX (*.BIA;*.bia;*.OSB;*.osb)", L"*.BIA;*.bia;*.OSB;*.osb"},
        fAll
    };
    inline constexpr GuiFileFilter fProd[] = {
        {
            L"GNSS 产品文件 (*.SP3;*.CLK;*.ERP;*.BIA;*.OSB;*.TRO;*.INX;*.ION;*.OBX;*.rnx;*.H00;*.H06;*.H12;*.H18;*.grd)",
            L"*.SP3;*.sp3;*.CLK;*.clk;*.ERP;*.erp;*.BIA;*.bia;*.OSB;*.osb;*.TRO;*.tro;*.INX;*.inx;*.ION;*.ion;*.OBX;*.obx;*.rnx;*.H00;*.H06;*.H12;*.H18;*.grd"
        },
        fAll
    };
    inline constexpr GuiFileFilter fIns[] = {
        fNovAtel,
        fAll
    };

    inline bool save(const std::string &defaultName, std::wstring &outPath,
                     const GuiFileFilter *filters, const int nFilters, const wchar_t *defExt) {
        const std::wstring wDef = Str::ACPToWide(defaultName);
        return ShowSaveFileDialog(outPath, filters, nFilters, wDef.c_str(), defExt);
    }

    inline bool saveCSV(const std::string &defaultName, std::wstring &outPath) {
        return save(defaultName, outPath, fCsv, 2, L"csv");
    }
}
