#pragma once

#include <string>
#include <vector>

#include "Gui.h"
#include "StringUtils.h"
#include "i18n/I18n.h"

namespace FileDialog {
    inline GuiFileFilter make(const std::string_view key, const std::vector<std::wstring> &exts) {
        return {Str::UTF8ToWide(std::string(I18n::trv(key))), exts};
    }

    inline GuiFileFilter all() { return make("dlg.filter.all", {L"*"}); }

    inline GuiFileFilter novAtel() {
        return make("dlg.filter.novatel", {L"dat", L"log", L"asc", L"gps", L"gnss"});
    }

    inline std::vector<GuiFileFilter> fClk() { return {make("PreciseClock", {L"CLK"}), all()}; }

    inline std::vector<GuiFileFilter> fAtx() { return {make("AntennaFile", {L"atx"}), all()}; }

    inline std::vector<GuiFileFilter> fSp3() { return {make("PreciseOrbit", {L"SP3"}), all()}; }

    inline std::vector<GuiFileFilter> fRnx() {
        return {make("BroadcastEphemeris", {L"??N", L"??G", L"??C", L"nav", L"rnx"}), all()};
    }

    inline std::vector<GuiFileFilter> fObs() {
        return {make("dlg.filter.obs", {L"??O", L"rnx", L"obs"}), novAtel(), all()};
    }

    inline std::vector<GuiFileFilter> fCsv() { return {{L"CSV", {L"csv"}}, all()}; }

    inline std::vector<GuiFileFilter> fPng() { return {make("dlg.filter.png", {L"png"}), all()}; }

    inline std::vector<GuiFileFilter> fErp() { return {make("EarthRotation", {L"ERP"}), all()}; }

    inline std::vector<GuiFileFilter> fTro() { return {make("TroposphereModel", {L"TRO"}), all()}; }

    inline std::vector<GuiFileFilter> fIonex() { return {make("IonosphereModel", {L"ION", L"IONEX"}), all()}; }

    inline std::vector<GuiFileFilter> fVmf() {
        return {make("Vmf3Model", {L"H00", L"H06", L"H12", L"H18", L"grd"}), all()};
    }

    inline std::vector<GuiFileFilter> fAtt() { return {make("SatelliteAttitude", {L"OBX"}), all()}; }

    inline std::vector<GuiFileFilter> fProd() {
        return {
            make("dlg.filter.product", {
                     L"SP3", L"CLK", L"ERP", L"BIA", L"OSB", L"TRO", L"INX",
                     L"ION", L"OBX", L"rnx", L"H00", L"H06", L"H12", L"H18", L"grd"
                 }),
            all()
        };
    }

    inline std::vector<GuiFileFilter> fIns() { return {novAtel(), all()}; }

    inline std::vector<GuiFileFilter> fBin() { return {make("dlg.filter.bin", {L"bin"}), all()}; }

    inline bool save(const std::string &defaultName, std::wstring &outPath,
                     const std::vector<GuiFileFilter> &filters, const wchar_t *defExt) {
        const std::wstring wDef = Str::ACPToWide(defaultName);
        return ShowSaveFileDialog(outPath, filters.data(), static_cast<int>(filters.size()),
                                  wDef.c_str(), defExt);
    }

    inline bool open(std::wstring &outPath, const std::vector<GuiFileFilter> &filters) {
        return ShowOpenFileDialog(outPath, filters.data(), static_cast<int>(filters.size()));
    }

    inline bool openMany(std::vector<std::wstring> &outPaths, const std::vector<GuiFileFilter> &filters) {
        return ShowOpenFilesDialog(outPaths, filters.data(), static_cast<int>(filters.size()));
    }

    inline bool saveCSV(const std::string &defaultName, std::wstring &outPath) {
        return save(defaultName, outPath, fCsv(), L"csv");
    }
}
