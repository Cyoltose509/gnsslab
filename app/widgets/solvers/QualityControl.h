#pragma once

#include <memory>
#include "imgui.h"
#include "QCProcessor.h"


using QC::SatQC;
using QC::QualityReport;


namespace GnssTask {
    struct SolveTask;
    struct EpochData;
}

namespace Solvers::PppProcessor {
    struct PppTask;
}

namespace QualityControl {
    struct QcUiState {
        std::map<std::string, ImVec4> curRect;
        std::map<std::string, float> contentY;
        std::vector<std::string> lastIds;
        std::vector<std::string> exportQueue;
        std::string exportDir;
        bool exporting = false;

        int sysSel = 0;
        int comboSel = 0;
        std::map<std::string, std::pair<std::string, std::function<void()> > > plotDefs;


        struct PlotBuf {
            std::vector<double> col[5];
            std::vector<std::string> labs;
            std::vector<const char *> labp;
        };
        std::map<std::string, PlotBuf> bufs;
        PlotBuf &buf(const std::string &id) { return bufs[id]; }
    };
    template<typename TaskT>
    void render(const std::shared_ptr<TaskT> &task);
}
