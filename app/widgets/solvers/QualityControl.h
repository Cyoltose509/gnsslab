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

        int sysSel = 0;   // 系统筛选：0=全部 1=GPS 2=GLO 3=GAL 4=BDS
        int comboSel = 0; // 逐卫星下拉选择：0=全部
        std::map<std::string, std::pair<std::string, std::function<void()> > > plotDefs;

        // 逐图缓冲（每帧重填；按 plot id 索引，列数按最多的 noise_combined 取 5）
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
