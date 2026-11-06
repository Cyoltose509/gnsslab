#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>
#include "imgui.h"
#include "QCProcessor.h"
#include "solvers/common/Core.h"


using QC::SatQC;
using QC::QualityReport;


namespace Solvers {
    struct Core;
    struct Epoch;
    struct View;
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

        /// 逐卫星断点时间轴。QC 序列只在双频齐全的历元上有点，卫星不可见或丢频的
        /// 整段是真空档，直连会画出一条跨越数小时的假斜线。`report` 变了就整批作废。
        struct SatAxes {
            const void *report = nullptr;
            double interval = 0.0;
            std::map<SatID, Solvers::BrokenAxis> bySat;
        };
        SatAxes axes;
    };
    /// 渲染一份质量分析：view 提供输入通道与快照，fileName 只用于导出命名。
    void render(Solvers::View &view, const std::string &fileName);
}
