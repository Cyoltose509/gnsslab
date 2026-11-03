#include "Pipeline.h"

#include "Log.h"
#include "io/ObsSource.h"

namespace Solvers::RTK {
    int pairFiles(const std::string &basePath, const std::string &roverPath,
                  const std::string &navPath, const double sampleInterval,
                  EphemerisTable &navTable,
                  const std::function<bool()> &shouldStop,
                  const std::function<void(const ObsData &, const ObsData &,
                                           const std::shared_ptr<EphemerisTable> &,
                                           EphemerisTable *)> &onPair,
                  std::string &err) {
        const auto fb = Observer::classifyObsPath(basePath);
        const auto fr = Observer::classifyObsPath(roverPath);
        if (fb == Observer::Format::Unknown || fr == Observer::Format::Unknown) {
            err = "无法识别观测文件格式";
            return -1;
        }
        const bool baseRinex = Observer::isRINEX(fb);
        const bool roverRinex = Observer::isRINEX(fr);
        if (baseRinex || roverRinex) {
            if (navPath.empty()) {
                err = "RINEX 模式需要广播星历（任一观测文件为 RINEX 即需要）";
                return -1;
            }
            if (!ObsSource::loadNav(navPath, navTable, err)) return -1;
            LOG_INFO << "RTK 读取广播星历: " << navPath
                    << " gps=" << navTable.gps.size()
                    << " bds=" << navTable.bds.size();
        }

        ObsSource::Handles h;
        ObsSource::Puller nextBase, nextRover;
        if (!ObsSource::openDual(baseRinex, roverRinex, basePath, roverPath, h, nextBase, nextRover, err))
            return -1;

        Observer::DualPair p(nextBase, nextRover);
        p.setInterval(sampleInterval);
        int n = 0;
        Observer::DualPair::Pair pr;
        while (!shouldStop() && p.next(pr)) {
            onPair(pr.base, pr.rover, pr.baseEph ? pr.baseEph : pr.roverEph, &navTable);
            ++n;
        }
        return n;
    }
}
