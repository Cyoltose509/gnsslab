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
        ObsSource::ObsPair files;
        if (!files.open(basePath, roverPath, navPath)) {
            err = files.err;
            return -1;
        }
        if (files.needsNav()) {
            if (navPath.empty()) {
                err = "RINEX 观测需要广播星历文件（任一侧为 RINEX 即需要）";
                return -1;
            }
            if (!ObsSource::loadNav(navPath, navTable, err)) return -1;
            LOG_INFO << "RTK 读取广播星历: " << navPath
                    << " gps=" << navTable.gps.size()
                    << " bds=" << navTable.bds.size();
        }

        Observer::DualPair p(ObsSource::nextOf(files.base), ObsSource::nextOf(files.rover));
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
