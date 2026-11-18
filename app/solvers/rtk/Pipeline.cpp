#include "Pipeline.h"

#include "Log.h"
#include "io/ObsSource.h"
#include "solver/Products.h"
#include "i18n/I18n.h"

namespace Solvers::RTK {
    int pairFiles(const std::string &basePath, const std::string &roverPath,
                  const std::string &navPath, const double sampleInterval,
                  Products &prod,
                  const std::function<bool()> &shouldStop,
                  const std::function<void(const ObsData &, const ObsData &,
                                           const std::shared_ptr<EphemerisTable> &)> &onPair,
                  std::string &err) {
        ObsSource::ObsPair files;
        if (!files.open(basePath, roverPath, navPath)) {
            err = files.err;
            return -1;
        }
        // 流内不带星历（RINEX 回放）时必须有广播星历：已由任务按产品表读进 prod.eph
        if (files.needsNav() && prod.eph.empty()) {
            err = I18n::tr("rtk.err.need_nav");
            return -1;
        }

        DualPair p(ObsSource::nextOf(files.base), ObsSource::nextOf(files.rover));
        p.setInterval(sampleInterval);
        int n = 0;
        DualPair::Pair pr;
        while (!shouldStop() && p.next(pr)) {
            onPair(pr.base, pr.rover, DualPair::selectEph(pr.baseEph, pr.roverEph));
            ++n;
        }
        return n;
    }
}
