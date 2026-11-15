#include "core/equ/EquStruct.h"

bool Variable::operator<(const Variable &right) const {
    if (station == right.station) {
        if (par == right.par) {
            if (obsID == right.obsID) {
                return sat < right.sat;
            }
            return obsID < right.obsID;
        }
        return par < right.par;
    }
    return station < right.station;
}

bool Variable::operator==(const Variable &right) const {
    return station == right.station &&
           sat == right.sat &&
           obsID == right.obsID &&
           par == right.par;
}

bool Variable::operator!=(const Variable &right) const {
    return !(*this == right);
}

Eigen::VectorXd EquSys::weightVector() const {
    Eigen::VectorXd w(static_cast<Eigen::Index>(obsEquData.size()));
    Eigen::Index i = 0;
    for (const auto &[eid, ed]: obsEquData) {
        w[i++] = ed.weight;
    }
    return w;
}
