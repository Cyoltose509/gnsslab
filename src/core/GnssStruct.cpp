
#include "GnssStruct.h"

bool Variable::operator<(const Variable &right) const {
    if (station == right.station) {
        if (paraName == right.paraName) {
            if (obsID == right.obsID) {
                return sat < right.sat;
            }
            return obsID < right.obsID;
        }
        return paraName < right.paraName;
    }
    return station < right.station;
}

bool Variable::operator==(const Variable &right) const {
    if (station == right.station &&
        sat == right.sat &&
        obsID == right.obsID &&
        paraName == right.paraName) {
        return true;
    }
    return false;
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



