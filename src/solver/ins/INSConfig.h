#pragma once

#include <string>
#include "core/ins/InsStruct.h"

struct INSConfig {
    std::string imuPath;

    double latDeg = 0.0, lonDeg = 0.0, hM = 0.0;
    double vN = 0.0, vE = 0.0, vD = 0.0;

    bool specifyAttitude = false;
    double rollDeg = 0.0, pitchDeg = 0.0, headingDeg = 0.0;

    double zuptAccelTol = 0.3;
    double zuptGyroTol = 0.02;
    double zuptHoldSec = 0.5;

    ImuScale manual;
};
