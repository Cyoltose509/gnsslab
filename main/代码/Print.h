#pragma once
#include<iostream>
#include"Decode.h"
//打印函数
void printGpsEphemeris(const GpsEphemeris& eph) {
    cout << "GPS PRN=" << eph.prn
        << " week=" << eph.week
        << " toe=" << fixed << setprecision(0) << eph.toe
        << " toc=" << eph.toc
        << " A=" << scientific << setprecision(6) << eph.A
        << " ecc=" << fixed << setprecision(6) << eph.ecc
        << " i0=" << fixed << setprecision(6) << eph.i0
        << " omega0=" << fixed << setprecision(6) << eph.omega0
        << " omega=" << fixed << setprecision(6) << eph.omega
        << " M0=" << fixed << setprecision(6) << eph.M0
        << " dN=" << scientific << setprecision(6) << eph.dN
        << " cuc=" << scientific << setprecision(6) << eph.cuc
        << " cus=" << scientific << setprecision(6) << eph.cus
        << " crc=" << fixed << setprecision(3) << eph.crc
        << " crs=" << fixed << setprecision(3) << eph.crs
        << " cic=" << scientific << setprecision(6) << eph.cic
        << " cis=" << scientific << setprecision(6) << eph.cis
        << " idot=" << scientific << setprecision(6) << eph.idot
        << " omegaDot=" << scientific << setprecision(6) << eph.omegaDot
        << " af0=" << scientific << setprecision(6) << eph.af0
        << " af1=" << scientific << setprecision(6) << eph.af1
        << " af2=" << scientific << setprecision(6) << eph.af2
        << " tgd=" << scientific << setprecision(6) << eph.tgd
        << " health=" << eph.health << endl;
}
