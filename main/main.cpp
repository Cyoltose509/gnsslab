#include <sstream>
#include<cmath>

#include"OEM7Reader.h"


int main() {
    string filename = "NovatelOEM20211114-01.log";

    OEM7Reader oem7;
    oem7.open(filename);
    for (int i = 1; i < 100; i++)
        oem7.readOne();

    return 0;
}
