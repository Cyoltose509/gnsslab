#pragma once

namespace UI::DatePicker {
    void today(int &year, int &month, int &day);

    bool button(int &year, int &month, int &day, const int *todayYmd = nullptr);
}
