#include "GuiHelpers.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <string>
#include <vector>

static int inputTextResizeCallback(ImGuiInputTextCallbackData *data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto *str = static_cast<std::string *>(data->UserData);
        str->resize(data->BufSize - 1);
        data->Buf = const_cast<char *>(str->c_str());
    }
    return 0;
}

namespace Solvers::Helpers {
    RobustResRange computeRobustResRange(const std::map<SatID, std::vector<double> > &satRes,
                                         const double minHalf) {
        std::vector<double> all_v;
        for (const auto &[_, vs]: satRes)
            for (double v: vs) all_v.push_back(v);
        RobustResRange r;
        if (all_v.size() < 2) return r;
        std::sort(all_v.begin(), all_v.end());
        const auto n = static_cast<long>(all_v.size());
        const double lo = all_v[static_cast<std::size_t>(0.01 * n)];
        const double hi = all_v[static_cast<std::size_t>(0.99 * n)];
        double half = 0.5 * (hi - lo);
        if (!(half > 0.0)) half = minHalf;
        half = std::max(minHalf, std::min(half, 20.0));
        r.lo = -half;
        r.hi = half;
        return r;
    }

    bool inputTextStd(const char *label, std::string *str, const ImGuiInputTextFlags flags) {
        if (str->capacity() < 512) str->reserve(512);
        const bool changed = ImGui::InputText(
            label, str->data(), str->capacity() + 1,
            flags | ImGuiInputTextFlags_CallbackResize, inputTextResizeCallback, str);
        if (const std::size_t len = std::strlen(str->data()); len != str->size()) str->resize(len);
        return changed;
    }
}
