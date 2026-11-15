#include "download/local/ProductType.h"

#include "StringUtils.h"

const char *typeDir(const ProductType t) {
    switch (t) {
        case ProductType::NAV: return "nav";
        case ProductType::ORB: return "sp3";
        case ProductType::CLK: return "clk";
        case ProductType::ERP: return "erp";
        case ProductType::ION: return "ionex";
        case ProductType::TRO: return "tro";
        case ProductType::OSB: return "osb";
        case ProductType::VMF: return "vmf";
        case ProductType::ATT: return "att";
        case ProductType::OBS: return "rinex";
        case ProductType::LEO_OBS: return "leo_obs";
        case ProductType::LEO_SP3: return "leo_sp3";
        case ProductType::SNX: return "ref_truth/snx";
        case ProductType::ATX: return "atx";
        case ProductType::GPT3: return "gpt3";
        default: return "other";
    }
}

ProductType typeFromDir(const std::string &dir) {
    for (int i = 0; i < kDateTypeCount; ++i) {
        if (const auto t = static_cast<ProductType>(i); dir == typeDir(t)) return t;
    }
    return ProductType::Unknown;
}

ProductType typeFromIniKey(const std::string &iniKey) {
    const std::string s = Str::toLower(iniKey);
    if (s.find("sp3") != std::string::npos) return ProductType::ORB;
    if (s.find("clk") != std::string::npos) return ProductType::CLK;
    if (s.find("erp") != std::string::npos) return ProductType::ERP;
    if (s.find("ionex") != std::string::npos ||
        s.find("ion") != std::string::npos)
        return ProductType::ION;
    if (s.find("tro") != std::string::npos) return ProductType::TRO;
    if (s.find("osb") != std::string::npos ||
        s.find("bia") != std::string::npos)
        return ProductType::OSB;
    if (s.find("brdc") != std::string::npos ||
        s.find("nav") != std::string::npos)
        return ProductType::NAV;
    if (s.find("vmf") != std::string::npos) return ProductType::VMF;
    if (s.find("att") != std::string::npos) return ProductType::ATT;
    if (s.find("atx") != std::string::npos) return ProductType::ATX;
    if (s.find("gpt3") != std::string::npos) return ProductType::GPT3;
    return ProductType::Unknown;
}
