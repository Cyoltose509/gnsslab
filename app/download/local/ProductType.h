#pragma once

#include <iterator>
#include <string>

#include "i18n/I18n.h"

/// 产品身份的唯一词表：下载库分类、解算输入、界面路径槽都按它索引，不再另立枚举。
/// 前 9 项按日期存（downloads/<typeDir>/<年>/<日序>/），ATX/GPT3 是按版本存（auxdata/<typeDir>/）。
enum class ProductType {
    NAV, ORB, CLK, ERP, ION, TRO, OSB, VMF, ATT,
    OBS, LEO_OBS, LEO_SP3, SNX,
    ATX, GPT3,
    Unknown
};

inline constexpr int kStoredTypeCount = static_cast<int>(ProductType::Unknown);
inline constexpr int kDateTypeCount = static_cast<int>(ProductType::OBS);
inline constexpr int kVersionedFrom = static_cast<int>(ProductType::ATX);

inline constexpr auto kTypeUnknownKey = "common.unknown";

inline constexpr const char *kTypeKeys[] = {
    "BroadcastEphemeris", "PreciseOrbit", "PreciseClock", "EarthRotation", "IonosphereModel",
    "TroposphereModel", "CodeBias", "Vmf3Model", "SatelliteAttitude", "ObservationFile",
    "LeoObservation", "LeoOrbit", "SiteCoordinate", "AntennaFile", "Gpt3Model",
};
static_assert(std::size(kTypeKeys) == kStoredTypeCount, "type key table out of sync with ProductType");

inline int dateTypeIndex(const ProductType t) {
    const int i = static_cast<int>(t);
    return i < kDateTypeCount ? i : -1;
}

/// 按版本存的产品：没有日期目录，取当前安装的那一份
inline constexpr bool isVersioned(const ProductType t) {
    const int i = static_cast<int>(t);
    return i >= kVersionedFrom && i < kStoredTypeCount;
}

inline const char *typeKey(const ProductType t) {
    const int i = static_cast<int>(t);
    return i < kStoredTypeCount ? kTypeKeys[i] : kTypeUnknownKey;
}

inline const char *typeLabel(const ProductType t) { return I18n::trc(typeKey(t)); }

/// 下载库里的目录名：产品在这三个地方（ini 键、下载目录、解算输入）共用一个词根
const char *typeDir(ProductType t);

ProductType typeFromDir(const std::string &dir);

ProductType typeFromIniKey(const std::string &iniKey);
