#pragma once

/// 可调高度的区域：按 key 记住用户拉出来的高度，并在其下方画一条分隔把手。
namespace UI {
    /// key 对应的高度，首次用 defH；返回引用，拖动时直接改写它。
    float &splitHeight(const char *key, float defH);

    /// 水平分隔把手（7 px 高、带短横线）：按住上下拖动改 height，夹在 [minH, maxH]。
    void splitHandle(const char *id, float &height, float minH, float maxH);
}
