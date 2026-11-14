#pragma once

/// 可拖拽高度的通用控件：按 key 记住用户拉出来的高度，并在其下方画一条拖拽把手。
namespace UI {
    /// key 对应的高度，首次用 defH；返回引用，拖动时直接改写它。
    float &dragHeight(const char *key, float defH);

    /// 水平把手（7 px 高、带短横线）：按住上下拖动改 height，夹在 [minH, maxH]。
    void heightGrip(const char *id, float &height, float minH, float maxH);
}
