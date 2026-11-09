#pragma once
#include <atomic>

// X1 从站顺序与 coffee_axis_model.cpp 一致：腰 1，左臂 2..8，右臂 9..15。
namespace diagnostic {
// SOEM 的组 0 表示全部从站，必须使用非零组实现单轴隔离。
inline constexpr unsigned char kGroup = 1;
inline std::atomic<int> selectedAxis{0};
inline std::atomic<bool> selectionLocked{false};
inline std::atomic<bool> busReady{false};
inline std::atomic<bool> stopRequested{false};
inline bool chooseAxis(int axis) {
    if (selectionLocked.load() || axis < 0 || axis > 15) return false;
    selectedAxis.store(axis);
    return true;
}
inline int axis() { return selectedAxis.load(); }
inline bool validAxis(int count) {
    return axis() >= 1 && axis() <= 15 && axis() <= count;
}
inline bool lockSelection() {
    if (!validAxis(15)) return false;
    selectionLocked.store(true);
    return true;
}
inline bool canWrite(int slave, int count) {
    return count == 15 && selectionLocked.load() && validAxis(count) && slave == axis();
}
}
