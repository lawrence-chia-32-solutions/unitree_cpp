// Hard guard helpers for low-level command path (Phase 1 defaults).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace unitree_bridge {

inline float ClampF(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}

/// Clamp each joint target to a conservative range (radians).
inline void ClampPositionTargets(std::vector<float>& q, float lo = -3.5f, float hi = 3.5f) {
    for (float& x : q) {
        x = ClampF(x, lo, hi);
    }
}

/// Limit per-joint step from previous command (Python-rate stepping).
inline void RateLimitPosition(std::vector<float>& q, const std::vector<float>& prev, float max_delta_rad) {
    if (q.size() != prev.size()) return;
    for (size_t i = 0; i < q.size(); ++i) {
        float d = q[i] - prev[i];
        if (d > max_delta_rad) q[i] = prev[i] + max_delta_rad;
        if (d < -max_delta_rad) q[i] = prev[i] - max_delta_rad;
    }
}

}  // namespace unitree_bridge
