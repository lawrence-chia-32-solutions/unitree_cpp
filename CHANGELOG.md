# CHANGELOG

## Versions

### 1.0.2
- Fix: shutdown as damping mode

### 1.0.3 [IMPORTANT FIX]
- Fix control delay, send command immediately after step.
    - this bug could lead to jittering and stability issue, see https://github.com/GDDG08/RoboJuDo/issues/2

### 1.0.4
- `UnitreeConfig::recurrent_lowcmd_writer` (default **false**): when **false**, only `step()` publishes `LowCmd` (Python tick); avoids duplicate ~50 Hz DDS sends from the recurrent thread + `step()` (can reduce mechanical jitter).
- `UnitreeConfig::max_q_delta_rad` configurable; `ResetPositionRateLimiter()` clears position rate-limit state.
