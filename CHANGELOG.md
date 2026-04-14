# CHANGELOG

## Versions

### 1.0.2
- Fix: shutdown as damping mode

### 1.0.3 [IMPORTANT FIX]
- Fix control delay, send command immediately after step.
    - this bug could lead to jittering and stability issue, see https://github.com/GDDG08/RoboJuDo/issues/2

### 1.0.4
- `UnitreeConfig::recurrent_lowcmd_writer` (default **true**): optional; set **false** to publish `LowCmd` only from `step()` (test on your hardware first).
- `UnitreeConfig::max_q_delta_rad` configurable; `ResetPositionRateLimiter()` re-seeds the rate limiter from **current** motor `q` (never clears — clearing caused one unbounded step after policy switch).
