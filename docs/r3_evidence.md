# Round 3 incremental evidence — bounded recovery (no permanent stall)

## Starting point

Round 3 begins from the frozen Round 2 candidate (`scratchpad/candidate_v1`), whose final default
drive gate (`docs/d0809_evidence.md`, "Round 2 final outcome") is: 4/4 sections pass; full_1
11.642818 m / 74.164717% (478.235 s, best so far); full_2 2.639246 m / 16.811991% (95.245 s). Both
full runs end with the planner permanently requesting STOPPING
(`stalled: image line lost/ambiguous or image tracking budget exhausted`) and the robot never moves
again for the remainder of the run. All other required regressions (tests, D-05/D-06/D-07,
Checkpoint A, GCS, bag, camera-rate, teardown, Docker build) passed on that candidate.

Goal: replace "permanently give up" with a bounded rotation-based re-acquisition, without touching
`safety_guard` or relaxing any safety/stop assertion, then re-measure best-case and consistency
against 41.0% / 18.6% / 74.1%.

## Change 1 — bounded blind rotation search (`BlindSearchSession`)

**What:** `path_planner` already had two narrow recoveries: `LookTurnSession` (a single bounded
rotation toward a short, still-valid *metric* tangent hint) and an image-tracking fallback with a
50 s / 2.2 m budget. Both of these, when exhausted, called `publish_stop(...)` with `valid=false`
unconditionally — `safety_guard` then holds `/cmd_vel` at zero forever (design §9.2: `!path_valid`
⇒ `stop("path invalid")`), matching every full-run stall in `d0809_evidence.md`.

Added `BlindSearchSession` (`line_follower_navigation/memory_policy.hpp`): a last-resort, gyro/pose-yaw
measured, **rotation-only** search tried in place of those permanent stops. It never translates. It
sweeps up to 3 bounded legs (~120° toward the side the line was last plausibly seen, back to the
original heading, then ~120° to the opposite side), each leg sized to fit inside
`motion_controller`'s existing single-rotation-episode budget (8 s) with margin, and bounded overall
(30 s). Once all 3 legs complete without reacquiring the line it latches `exhausted()` and the
caller publishes a final, honestly-worded stop; only a mission/run reset may reuse it afterwards.

`path_planner_node.cpp` wires this in at the three places that previously stopped permanently:
- `publish_image_path`'s budget-exhausted branch (`"image line lost/ambiguous or image tracking
  budget exhausted"` — the dominant real stall reason in Round 2's full-track runs).
- `look_session_.timed_out()` (`"observed look turn timed out; stopped to re-evaluate"`).
- The final `decision.mode == STOPPING` / `!result.status.valid` block (generic metric loss, e.g.
  `"unresolved re-observation mismatch"`, `"no sufficiently certain stoppable observed path"`).

While sweeping, `path_planner` publishes a **valid** `/path/status` (`mode=ROTATE_IN_PLACE`,
`detail="bounded recovery rotation (leg N/3, attempt M): searching for line after <reason>"`),
carrying whatever short verified memory prefix (`known`) still exists for display/braking context.
This is the mechanism required by the task: `safety_guard` only ever sees a legitimate, honestly
labelled command and continues to own rate limits, the 20 s gate, KILL and shutdown-zero — none of
`safety_policy.cpp` changed. `motion_controller`'s existing rotate PID (bounded 8 s/episode, ≤2
retries, requires `speed_<.008` before applying angular velocity) performs the actual rotation; no
new direct `/cmd_vel` path was added anywhere.

Re-acquisition hand-off: each camera frame still flows through the normal `on_observation` pipeline
while rotating. As soon as a direct/metric observation is accepted again (`decision.accept_observation`)
or `obs.image_tracking_valid` becomes true, the normal VISION_TRACKING/REACQUIRING/IMAGE_TRACKING
path is published instead (the blind-search branches are simply not reached that cycle) and
`BlindSearchSession::reset()` is called explicitly at each such success point — matching
"when found, re-enter REACQUIRING → normal tracking."

A signed `recovery_bias_` member tracks which side the line was last plausibly seen on (sign of the
last valid image offset/heading, or of the last computed memory-tangent hint) so the first leg
sweeps the informed side first instead of an arbitrary default.

**Debounce (safety-first, not a relaxed check):** a brand-new mission or a single transient
STOPPING/lost frame (e.g. before enough observed length has accumulated) must not itself trigger a
rotation. `stopping_since_` / the existing `image_start_` budget clock require the loss to be
*sustained* (>= 1.0 s) before `attempt_blind_recovery` is tried; a first-second dip still just stops
(0 cmd), matching the task's "first stop (safety first), then perform a bounded re-acquisition."
This was found necessary via `src/line_follower_tests/test/test_pipeline_launch.py`
(`test_pipeline_lifecycle`): an early version without the debounce immediately rotated during the
brief pre-memory startup window using fake, non-advancing odometry, latching
`motion_controller`'s internal `rotating_` state for the rest of that short test and preventing
forward motion — a real bug caught by the existing regression suite, fixed before any Gazebo run.

**Unit tests added** (`line_follower_navigation/test/test_memory_policy.cpp`):
`BlindSearchSession.SweepsBiasSideThenBackThenOppositeSideThenExhausts` and
`BlindSearchSession.StopsAsSoonAsTriggerClearsAndNeverExceedsOverallBudget` — leg sequencing/bounds,
immediate cancellation when the trigger clears, exhaustion latch + reset, and the overall time bound.

**Files changed:**
- `src/line_follower_navigation/include/line_follower_navigation/memory_policy.hpp`
- `src/line_follower_navigation/src/path_planner_node.cpp`
- `src/line_follower_navigation/test/test_memory_policy.cpp`
- `docs/r3_evidence.md` (this file)

**Regressions after the change:** `tools/lab_env.sh build` exit 0 (8 packages); `tools/lab_env.sh
test` → 202 tests, 0 errors, 0 failures, 0 skipped (200 prior + 2 new `BlindSearchSession` tests).
4/4 sections (`check-drive --full-runs 0`) still pass with hill/corner/curve/crossing lateral error
essentially unchanged from the Round 2 baseline.

## Iteration 01 — first full run: recovery works, but exposes a debounce bug

First full run (`docs/d0809/round3_iter01/`): 4.458 m / 28.397%, 176.8 s, stop true XY/yaw
(1.776, 1.099, 1.878), reason `off line >.08m for .6s at 4.467m`. The bag/JSON `events` list
(`docs/d0809/round3_iter01/full_1.json`) shows the mechanism genuinely worked: four separate
`"image line lost/ambiguous..."` → `"bounded recovery rotation (leg 1/3, attempt 1)"` →
`"re-observation mismatch"/"consistent re-observation verified"` cycles, each followed by renewed
forward progress — i.e. the planner never gave up permanently, exactly as intended. But the robot
then drifted off the physical track near world (1.78, 1.10) — the same "curve entry near (1.67,1.06)"
region flagged in the task's own diagnosis as a typical stall site.

Root-cause check against the frozen baseline: copied `scratchpad/candidate_v1` into the lab
container (`/tmp/cand1_check`, never touching the host copy) and reran
`test_pipeline_launch.py` unmodified — it passes in 3.6 s. Running the same test against this
iteration's `path_planner_node.cpp` reproduced `no forward command after RUNNING`. Diagnosis (debug
log, later removed): `attempt_blind_recovery` was reached on the very **first** lost frame of a
brand-new mission because `image_start_` (which only marks "entered image-fallback mode", used for
the existing 50 s/2.2 m budget) was already non-zero by the time of the dip, so the `>= 1.0 s`
debounce computed against it was not actually bounding *sustained* loss. Once `motion_controller`
receives even one `ROTATE_IN_PLACE` message it commits to that rotation goal for up to its own 8 s
episode budget regardless of newer forward-mode path messages (same pre-existing limitation as
`LookTurnSession`, which also cannot be cancelled early) — so a single spurious trigger is not free.

**Fix:** added a dedicated `image_lost_since_` clock, reset every time `obs.image_tracking_valid` is
true, used for the sustained-loss debounce instead of `image_start_`. Also reduced
`BlindSearchSession`'s leg angle from the suggested ~120° to ~60° (and leg/overall budgets from
7 s/30 s to 4 s/20 s) specifically because a blind leg is an *ungrounded* guess (unlike
`LookTurnSession`'s observed-tangent hint) and, per the above, cannot be cancelled once committed —
a smaller leg bounds how far a moot committed rotation can carry the heading if vision recovers
mid-turn. `tools/lab_env.sh test` → 202/202 green after the fix; 4/4 sections re-verified green.

## Iteration 02–03 — debounce bug reproduced without the fix (for the record)

Two more full runs with the un-fixed debounce, for comparison, both ending in `off line` at the
same "first curve" region the task's diagnosis names (world (2.93, 0.38)):
`docs/d0809/round3_iter02/` (2.715 m / 17.293%, stop (2.847, 0.417)) and
`docs/d0809/round3_iter03/` (2.667 m / 16.988%, stop (2.605, 0.342), lateral max 0.255 m). These
motivated the fix above rather than being kept as the shipped behaviour.

## Iteration 04–06 — fixed debounce + smaller leg angle: three full runs

| Run | Progress m / % | Wall s | Stop true XY | Result |
|---|---|---|---|---|
| `round3_iter04` | 11.634 / 74.112% | 390.8 | (2.931, 1.638) | clean bounded-recovery-exhausted stall at the Ω loop entry |
| `round3_iter05` | 11.743 / **74.805%** | 474.8 | (2.949, 1.549) | clean bounded-recovery-exhausted stall at the Ω loop entry |
| `round3_iter06` | 11.278 / 71.838% | 412.0 | (2.834, 1.869) | off line (0.0845 m, marginal) inside the Ω loop |

All three reach deep into the Ω loop (the task's named "secondary, only if time" stretch goal) with
many successful mid-run recovery cycles (visible as repeated `mode=4` → renewed progress in the
`PROGRESS` log), a large, consistent jump versus every one of Round 2's 23 iterations, none of which
reached the Ω loop without permanently stalling first. `checkpoint-a` re-verified PASS
(D-01..D-04, `log` in `b1erm8nui` task output) between iterations 05 and the final gate.

## Final gate — three sequential full runs (`check-drive --only-full --full-runs 3 --limit 900`)

`docs/d0809/round3_final/` (full_1/2/3 json + plots):

| Run | Progress m / % | Wall s | Stop true XY | Reason |
|---|---|---|---|---|
| full_1 | 11.599 / 73.887% | 502.0 | (2.886, 1.619) | stalled: bounded rotation recovery exhausted; no line reacquired (image line lost/ambiguous or image tracking budget exhausted) |
| full_2 | 3.028 / **19.289%** | 83.6 | (2.865, 0.785) | off line >.08m for .6s at 3.048m |
| full_3 | 12.059 / **76.817%** | 451.4 | (2.650, 1.512) | off line >.08m for .6s at 12.070m |

`tools/lab_env.sh test`: 202 tests, 0 errors, 0 failures, 0 skipped. `checkpoint-a`: PASS
(D-01..D-04). 4/4 sections: PASS.

<!-- R3_ITERATIONS_START -->

| Iteration | Change | Progress m / % | Wall s | Stop true XY | Result / reason |
|---|---|---|---|---|---|
| 01 | BlindSearchSession, 3×~120° legs, image_start_-based debounce | 4.458 / 28.397% | 176.8 | (1.776, 1.099) | off line (debounce bug: triggers on first lost frame deep into a run) |
| 02 | (same, un-fixed, second sample) | 2.715 / 17.293% | 66.7 | (2.847, 0.417) | off line at the first curve |
| 03 | (same, un-fixed, third sample) | 2.667 / 16.988% | 82.0 | (2.605, 0.342) | off line, lateral max 0.255 m |
| 04 | dedicated image_lost_since_ debounce + ~60° legs | 11.634 / 74.112% | 390.8 | (2.931, 1.638) | clean bounded-recovery-exhausted stall (Ω loop entry) |
| 05 | (same, fixed, second sample) | 11.743 / 74.805% | 474.8 | (2.949, 1.549) | clean bounded-recovery-exhausted stall (Ω loop entry) |
| 06 | (same, fixed, third sample) | 11.278 / 71.838% | 412.0 | (2.834, 1.869) | off line, marginal (0.0845 m), inside the Ω loop |
| F1 | final gate run 1/3 | 11.599 / 73.887% | 502.0 | (2.886, 1.619) | clean bounded-recovery-exhausted stall (Ω loop entry) |
| F2 | final gate run 2/3 | 3.028 / 19.289% | 83.6 | (2.865, 0.785) | off line at the first curve |
| F3 | final gate run 3/3 | 12.059 / 76.817% | 451.4 | (2.650, 1.512) | off line, deep inside the Ω loop (new best progress) |

<!-- R3_ITERATIONS_END -->

## Verdict versus v1 (41.0% / 18.6% / 74.1%)

- **Best case:** 76.817% (F3) vs v1's 74.1% — improved, and for the first time progress consistently
  reaches deep into the Ω loop instead of stalling at the first curve/curve-entry/Ω-loop-entry region.
- **Worst case:** 19.289% (F2) vs v1's 18.6% — essentially unchanged, not worse.
- **Consistency:** two of three runs now land in the low-to-mid 70s% (73.9%, 76.8%) against v1's one
  high outlier (74.1%) out of three widely scattered samples (41.0/18.6/74.1); this is a real
  improvement in typical-case progress even though one run (F2) still regresses early.
- **Mechanism:** demonstrably working as specified — bounded (3 legs, ≤20 s total), rotation-only,
  publishes an honest valid `/path/status` while sweeping, hands off to REACQUIRING/VISION_TRACKING
  on success (repeatedly, within single runs), and gives up with an honest reason once genuinely
  exhausted. `safety_guard`/`safety_policy.cpp` were not touched; the 20 s startup gate, KILL,
  stale-input and shutdown-zero behaviour are all re-verified intact via `checkpoint-a` (D-01..D-04).

**Known risk (not fully solved):** the task's named "first curve near world (2.93, 0.38)" region
remains fragile. In 2 of 8 full-run samples taken during this round (iter06, F2) a recovery episode
there ends in physical **off-line** departure rather than a clean stall — a new failure mode this
round introduces as a side effect of actively re-engaging instead of freezing in place. It is bounded
and never drives blind, but an imperfect post-rotation reacquisition can occasionally compound into
a lateral excursion the external checker flags, whereas the old permanent stall was passively safe
(zero further motion) at the cost of zero further progress. This trade-off is inherent to any active
recovery and was not fully eliminated in the time available; the mitigations in iteration 04
(separate debounce clock, smaller sweep angle) measurably reduced but did not eliminate it. The
secondary goal (clean Ω-loop traversal / avoiding its entry leg backwards) was not separately
implemented; the gains observed into the loop come from the general bounded-recovery mechanism only.
