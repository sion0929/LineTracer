# D-08 / D-09 incremental evidence

## Round 2 final outcome — 2026-11-21

**FAILED:** `tools/lab_env.sh check-drive` exits **1**. All four required sections pass, but neither full competition reaches the finish area. Final full 1 reaches **11.642818 m / 74.164717%** (478.235 s; lateral p95/max **0.036519/0.039455 m**); final full 2 reaches **2.639246 m / 16.811991%** (95.245 s; **0.027913/0.040146 m**). Best full progress is final full 1. Two consecutive ≥95% runs were not achieved.

All other required final gates pass: **200 tests, 0 errors/failures/skips**, D-05/D-06/D-07, Checkpoint A, GCS (26/26), bag/replay, camera rate, teardown (10/10), and production Docker build. These results do not establish D-08/D-09 completion. The complete final run table and diagnostic findings are at the end; earlier failed approaches remain historical evidence. No §17 checkbox was changed.

The clean-output default gate verified directory creation with `docs/d0809`, `log_lab/d0809`, and `bags/drive` absent. All prior evidence was restored afterwards; conflicting earlier root files remain preserved under `log_lab/d0809-r2-clean-inputs-20261121-043523/`.

## Round 1 historical evidence

Status: **FAILED** final drive gate (exit 1): three sections pass, crossing stalls, neither full run finishes. All other requested final regressions and the production Docker build passed; see the final table below. Runtime code uses no offline mesh oracle; the earlier experiments below are retained as history.

## Early implementation / failures

The image-feature test failed before fields/algorithm were added, branch-continuity tests failed before selection/lock support, and the sharp-corner test failed before target clamping/rotation was implemented. Initial full tests after implementation passed 146 cases with zero failures.

The offline ordered oracle initially failed because triangle file order reverses some arc blocks. It now walks unused endpoint-connected strips, selecting nearest endpoint first and straight continuity at the self-crossing, and reaches finish at 15.699 m XY arc length. This logic stays in tools.

Two early autonomous hill attempts stopped at 0.468 m progress (world x≈0.798), lateral error <1 mm. The bag showed image validation turned false. The saved camera shows a clear centred line plus an unrelated stroke at the extreme left in one ROI row; a new test reproduced this false rejection. Pixel following now uses the central 80% corridor, retaining rejection of multiple strokes in that corridor and of a transverse/broad line. No safety assertion was loosened; path validity stays false on actual lost/ambiguous target lines.

Current implementation includes image-only offset/heading fields, IMAGE_TRACKING=7 (GCS mode list extended), bounded 2.2 m / 50 s image sessions, metric reacquisition checks, forward continuity/candidate locking, polyline corner target clamping, and stop-before-rotate feedback from localization and odom. Slow-driver memory retention is 20 s; uncertainty grows at 1.5 mm/s plus 10 mm/m (gyro mapping measurements motivate this baseline). Final tests/measurements pending.

The complete hill section first passed at 1.902 m progress in 33.55 s, lateral p95 0.000540 m / max 0.000557 m (prototype controller), and passed again after virtual image-space Pure Pursuit. The lower-band validator now selects the component reaching the bottom centre and accepts a raised horizon while requiring the bottom stroke; two plausible bottom strokes and missing-bottom cases stay rejected. Brief conservative stops remain in the timeline.

Early other-section attempts failed on projection/image ambiguity; the corner harness also initially approached opposite the ordered route and was corrected to the oracle's incoming direction. The runtime policy now retains usable trusted metric memory ahead of image fallback instead of discarding a known corner when an unrelated image component invalidates global projection. No confirmed geometry is added from an untrusted frame. More section tests follow.

A debugger replay of the actual stopped corner camera frame found 304/518 skeleton pixels inside a broad grey terrain-shadow component, despite a normal near-line half-width of 11.46 mm and flat IMU/height. The grey hill shadow had been thresholded as black path and invalidated all metric observations. A portable grey-shadow-plus-black-line test reproduced the error; black segmentation now caps the strong threshold at 50, with contrast recovery reserved for lighter faint strokes (110–170), rather than recovering grey terrain shadow. The faint-line unit remains required.

Two preliminary real-competition runs both honored the default 20 s gate and crossed the hill, then stalled near the first curve: progress 2.667 m (16.99%) in 70.45 s and 2.572 m in 63.93 s. Their JSON/plots/bags are preserved by the run logs; these are earlier failures, not a completion claim.

A new controller-only launch test reproduced unsafe rotation candidates with no localization feedback. The legacy fall-through was removed; the test now passes all phases: missing feedback → zero, actual linear motion → zero rotation, stopped fresh feedback → bounded rotation, eight-second timeout → zero, new run → reset.

After excluding grey shadow, the hill, right-angle corner and straight-through crossing sections passed; crossing now records the explicit locked-straight reason. The curve section revealed a distinct root-selection bug: a long curved segment was rejected because its whole-segment chord was perpendicular, even though the tangent under the rover pointed forward. A curved-segment test failed before the fix; root eligibility and orientation now use the local tangent at the nearest point. The diagonal physical-width stroke regression already passed before any width-threshold adjustment, so no such adjustment was made.

The local-tangent section command `tools/lab_env.sh check-drive --full-runs 0 --limit 120` passed all four sections: hill 1.903 m / 35.08 s / lateral p95 0.000987 m; corner reached arc 6.140 m / 21.45 s / p95 0.023370 m / max 0.028000 m; curve reached arc 2.993 m / 32.47 s / p95 0.026505 m / max 0.027562 m; crossing reached arc 13.675 m / 32.86 s / p95 0.000543 m / max 0.002096 m with the locked-straight reason. Log: `log_lab/d0809-sections-local-tangent.log`. Two new default-20-second full competitions are running.

The observer's liveness wait is now 25 s (20 s memory retention plus a 5 s margin), rather than prematurely ending at 15 s before retained conflicting memory could expire/reverify. The 0.08 m / 0.6 s off-line failure bound and every stop/sensor safety assertion are unchanged. Full runs remain capped at 600 s.

The first full local-tangent trial still stopped near 2.57 m: usable curve memory was being abandoned at uncertainty 0.04 m in favour of image following, but the curved image was correctly ambiguous. The actual memory validity cutoff is 0.06 m; a new preference test failed before the helper existed, and the policy now keeps trusted metric memory until that existing cutoff or insufficient metric geometry. Speed still shrinks continuously with uncertainty; no validity/safety cutoff was raised. The observer's earlier 15 s liveness wait was also shorter than the configured 20 s retention; subsequent full trials use 25 s, with the off-line bound unchanged.

## Earlier full-track results before the final segmentation correction

`tools/lab_env.sh build` completed eight packages (25.2 s); `tools/lab_env.sh test` reports **158 tests, 0 errors, 0 failures, 0 skipped**. Logs: `log_lab/d0809-build-memory-validity.log` and `log_lab/d0809-test-final.log`. The only later runtime-source edits remove stale comments.

Command: `tools/lab_env.sh check-drive --only-full --full-runs 2 --limit 600` (exit **1**, `log_lab/d0809-full-memory-validity.log`). Both runs used the real competition launch with its unmodified default 20 s delay. This task is not a full-track success.

| Run | Progress m / % | Finish reached | Wall seconds after RUNNING | Lateral p95 / max m | Stop reason |
|---|---|---|---|---|---|
| full_1 | 2.943643 / 18.751% | No | 95.139 | 0.021278 / 0.027347 | stalled: no sufficiently certain stoppable observed path |
| full_2 | 2.993574 / 19.069% | No | 133.516 | 0.015107 / 0.026747 | stalled: no sufficiently certain stoppable observed path |

Default gate rows: elapsed 20.206 s and 20.205 s (see exact log rows). The XY mesh oracle length is 15.698594 m. Both crossed the entire hill and first curve, then stopped near the following right-angle region (world x≈2.99, y≈0.68–0.73). Bag inspection shows metric projection is valid, but the near detected segment is almost transverse and disconnected from retained forward geometry; after memory expires there is no safe forward chain. The image tracker correctly rejects that transverse/ambiguous image. No unobserved connector or guessed turn was inserted. A next fix should preserve the upcoming observed corner and its incoming/outgoing connection through the curve and blind zone, including detector fragmentation/mapper merge diagnostics; simply relaxing uncertainty or allowing blind image turns would conceal the failure.

Diagnostic bags: `/ws/bags/drive/full_1-151656` and `/ws/bags/drive/full_2-151656` (host paths replace `/ws` with the workspace). These contain perception, odom/IMU/localization, selected path/candidates, mission, candidate/final velocity and safety; camera is saved separately as each run's PNG. JSON mode events and `<run>_timeline.png` provide the mode timeline. Raw trajectory arrays and launch logs are under `log_lab/d0809/`. Earlier full-run bags/logs are retained, not reclassified as passing.

## Final implementation and tests

The detector extracts only a single bottom-centre image component, scans lower rows, fits a line and
rejects missing-bottom, multiple-stroke, broad/transverse and poor-fit images. Pixel offset/heading
are independent of `projection_valid`; invalid 3D points never enter confirmed memory. The image
controller uses a bounded virtual Pure Pursuit servo target internally (not published as geometry).
Relative elevation integrates odom travel and IMU pitch after flat-rest calibration: a level chassis
on the elevated hilltop stays projection-untrusted. A restart on an elevated surface is not validated.

Metric root selection uses a local tangent, recorded graph connectivity and confirmed points only.
At multiple observed legs, reject short (<0.08 m) or backward (>2.2 rad) candidates, prefer heading
continuity and length cost, stop for a score ambiguity <0.25, and retain the selected ID while feasible.
All observed options are published with rule scores, feasibility, selection and reasons; MLP is off.
Pure Pursuit uses the ordered connected polyline and 0.12 m nominal lookahead. Sharp turns >1.05 rad
inside 0.45 m clamp the target to the observed vertex, slow, then request rotation within 0.035 m.
The controller waits for measured speed <0.008 m/s, caps rotation at 0.45 rad/s, stops within 0.08 rad,
and faults to zero after eight steady-clock seconds. Fresh pose/odom are mandatory. `safety_guard`
continues to own final commands, rate limits and timeouts; it was not weakened.

New tests cover:

- Slope image tracking without confirming projection, missing/ambiguous image rejection, peripheral
  unrelated strokes, raised horizon, grey-shadow segmentation, faint/diagonal physical stroke regressions.
- Relative terrain elevation across uphill/level top/downhill and rest-pitch calibration.
- Crossing continuity/candidate publication, outgoing forward leg after rotation, local curve tangent root selection.
- Sharp observed corner approach and rotation target; usable curve memory before image fallback.
- Image steering signs/bounds and a real controller launch with missing feedback, moving odom,
  stopped feedback, eight-second timeout zero and new-run reset.

Failing-first evidence is retained in `log_lab/d0809-red-*`; synthetic off-axis/diagonal cases that
already passed are explicitly not claimed as red tests. Controller launch final pass is also in
`log_lab/d0809-green-rotation-feedback-final.log`. Fast-SCNN remains a stub/fallback; no learning or
pre-known route coordinates were added to runtime code.

The four-section gate uses autonomous application nodes and `safety_guard`, not the observer's own
Twist publisher. Section spawn coordinates and the ordered 15.699 m mesh oracle live only in tools.
Plots: `docs/d0809/{hill,corner,curve,crossing,full_1,full_2}_topdown.png`, matching `_timeline.png`,
`_camera.png` and `.json`. The full-run recorder omits camera to bound diagnostic bag size; final
camera images are saved separately. `bags/` stays gitignored/dockerignored.

Files added: `tools/check_drive.py`, `tools/drive_capture.launch.py`,
`src/line_follower_perception/include/line_follower_perception/terrain_trust.hpp`,
`src/line_follower_tests/test/test_controller_rotate_launch.py`, this evidence and `docs/d0809/` plots/JSON.
Files changed: perception backend/node/CMake/package.xml/test backend; interfaces `DriveMode.msg`,
`LineObservation.msg`, `LocalPath.msg`; navigation memory/policy headers, memory/planner/trajectory
sources and their three gtests; control Pure Pursuit header/source, controller, CMake and safety-policy
unit file (image-servo tests only); tests CMake; competition YAML (lookahead and memory retention);
`tools/check_d05.py` (native truth subscription cleanup), `tools/lab_env.sh`, GCS `model.py` mode list,
`README.md`, `docs/test_plan.md`. No provided assets, hashes or design checkboxes were changed.

## Regression discovery and segmentation correction

The first regression attempt failed D-05 corner recall (0.2938) and D-06 straight blind-zone memory (no local points in the final near range), despite passing geometric error/source/size checks and sharp-corner memory. These are real earlier failures, not tolerance changes. A new dark-grey (80) one-pixel distant-stroke test failed under the 110 lower cutoff. Merely lowering that cutoff restored terrain-shadow false positives, so the final segmentation excludes whole broad grey components (distance-transform radius >3 px) before thin-stroke contrast recovery, while retaining the strong-black threshold of 50. All 25 detector/terrain tests then passed, including both shadow rejection and dark/faint stroke preservation. Logs: `log_lab/d0809-red-faint-grey.log`, `log_lab/d0809-green-grey-components.log`. A new frozen build, all regressions and a default four-section/two-full check from absent output directories are scheduled; earlier full results above remain historical until that final run.

The frozen correction build passed eight packages in 24.1 s, and the complete test command passed
**159 tests, 0 errors, 0 failures, 0 skipped** (30.1 s). Final D-05 and D-06 commands now exit 0;
the straight blind-zone points span 0.0120–0.2574 m and the corner memory vertex error is 0.00145 m
with an 85.34° retained turn. These replace the earlier failing regression attempts without changing
any assertion or numeric ground-truth tolerance. Final command logs use `log_lab/d0809-final-*`.

## Final clean-directory drive gate — FAILED

Exact command: `tools/lab_env.sh check-drive --limit 600` (exit **1**). This is the default four-section / two-full sequence. Before invocation, `docs/d0809`, `log_lab/d0809`, and `bags/drive` were moved aside; all three were absent, and the checker created them itself. The wrapper log `log_lab/d0809-final-frozen-wrapper.log` records those three absent checks and subsequent restoration. Prior evidence is preserved under `log_lab/docs_d0809_before_absent_archive/` and `log_lab/log_lab_d0809_before_absent_archive/`; earlier bags were moved back without deleting any recording. The authoritative final log is `log_lab/d0809-final-check-drive.log`.

| Run | Result | Progress m / % of 15.698594 m | Goal arc m | Seconds after RUNNING | Lateral p95 / max m | Reason |
|---|---|---|---|---|---|---|
| hill | PASS | 1.902946 / 12.122% | 1.900000 | 37.255 | 0.000324 / 0.000339 | section traversed |
| corner | PASS | 6.141996 / 39.124% | 6.138270 | 29.086 | 0.022172 / 0.029018 | section traversed |
| curve | PASS | 2.992424 / 19.062% | 2.990000 | 51.871 | 0.026685 / 0.028832 | section traversed |
| crossing | FAIL | 13.538645 / 86.241% | 13.670324 | 45.422 | 0.000394 / 0.000394 | stalled: trusted current observation |
| full_1 | FAIL | 3.226408 / 20.552% | 15.578594 | 106.627 | 0.020106 / 0.028246 | stalled: trusted current observation |
| full_2 | FAIL | 4.616861 / 29.409% | 15.578594 | 166.692 | 0.013726 / 0.033078 | stalled: no sufficiently certain stoppable observed path |

Each table row has a corresponding `.json`, `_camera.png`, `_topdown.png` and `_timeline.png` under `docs/d0809/`. Final bags:

- `/ws/bags/drive/hill-162368`
- `/ws/bags/drive/corner-162368`
- `/ws/bags/drive/curve-162368`
- `/ws/bags/drive/crossing-162368`
- `/ws/bags/drive/full_1-162368`
- `/ws/bags/drive/full_2-162368`

The crossing selected `locked straight-through line continuity`, travelled through the crossing centre but stopped before the required exit distance. At stop `/path/status.valid=false`, `/safety/status.stop_reason="path invalid"`; the local chain did not meet the feasible-length requirement (verified remaining distance included the first observed point's forward distance). Its `detail="trusted current observation"` is misleading when feasibility subsequently fails; this diagnostic needs correction. No exit span or safety assertion was relaxed.

Full 1 actually stopped because the controller's eight-second rotation episode timed out, then correctly kept candidates zero despite later valid paths. Launch log contains `rotation feedback stale or eight-second budget exhausted; zero until new run`; the observer's final planner-only reason does not identify this controller fault, so the controller/guard bag and launch log are authoritative. The episode budget persists across transient STOPPING paths to prevent unlimited relatching, but needs a tested completion/cancellation distinction when genuine forward progress resolves a previous rotation. Do not fix this by removing the time bound or clearing faults on every path update. Full 2 passed the first right-angle turn, then exhausted its usable path in the next curved/corner region. Upcoming corner connectivity must survive the preceding curve and camera blind zone; disconnected fresh geometry is not permission to invent a connector.

Final stopped true poses (world x, y, yaw):

- crossing: (2.089606, 2.428829, 1.580018).
- full_1: (2.983014, 0.957949, 2.971635).
- full_2: (1.660207, 1.064733, 2.173403).

Open verification gaps: T-21/T-22 staged partial/new-leg observations and noisy lock persistence were not separately exercised in Gazebo; the branch unit test checks initial continuity/candidate publication, not repeated noisy lock updates. The terrain baseline assumes initial flat-rest calibration; restart on an elevated surface is unverified. Full-track completion and D-10 finish recognition are not claimed. Current results do not satisfy the autonomous drive gate even if every regression below passes.

Both final full runs reported `elapsed=20.204s` for their real competition RUNNING transition.
The mode plots show planner modes; a latched controller rotation fault can leave planner mode VISION
while actual candidates remain STOPPED/zero. Consult `/control/cmd_vel_candidate` in the bag and the
launch log for that fault; propagating a distinct controller fault to the operator display remains open.

Recommended next tests/fixes before claiming D-08/D-09 complete: reproduce the crossing's short
forward fragment plus longer connected memory in a unit test and retain the feasible continuous chain;
add a controller test for a partial rotation interrupted by path STOPPING followed by measured forward
progress, then a genuinely new corner (without allowing repeated stale paths to reset the same timeout);
distinguish smooth high-curvature arcs from sharp observed vertices in trajectory tests; publish the
actual infeasibility/controller-fault reason instead of a stale trusted-observation detail. Re-run all
four sections and at least two full competitions after those changes. These fixes are not claimed here.

Final camera-rate regression: composed reliable camera bag 715 frames over 23.793649 s = 30.008 Hz;
perception status bag rate 29.461 Hz; detector processing p50/p95 6.354/7.971 ms. The external C++
sensor-data observer received 21.850 Hz, while the reliable Python observer received 30.056 Hz;
`ros2 topic hz` also under-reported this composed run. The reliable recording gate passed; this is
not a claim that every external observer receives 30 Hz. Segmentation now costs more CPU time than
the earlier 2.63/4.79 ms processing sample; no processing-speed improvement is claimed.

## Final regression results and rerun commands

Each command below ran to completion; authoritative exit codes are in `log_lab/d0809-final-status.log`, and individual output is in `log_lab/d0809-final-<check>.log`. The initial D-07 regression wrapper was interrupted before completing its output and is inconclusive; the frozen D-07 rerun completed and exited 0.

| Exact command | Exit | Key evidence |
|---|---|---|
| `tools/lab_env.sh build` | 0 | 8 packages, 24.1 s |
| `tools/lab_env.sh test` | 0 | 159 tests; 0 errors, 0 failures, 0 skipped |
| `tools/lab_env.sh check-d05` | 0 | All flat poses pass; corner recall 1.0, p95 error 0.007724 m |
| `tools/lab_env.sh check-d06` | 0 | Straight/corner/rotation pass; blind memory 0.0120–0.2574 m; corner vertex error 0.00145 m |
| `tools/lab_env.sh check-d07` | 0 | Heading/hill/blind-end gates pass; gyro 0.5 rad/s heading p95 0.001089 rad, raw spread 0.002955 m |
| `tools/lab_env.sh check-drive --limit 600` | **1** | Clean output dirs; 3/4 sections; full runs 20.55% / 29.41%, neither finishes |
| `tools/lab_env.sh checkpoint-a` | 0 | D-01 through D-04 pass; controller loss zero 0.205 s within 0.300 s; guard exit physical stop 0.059 s |
| `tools/lab_env.sh check-gcs` | 0 | 26/26 integration checks plus model/offscreen tests |
| `tools/lab_env.sh check-bag` | 0 | `/cmd_vel` publishers/messages 0/0; blocked commands 424; regenerated line/status 355 each, local/path status 399 each |
| `tools/lab_env.sh check-camera-rate` | 0 | Reliable composed bag 30.008 Hz; processing p50/p95 6.354/7.971 ms |
| `tools/lab_env.sh check-teardown` | 0 | 10/10 clean composed SIGINT teardown |
| `docker build -t line-follower-d0809:local .` | 0 | 6 production packages; Release, BUILD_TESTING=OFF, packages-up-to bringup; no GCS/Qt dependency |

The new gate also accepts `--section hill|corner|curve|crossing`, `--full-runs 0`, `--only-full`, `--full-runs 2`, `--limit`, `--stall-sec`, and `--domain`; the default remains all four sections plus two 600 s real-competition runs. Section starts alone shorten the delay and use dev-only placement. Full launches retain the official one-line launch and default 20 s gate. `README.md` and `docs/test_plan.md` document the commands and limits.

Final outcome: **FAILED**, because the final autonomous drive command exited 1. No acceptance claim is made for D-08/D-09, and no §17 checkbox was ticked. The original worlds/models/hashes remain unchanged (static policy and SHA checks passed). Final lab-container process inspection contains only docker-init, its sleep process and the inspection command; no ROS/Gazebo/GCS/recorder/player remains. Cleanup evidence: `log_lab/d0809-final-cleanup.log`.

# Round 2

Active objective: two consecutive full-track completions; earlier Round 1 failures remain above. Fix 1 begins with an extended controller launch test covering timeout with a genuinely changed observed heading and partial rotation → STOPPING → measured forward progress → new corner. Safety limits and all Gazebo assertions stay unchanged.

Round 2 fix 1: extended controller launch test failed on the old permanent latch, then passed all phases in 21.9 s (`log_lab/d0809-r2-rotation-{red,green}.log`). The 8 s budget remains, same-target stamp refresh cannot reset it, a fresh changed absolute heading can receive one bounded retry after cooldown, and 3 cm measured forward travel resets the old episode. Autonomous corner run passed: arc 6.139122 m, 28.979 s, lateral p95/max 0.021766/0.028642 m (`log_lab/d0809-r2-fix1-corner.log`, bag `bags/drive/corner-168907`).

Fix 2 tests: an isolated 4 cm fragment hid a longer forward chain (old output 5 points), and the new verified-length helper was absent; both red results are in `log_lab/d0809-r2-feasibility-red.log`. Verified length now sums observed polyline edges only, never the unseen gap before its first point. Root selection skips isolated fragments shorter than 8 cm when seeking a continuous path. Infeasible status preserves the actual planner/validity reason.

Round 2 fix 2 crossing run passed arc 13.670630 m, 32.599 s, lateral p95/max 0.002248/0.006088 m, with locked-straight reason (`log_lab/d0809-r2-fix2-crossing.log`, bag `bags/drive/crossing-169801`). Navigation unit suites passed.

Fix 3 starts with two synthetic failures: one curved continuous stroke and one visible right-angle were both rejected by the old straight fit (`log_lab/d0809-r2-image-red.log`). The new image tracker skeletonizes the selected lower-image component, traces its actual pixels from the nearest bottom point toward its unique outgoing endpoint, and chooses a look-ahead point on that stroke. Multiple bottom components, branched traces and an isolated transverse bar with two plausible directions remain rejected. Tight image turns cap speed at 0.035 m/s. The virtual image-servo lookahead is 0.08 m: the old 0.20 m virtual lookahead could never command the curvature of a small-radius bend. These are control-scale constants, not observed ground coordinates.

Fix 3's first pixel trace failed five existing/new image tests because thinning preserved a thick border row. Clearing the one-pixel thinning border restored all 27 detector/terrain tests, without changing visibility/ambiguity assertions (`log_lab/d0809-r2-image-{border-failure,green}.log`). The earlier in-flight curve section (pre-border rebuild) passed arc 2.992669 m / 33.780 s / lateral p95/max 0.026050/0.026898 m; a fresh rebuilt full iteration is now the actual fallback test. Observer bags now include `/path/memory` and `/path/observed_graph` for continuity diagnosis.

Round 2 full iteration 01 (rotation recovery + feasible root/strict observed arc + curved pixel tracing): progress 4.586033 m (29.213%), PASS=False, 143.154 s, lateral p95/max 0.022600/0.034963 m; stop true XY/yaw [1.6818138452527305, 1.0440232436371857, 2.329252050964362]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-171522`; plots/JSON `docs/d0809/round2_iter01/`; log `log_lab/d0809-r2-iter01.log`.

Iteration 01 memory diagnosis: preview storage at the 32-segment cap evicted an older near curve (0.286 m long, 0.021 m from the robot, uncertainty 0.026 m) in favour of fresh distant fragments. The actual store still held the near curve because the early image-stop return prevented commit, reproducing the same bad preview indefinitely. A new capacity test failed before changing eviction (`log_lab/d0809-r2-capacity-red.log`). Eviction now drops the spatially farthest segment first, using age only to break near-equal-distance ties; caps, expiry, uncertainty and source gates are unchanged. A signed forward-progress test also failed before adding the helper; episode reset now requires 3 cm along the measured initial heading, not lateral/backward displacement. Iteration 02 is the next full run.

The complete mapping-memory suite and control safety/servo/progress suite passed after the capacity and signed-progress changes (`log_lab/d0809-r2-capacity-progress-green.log`); iteration 02 build passed eight packages in 17.0 s. No memory-capacity or safety threshold was increased.

Round 2 full iteration 02 (spatial memory eviction preserves the blind-zone curve; signed forward-progress reset): progress 1.678571 m (10.692%), PASS=False, 60.468 s, lateral p95/max 0.000737/0.000774 m; stop true XY/yaw [2.0098137537453993, 0.32962679024135594, -0.0075700661511941296]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-172980`; plots/JSON `docs/d0809/round2_iter02/`; log `log_lab/d0809-r2-iter02.log`.

Iteration 02 stopped at 1.678571 m / 10.692% on the downhill, with a clearly visible centred trapezoid stroke. The pixel skeleton's widening bottom cap produced extra short endpoints; the new trapezoid test failed (`log_lab/d0809-r2-cap-spurs-red.log`). Tracing now treats only local bottom-cap endpoints as incoming; exactly one distinct outgoing direction is still required. Two distant bar directions or a junction remain ambiguous. This changes image-trace topology handling, not a safety/GT tolerance. Iteration 03 follows.

All 28 detector/terrain tests pass after bottom-cap handling (`log_lab/d0809-r2-cap-spurs-green.log`). Iteration 03 passed the previous downhill stop and reached metric reacquisition beyond the hill; final per-run values are appended automatically when it settles.

Round 2 full iteration 03 (perspective-stroke bottom cap spur handling, preserving unique outgoing trace): progress 5.717975 m (36.423%), PASS=False, 186.464 s, lateral p95/max 0.022887/0.030270 m; stop true XY/yaw [1.08773585152908, 1.646239796914393, -3.038930212494697]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-173892`; plots/JSON `docs/d0809/round2_iter03/`; log `log_lab/d0809-r2-iter03.log`.

Iteration 03's next-corner diagnosis: near memory segment 547 (0.182 m) remained connected to segment 548 (four points, 0.148 m), with a 1 cm endpoint gap and moderate uncertainty. Traversal's ≥25% overlap heuristic counted the compulsory shared corner endpoint as duplicate geometry, rejecting the entire four-point outgoing leg. A synthetic four-point outgoing-leg test failed (`log_lab/d0809-r2-junction-anchor-red.log`). The duplicate check now excludes only the shared endpoint neighbourhood and evaluates the remaining outgoing geometry; the existing parallel retracing tests remain. Iteration 04 first reruns the corner section, then the full track.

All 21 mapping tests passed after the shared-endpoint correction (`log_lab/d0809-r2-junction-anchor-green.log`), including the prior duplicate-parallel retracing checks. Corner section rerun passed: arc 6.140146 m, 24.526 s, lateral p95/max 0.022351/0.028339 m (`log_lab/d0809-r2-fix4-corner.log`, bag `bags/drive/corner-175229`).

Round 2 full iteration 04 (shared corner endpoint excluded from duplicate retracing check): progress 3.173406 m (20.215%), PASS=False, 96.846 s, lateral p95/max 0.011753/0.013789 m; stop true XY/yaw [2.9633961301177814, 0.912287972296243, 1.676527066118919]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-175497`; plots/JSON `docs/d0809/round2_iter04/`; log `log_lab/d0809-r2-iter04.log`.

Iteration 04 diagnosis corrected by timestamp/edge inspection: the outgoing segment did NOT expire. Its six points were valid until 98.6–101.7 s at removal time 81.8 s, with max gap already 0.07874 m. Averaging a fresh endpoint opened the sparse tail beyond the 0.08 m edge limit; `bound()` deleted the entire still-connected outgoing leg. A new sparse-tail test reproduced this deletion (`log_lab/d0809-r2-sparse-tail-red.log`). Bounding now retains a contiguous observed run (prefer nearby geometry, largest run on ties) and discards the separated tail. It never bridges an >8 cm gap, and the original edge-length tests remain unchanged. No expired-geometry cache or proximity-created connection was added. Iteration 05 follows a corner rerun.

After retaining the contiguous observed run, all 22 mapping tests passed (`log_lab/d0809-r2-sparse-tail-green.log`), the corner section passed arc 6.139178 m / 24.742 s / lateral p95/max 0.023756/0.030039 m, and the complete test command passed **168 tests, 0 errors, 0 failures, 0 skipped** (`log_lab/d0809-r2-test-iter05.log`). Iteration 05 is the next full-track gate for the corrected continuity handling.

Round 2 full iteration 05 (keep verified contiguous near run when sparse stale tail opens a gap): progress 4.613357 m (29.387%), PASS=False, 149.855 s, lateral p95/max 0.028878/0.035564 m; stop true XY/yaw [1.6686468433803738, 1.0628766539379617, 2.2650456954638067]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-176676`; plots/JSON `docs/d0809/round2_iter05/`; log `log_lab/d0809-r2-iter05.log`.

Iteration 05 again stopped near 4.613 m. Connection history showed segment 29 repeatedly gaining then losing links to still-present outgoing fragments 30/37/41/44. A new test confirmed that a re-observed feature extending through a known junction moved that junction into the segment interior, after which endpoint-only link validation erased it (`log_lab/d0809-r2-pinned-junction-red.log`). Fusion now pins already observed junction endpoints and clips feature extension across that boundary; the connected leg owns geometry beyond it. Fresh explicit joint observations still update the shared anchor together. No proximity-only link or unobserved connector is introduced. Iteration 06 follows the unit and corner checks.

Round 2 full iteration 06 (preserve observed junction endpoint while split boundary changes): progress 7.205590 m (45.900%), PASS=False, 224.080 s, lateral p95/max 0.023497/0.041365 m; stop true XY/yaw [0.33761342237926195, 1.0664773832885883, 2.123714801005291]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-178304`; plots/JSON `docs/d0809/round2_iter06/`; log `log_lab/d0809-r2-iter06.log`.

Iteration 06 reached 7.205590 m (45.900%), then stopped on the left-edge bend with no visible line in the lower camera. The remaining memory was near the 20 s age cap (uncertainty ≈0.045 m, still below the unchanged 0.06 m cutoff); its outgoing part disappeared before the slow turn restored vision. A new fast-sim-clock planner launch test failed because valid observed memory was gone at a 25 s slow approach (`log_lab/d0809-r2-slow-memory-red.log`). Competition retention is now 30 s; confidence/uncertainty still grow, ≥0.06 m still stops, source expiry is still strict, and no sensor timeout/braking/safety assertion changed. The same test requires valid MEMORY_UPDATED points at 25 s and no driveable geometry after finite expiry. Iteration 07 follows.

Round 2 full iteration 07 (30 s observed retention for slow blind-curve approach; unchanged uncertainty cutoff): progress 4.598877 m (29.295%), PASS=False, 166.823 s, lateral p95/max 0.023380/0.035070 m; stop true XY/yaw [1.6715207669829644, 1.0501210870046487, 2.2802944883766156]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-179463`; plots/JSON `docs/d0809/round2_iter07/`; log `log_lab/d0809-r2-iter07.log`.

Iteration 07 stopped near 4.598877 m despite the retention lifecycle test passing; retention alone does not solve curve continuity. The new nominal 0.11 m-radius arc test already PASSED before the change (`log_lab/d0809-r2-smooth-arc-red.log` is a misleading filename, retained honestly); it does not prove the stall cause. A provisional short-window condition was added for the next experiment, but needs a genuinely failing classifier case before being retained. Corner rotation now requires an abrupt >1.05 rad turn at the local observed vertex as well as in the broad window; the right-angle stop/rotate test remains. This removes false corner clamps on smooth curves while preserving the existing curvature speed cap. Iteration 08 first reruns both corner and curve sections.

Round 2 full iteration 08 (separate smooth tight arcs from abrupt observed vertices): progress 4.611946 m (29.378%), PASS=False, 167.714 s, lateral p95/max 0.023077/0.047206 m; stop true XY/yaw [1.6666638006410601, 1.061246066840257, 2.242279507025206]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-181420`; plots/JSON `docs/d0809/round2_iter08/`; log `log_lab/d0809-r2-iter08.log`.

Iteration 08's provisional classifier change did not fix the 4.61 m stall and increased curve-section error; it is reverted. The nominal 0.11 m arc regression remains and was never a failing case. The next failing test (`log_lab/d0809-r2-look-turn-red.log`) introduces a bounded active-perception action: for a short, still-valid curved memory prefix near the robot and no visible lower stroke, stop translation and rotate toward its actually observed forward tangent to restore view. Sharp corners, UNKNOWN/uncertain points, straight fragments and remote geometry cannot request this action. The 8 s controller rotation budget remains, and no geometry or translation beyond the known prefix is created. Iteration 09 reruns corner/curve then full track.

Stopped-look-turn unit tests passed, and both corner and curve section checks passed. Curve section reached arc 2.993504 m in 35.925 s, lateral p95/max 0.012439/0.014487 m. Completed logs are normalized to `log_lab/d0809-r2-fix9-{corner,curve}.log`; the iteration-09 wrapper accidentally used a `d0909` output prefix, which affects filenames only and will be normalized after completion. No check assertion or ROS launch command changed.

Round 2 full iteration 09 (stopped look turn): progress 4.319497 m (27.515%), PASS=False, 150.063 s, lateral p95/max 0.032473/0.052922 m; stop true XY/yaw [1.920980825424195, 0.9970199721269065, -3.1361207833363673]; reason stalled: unresolved re-observation mismatch; bag `/ws/bags/drive/full_1-182872`; plots/JSON `docs/d0809/round2_iter09/`; log `log_lab/d0809-r2-iter09.log`. The wrapper filename-prefix error only broke its post-run archive; actual check exited 1, and the complete archive was recovered manually from the observer output.

Iteration 09 stopped on unresolved re-observation mismatch, despite valid image tracking being a permitted fallback. New failing tests (`log_lab/d0809-r2-look-session-red.log`) cover fallback from rejected metric memory, one latched bounded look action with measured-progress hysteresis, and a verified near-field memory prefix shorter than the initial-detection length threshold. Image-only steering can now take over when metric memory is explicitly rejected; it never reuses that rejected geometry for translation. A near memory prefix may drive only beyond the unchanged 5.5 cm braking reserve, with strict source/uncertainty/expiry checks; a distant short fragment remains infeasible. Look rotation targets are latched through alignment, limited to six planner seconds / eight controller seconds, and do not retrigger at the same position except one genuinely changed target. Unit and section gates precede iteration 10.

<!-- R2_ITERATIONS_START -->

| Iteration | Change | Progress m / % | Stop true XY | Result / reason |
|---|---|---|---|---|
| 01 | rotation recovery, strict observed length, curved pixel trace | 4.586 / 29.21% | (1.682, 1.044) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 02 | near-field spatial eviction, signed progress | 1.679 / 10.69% | (2.010, 0.330) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 03 | perspective bottom-cap spur handling | 5.718 / 36.42% | (1.088, 1.646) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 04 | exclude shared junction from duplicate overlap | 3.173 / 20.21% | (2.963, 0.912) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 05 | retain contiguous run when sparse tail opens gap | 4.613 / 29.39% | (1.669, 1.063) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 06 | preserve known junction endpoints | 7.206 / 45.90% | (0.338, 1.066) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 07 | 30 s retention with unchanged uncertainty cutoff | 4.599 / 29.29% | (1.672, 1.050) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 08 | classifier experiment (reverted after no improvement) | 4.612 / 29.38% | (1.667, 1.061) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 09 | bounded stopped-look-turn prototype | 4.319 / 27.52% | (1.921, 0.997) | stalled: unresolved re-observation mismatch |
| 10 | latched look recovery, verified short memory and image fallback on metric rejection | 3.191 / 20.33% | (2.978, 0.931) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 11 | remove tiny same-segment backtracking spurs that falsely trigger corner rotation | 4.473 / 28.50% | (1.768, 1.001) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 12 | verified preview prefix and local control-horizon uncertainty | 8.400 / 53.50% | (0.332, 2.258) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 13 | group overlapping outgoing-track aliases while retaining genuine fork ambiguity | 4.606 / 29.34% | (1.669, 1.057) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 14 | use coherent two-point memory solely for bounded stopped camera orientation | 10.202 / 64.98% | (2.065, 2.409) | off line >.08m for .6s at 10.206m |
| 15 | correct premature turn lock with newly observed straight crossing exit; reject invalid roots | 10.694 / 68.12% | (2.555, 2.325) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 18 | distinguish curved bottom continuation from a second outgoing image branch | 6.916 / 44.06% | (0.592, 1.002) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 20 | retain ordered previously selected connected route through graph fragmentation | 4.324 / 27.54% | (1.917, 0.994) | stalled: image line lost/ambiguous or image tracking budget exhausted |
| 21 | align selected-memory direct-observation expiry with configured capture-based retention | 11.554 / 73.60% | (2.840, 1.576) | stalled: image line lost/ambiguous or image tracking budget exhausted |

<!-- R2_ITERATIONS_END -->

Round 2 full iteration 10 (latched look recovery, verified short memory and image fallback on metric rejection): progress 3.191465 m (20.330%), PASS=False, 117.213 s, lateral p95/max 0.020802/0.037222 m; stop true XY/yaw [2.978267836636081, 0.9308694886171917, 1.5696437071588354]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-184291`; plots/JSON `docs/d0809/round2_iter10/`; log `log_lab/d0809-r2-iter10-drive.log`.

Iteration 10 stopped at the first corner (3.191465 m, 20.330%). Bag inspection identified its first rotation as the legacy corner detector, not a look action: a <3 cm backtracking spur inside memory segment 2 produced a false pivot while a 0.484 m verified curve remained. The new trajectory test reproduces that exact local shape and failed (`log_lab/d0809-r2-backtrack-red.log`). Planning now removes only tiny >120° backtracking excursions whose two edges are <3 cm, whose replacement edge is <4 cm, and whose points belong to the same already connected segment. It keeps actual point coordinates, rejects UNKNOWN shortcuts, and leaves 90° corners / cross-segment junctions untouched. No unobserved branch or track coordinate is introduced. Iteration 11 follows unit, corner and curve gates.

Round 2 full iteration 11 (remove tiny same-segment backtracking spurs that falsely trigger corner rotation): progress 4.473323 m (28.495%), PASS=False, 152.117 s, lateral p95/max 0.011496/0.028851 m; stop true XY/yaw [1.7680361452966584, 1.001495977723525, 2.974076870377888]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-186278`; plots/JSON `docs/d0809/round2_iter11/`; log `log_lab/d0809-r2-iter11-drive.log`.

Iteration 11 improved the curve section to 20.019 s / lateral p95/max 0.015010/0.015785 m, but the full run stopped at 4.473323 m. Preview policy was evaluating the entire unfiltered stored chain, while publishing used a verified prefix: an unusable distant point could stop valid near geometry or throttle it until memory aged out. The new test (`log_lab/d0809-r2-control-prefix-red.log`) requires near control uncertainty to stay local, while a far ≥0.06 m point still truncates the verified path and a near uncertain point still throttles/stops. Preview now applies the same source/expiry/uncertainty filter as publishing. Speed uncertainty uses the next 0.30 m of verified arc, longer than the 0.12 m target plus 0.055 m reserve and 0.1 m/s braking horizon; every published point still passes the unchanged 0.06 m and expiry gates. Debug-only fallback logs now include actual prefix lengths and uncertainty. Iteration 12 follows.

Round 2 full iteration 12 (verified preview prefix and local control-horizon uncertainty): progress 8.399508 m (53.505%), PASS=False, 304.982 s, lateral p95/max 0.023475/0.046316 m; stop true XY/yaw [0.3320759666153052, 2.2584766972047334, 1.5491304769861478]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-187693`; plots/JSON `docs/d0809/round2_iter12/`; log `log_lab/d0809-r2-iter12-drive.log`.

Iteration 12 reached a new best 8.399508 m (53.505%) with max lateral error 0.046316 m, then correctly stopped with only a 0.048 m verified prefix. Its graph contained three near-identical right-turn copies (tracks 2523/2524/2526), all linked to incoming 2522; the evaluator incorrectly treated them as three ambiguous branches. New tests (`log_lab/d0809-r2-duplicate-legs-red.log`) reproduce that false ambiguity and separately preserve stopping for a genuinely diverging shallow fork. Candidate grouping now requires headings within 0.20 rad and ≥80% sampled overlap of the shorter observed leg within 0.015 m. The ambiguity score margin is unchanged. All raw candidates/scores remain visible; aliases are marked as duplicate evidence, one representative is selected using uncertainty and existing lock stability, and real divergent geometry remains ambiguous.

The intermediate test run reported 178 tests / 2 failures representing the same GCS launch case in two result files. It published a volatile one-shot request before broker discovery was required. The test now waits for matched request subscription and event publication within its existing readiness timeout; KILL acceptance, E_STOP→STOPPED and command deadlines are unchanged. The targeted launch test passed (`log_lab/d0809-r2-gcs-readiness-green.log`). Iteration 13 follows.

Round 2 full iteration 13 (group overlapping outgoing-track aliases while retaining genuine fork ambiguity): progress 4.605775 m (29.339%), PASS=False, 164.443 s, lateral p95/max 0.021237/0.039702 m; stop true XY/yaw [1.6693040482913528, 1.057384231352481, 2.2519271076765404]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-189740`; plots/JSON `docs/d0809/round2_iter13/`; log `log_lab/d0809-r2-iter13-drive.log`.

Iteration 13 stopped at 4.605775 m after the outgoing-alias tests and both sections passed; all 180 tests were green. Debug logs showed the actual remaining trusted fragment had only two same-segment points / 0.028 m, so it could not support translation or the old ≥3-point tangent helper. A new failing test (`log_lab/d0809-r2-short-look-red.log`) covers using two coherent, still-valid same-segment points spanning ≥0.020 m solely for a bounded orientation action; UNKNOWN or different-segment pairs remain rejected. Linear feasibility and the 0.055 m reserve do not change. The aim is to restore camera visibility before this final orientation cue expires. Iteration 14 follows.

Round 2 full iteration 14 (use coherent two-point memory solely for bounded stopped camera orientation): progress 10.201633 m (64.984%), PASS=False, 324.421 s, lateral p95/max 0.025230/0.098630 m; stop true XY/yaw [2.0649095309441354, 2.4086299811736085, 1.3991964954188725]; reason off line >.08m for .6s at 10.206m; bag `/ws/bags/drive/full_1-191623`; plots/JSON `docs/d0809/round2_iter14/`; log `log_lab/d0809-r2-iter14-drive.log`.

Round 2 crossing diagnosis after iteration 14: bag `/ws/bags/drive/full_1-191623` shows incoming track 2041 holding turn 2443 (heading +1.55 rad), even after newly observed straight track 2622 had heading -0.015 rad and score -0.031 versus the held turn -1.57. The turn was provisionally locked before the fourth leg appeared; this was not an image-follower turn. Added `NewlyObservedStraightLegCorrectsPrematureTurnLock` (red: wrong final x/y and no selected straight; `log_lab/d0809-r2-cross-red.log`). A newly observed clearly straight leg now corrects a held turn while the junction is still >0.12 m ahead; an already selected straight leg remains locked. Added eastbound crossing section (mesh-derived test pose only) because the existing crossing section covered the later northbound traversal.

Root validity tests written before iteration 14 exposed three failures: uncertain near roots, a short unusable prefix, and expired near roots masking fresh usable forward geometry (`log_lab/d0809-r2-valid-root-red.log`). Root search now checks source/finite coordinates/uncertainty/expiry at current time; an unusably short near prefix may yield only to an aligned continuation within 4 cm of its tangent, never an unrelated parallel line (new negative test). An initial compile/run revealed a ROS-versus-system Time comparison exception; comparing nanosecond values corrected that test-helper clock mismatch. Green results pending the next run.

Round 2 iteration 15 preflight: build exit 0; all navigation unit suites pass (mapping 30 tests); full `tools/lab_env.sh test` reports **186 tests, 0 errors, 0 failures, 0 skipped** (`log_lab/d0809-r2-test-iter15.log`). Corner and curve pass. New eastbound crossing passes, ordered arc 10.684947 m, 21.311 s, lateral p95/max 0.00002240/0.00002614 m, straight-through reason, bag `/ws/bags/drive/crossing_east-193546` (`log_lab/d0809-r2-iter15-crossing_east.log`). Full run follows.

Round 2 full iteration 15 (correct premature turn lock with newly observed straight crossing exit; reject invalid roots): progress 10.693689 m (68.119%), PASS=False, 339.848 s, lateral p95/max 0.032476/0.083634 m; stop true XY/yaw [2.555402764855454, 2.3249933302036023, 0.029561414348293683]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-194073`; plots/JSON `docs/d0809/round2_iter15/`; log `log_lab/d0809-r2-iter15-drive.log`.

Iteration 15's crossing correction succeeded in the full run, but the next blind right curve stalled at 68.119%. The final camera contains no visible stroke. Bag inspection shows multiple small tracks and lost endpoint connections; the static/noisy synthetic repeated-curve experiment **already passed** and does not prove that merging itself is the cause (`log_lab/d0809-r2-curve-merge-red.log`, despite that provisional filename). Extending the eastbound crossing section through the following bend reproduces a stall at 10.833317 m / 69.008%, true pose (2.692333, 2.300254, -0.878591), lateral p95/max 0.003144/0.003918 m (`log_lab/d0809-r2-cross-curve-red.log`, bag `bags/drive/crossing_east-194475`).

A concrete root-search failure is now isolated: the old 8 cm isolated-fragment prefilter discarded even the two-point trusted memory cue required by the stopped-look recovery. `ShortVerifiedNearCurveRemainsAvailableForStoppedOrientation` fails with an empty result (`log_lab/d0809-r2-short-root-red.log`). Remove that prefilter; the verified-length/braking gate still prohibits translating on a ≤5.5 cm path, while the bounded look-turn helper can use its actual observed tangent. The aligned-root preference still prevents a tiny fragment masking a longer compatible path. Candidate messages now also set their existing `selection_locked` field when a branch is selected.

Iteration 16: build and all navigation unit suites pass; corner and curve sections pass. The extended eastbound crossing section advances through the blind bend but stalls at 11.055451 m (70.423%), true pose [2.773357928279615, 2.097334144524848, -1.5817663877783217], reason `stalled: no feasible candidate`, lateral p95/max 0.023386/0.023454 m. Camera tracking was valid at the stop. The wrapper correctly stops before a full run because this section failed; artifacts are `docs/d0809/round2_iter16_section/`, bag `/ws/bags/drive/crossing_east-195961`.

The first aging-based image-fallback launch test already passed the old code and was not a reproducer. A deterministic version now sets the test-only metric minimum to 0.20 m and presents a valid 0.15 m observed prefix, exposing the mismatch between the preliminary validity gate and final planner feasibility. Disabling only the new late-fallback branch produces the expected failure: no image path despite clear pixels (`log_lab/d0809-r2-final-metric-red.log`). The fix shares the existing visibility/time/distance-limited image publisher between preliminary and final metric rejection. Its budget resets only after an actually valid accepted metric plan, not preliminary acceptance. The test also requires a zero path when image visibility is withdrawn. Green run and extended section follow.

The deterministic metric-rejection launch test required proper isolation: reset to a fresh mission and monotonically increasing stamps (101 s onward), so the preceding test's exhausted image episode cannot mask the result. The initial clock/reset versions were inconclusive. Final test uses a 0.18 m observed prefix and a test-only 0.20 m planner minimum: with late fallback disabled it fails with no image path (`log_lab/d0809-r2-final-metric-red3.log`); enabled, both launch tests pass in 11.749 s (`log_lab/d0809-r2-final-metric-green3.log`), including zero speed after image visibility is withdrawn. Competition minimum remains 0.10 m. Iteration 17 follows.

Iteration 17: build/core unit/corner/curve pass; `tools/lab_env.sh test` reports **189 tests, 0 errors, 0 failures, 0 skipped** (`log_lab/d0809-r2-test-iter17.log`). The extended eastbound section reaches 11.107852 m / 70.757%, then stops with actual image tracking invalid; true pose [2.7412455758315897, 2.045934194736325, -1.4011305464271229]; p95/max lateral 0.012639/0.017340 m, bag `/ws/bags/drive/crossing_east-200015`. No full run started because the extended section failed. Artifacts: `docs/d0809/round2_iter17_section/`. A captured frame from this run is now a perception regression fixture (observed pixels only, no track coordinates); diagnostic investigation of image-component continuity follows.

Image root diagnostics identified one bottom-connected component but two apparent outgoing endpoints: depths 148 and 30 skeleton steps (`log_lab/d0809-r2-loop-red.log`). The 30-step endpoint is actually the curved incoming continuation below the selected root. A synthetic strongly curved bottom continuation reproduces the same error (depths 126 and 29, `log_lab/d0809-r2-root-pixel-red.log`). The first lowest-pixel-root attempt fixed those frames but broke the broad-trapezoid cap regression and was reverted. Shipped approach retains root selection and identifies a bottom-clipped incoming continuation only when its entire trace remains below the root. It additionally requires exactly one contiguous stroke at the clipped bottom: connected Y junctions with two bottom strokes still reject. No global branch/prefix acceptance or ROI widening was shipped. All **31 perception tests pass**, including the captured frame, curved synthetic, connected-Y rejection, disjoint strokes, transverse bar, broad cap and existing geometry tests (`log_lab/d0809-r2-root-pixel-green2.log`). Iteration 18 follows.

Iteration 18 preflight: build, navigation unit suites, corner and curve pass. Extended eastbound crossing plus blind following bend now **PASS**, ordered progress 11.333024 m, 50.440 s, lateral p95/max 0.019114/0.020442 m, bag `/ws/bags/drive/crossing_east-202186` (`log_lab/d0809-r2-iter18-crossing_east.log`). Full real-competition run follows.

Round 2 full iteration 18 (distinguish curved bottom continuation from a second outgoing image branch): progress 6.916024 m (44.055%), PASS=False, 222.266 s, lateral p95/max 0.028086/0.031267 m; stop true XY/yaw [0.5920935861248534, 1.0020711963761724, -3.00107973350241]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-202454`; plots/JSON `docs/d0809/round2_iter18/`; log `log_lab/d0809-r2-iter18-drive.log`.

Iteration 18 full stops at 6.916024 m / 44.055%, true pose (0.592094, 1.002071, -3.001080), lateral p95/max 0.028086/0.031267 m, bag `bags/drive/full_1-202454`; no finish. At initial loss the chosen near path has one point, while a separate fresh curve fragment remains at base (0.258, -0.085) → (0.263, -0.117), uncertainty 0.0112 m. It cannot support a connected drive path and is not used as one. Added unit tests fail before `look_to_base` exists (`log_lab/d0809-r2-future-look-red.log`); all 34 mapping tests pass after it is added (`log_lab/d0809-r2-future-look-green.log`). A stopped camera look may use a single coherent future observed fragment within 0.35 m, stricter uncertainty <0.04 m, valid capture expiry, ≥2 points / 0.025 m observed chord; conflicting spatial/directional evidence rejects. It reports zero verified drive distance and zero translation, limits the turn to 1.1 rad toward its actual observed tangent, and retains the existing 6 s look / 8 s controller budgets. UNKNOWN/expired/uncertain/remote/distinct-leg tests reject. New optional `left_bend` section starts at mesh arc 6.55 m and traverses to 7.75 m; these oracle arc positions exist only in the dev harness. Iteration 19 follows.

Iteration 19: 194 tests pass; build/core unit/corner/curve pass, but extended eastbound crossing fails **off line** at 11.310852 m / 72.050%, true pose [2.6430243136612113, 1.827364096251958, -2.6871927437522274], p95/max lateral 0.023724/0.106976 m (`log_lab/d0809-r2-iter19-crossing_east.log`, bag `/ws/bags/drive/crossing_east-204448`, artifacts `docs/d0809/round2_iter19_section/`). No full run starts. The disconnected-fragment look experiment is **reverted**, including its unused API/tests: a nearby later loop leg is actual observed geometry but does not prove the connected route from the current prefix. Turning toward it allowed an image-space shortcut. Preserve the unchanged off-line gate; do not ship this approach. The next investigation concerns retaining the order of a previously selected, actually observed connected route through subsequent map fragmentation.

Selected-route investigation: bag 18 contains a coherent, already selected connected route with the actual left-bend tail through sim t=205 s; the graph discards that tail at t=206 even though some of its newer-capture evidence remains valid. The final near graph fragment has a newer capture than the original route prefix. Preserve the selected route's ordered vertices separately from graph track IDs; do not select a disconnected replacement. Added `SelectedPathMemory` (max 256 points / 1.5 m), storing only verified selected connected polylines in odom. Short compatible updates cannot erase its tail; only overlap-proven extensions append, while an actually selected contradictory route replaces it. Capture expiry and uncertainty still apply point by point; an interior invalidity truncates without joining a fresh tail. Newer known evidence may refresh metadata at the same coordinates with spatial-offset uncertainty; the same capture cannot renew itself. Monotonic progress in a short arc window prevents switching to nearby return legs. It resets with mission/fault memory, contributes only while current branch observations are unambiguous and geometrically compatible, and remains subject to the existing policy/controller/safety gates. Source UNKNOWN and gaps >8 cm reject.

Tests first fail for the missing selected-route class and refresh API (`log_lab/d0809-r2-selected-route-red.log`, `log_lab/d0809-r2-selected-refresh-red.log`); all 38 mapping/selected-route tests pass (`log_lab/d0809-r2-selected-route-green2.log`). Cases cover retained corner, current-base transform, contradiction, expiry, UNKNOWN/gap rejection, uncertainty growth, coordinate-preserving re-observation, no self-renewal, interior expiry, bounded storage and no jump to a nearby return leg. Current captures still use the original pose interpolation and source contract. Iteration 20 tests the left bend first, then corner/curve/extended east crossing and full launch.

Iteration 20 passes left-bend, corner and curve preflights. A follow-up source correction aligns direct OBSERVED points retained in the selected-route buffer with the existing `memory_retention_sec` storage lifetime (capture + retention), rather than the raw observed-graph's transient 0.5 s publication validity. Candidate/path watchdog validity is unchanged; MEMORY_UPDATED expiry is preserved. **This source correction was made after iteration 20's build and is not in its running binary; it must be rebuilt and exercised in subsequent/final gates.** No geometry or expiry beyond the existing configured capture-based memory lifetime is added.

Round 2 full iteration 20 (retain ordered previously selected connected route through graph fragmentation): progress 4.323697 m (27.542%), PASS=False, 140.855 s, lateral p95/max 0.015368/0.034115 m; stop true XY/yaw [1.91738133636421, 0.9943804935587685, 3.138172635447072]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-207156`; plots/JSON `docs/d0809/round2_iter20/`; log `log_lab/d0809-r2-iter20-drive.log`.

Iteration 20 full (before the direct-observation retention-parity correction) stalls at 4.323697 m / 27.542%, true pose (1.917381, 0.994380, 3.138173), lateral p95/max 0.015368/0.034115 m, bag `bags/drive/full_1-207156`. All 198 tests were green (`log_lab/d0809-r2-test-iter20.log`). The follow-up branch replacement regression also passes: a newly selected straight continuation replaces a provisional cached turn without retaining the turn's tail (`log_lab/d0809-r2-selected-branch-green.log`). Next build applies capture + configured retention to direct observations stored as selected memory; their outgoing command/path watchdog remains unchanged. Iteration 21 follows with all focused sections and a full competition.

Iteration 21 focused section results after applying capture-based retention parity:

| Section | PASS | End arc m | Wall s | Lateral p95 / max m | Bag |
|---|---|---|---|---|---|
| left_bend | True | 7.753925 | 25.234 | 0.015398 / 0.016775 | `/ws/bags/drive/left_bend-208527` |
| corner | True | 6.138824 | 35.437 | 0.022900 / 0.030175 | `/ws/bags/drive/corner-208795` |
| curve | True | 2.992694 | 37.607 | 0.012853 / 0.014989 | `/ws/bags/drive/curve-209063` |
| crossing_east | True | 11.334082 | 52.191 | 0.026569 / 0.026594 | `/ws/bags/drive/crossing_east-209331` |

Section PNG/JSON artifacts are preserved in `docs/d0809/round2_iter21_sections/`; full launch is running.

Iteration 21 full test suite: `tools/lab_env.sh test` → **199 tests, 0 errors, 0 failures, 0 skipped** (`log_lab/d0809-r2-test-iter21.log`). Full-track run is still pending; no completion claim yet.

Round 2 full iteration 21 (align selected-memory direct-observation expiry with configured capture-based retention): progress 11.554129 m (73.600%), PASS=False, 414.719 s, lateral p95/max 0.075571/0.075951 m; stop true XY/yaw [2.840230563872457, 1.5755370385127911, -0.265282376486903]; reason stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/full_1-209599`; plots/JSON `docs/d0809/round2_iter21/`; log `log_lab/d0809-r2-iter21-drive.log`.

Iteration 21 new best: 11.554129 m / **73.600%**, true pose (2.840231, 1.575537, -0.265282), 414.719 s, lateral p95/max 0.075571/0.075951 m, stalled with no visible image line (`log_lab/d0809-r2-iter21-drive.log`, bag `bags/drive/full_1-209599`, `docs/d0809/round2_iter21/`). The loop's current-path sample at sim 383 contains the full observed loop; at sim 387 a partial path cuts from incoming to return side and extends a new tail. The saved-route preference rejects that candidate only when its entire geometry fits the saved path, so the added tail bypasses the preference. This is a route-ordering issue, not grounds to increase the off-line tolerance. Raw wheel-odom yaw differs from gyro heading by ~0.107 rad at the stop; translation integration is unchanged.

New regression fails because a partial shortcut with a new tail is preferred over a previously observed connected loop (`log_lab/d0809-r2-loop-order-red.log`). The fix projects candidates onto the saved ordered arc: an arc jump >0.12 m and >2.5× its spatial chord +0.05 m, or a >0.12 m reversal, retains the still-valid selected route. Expired/uncertain prefixes are truncated before enforcing this lock; genuinely new straight selections still replace provisional turns, and fresh geometry can replace an expired route. All **40 mapping/selected-route tests pass** (`log_lab/d0809-r2-loop-order-green.log`). No disconnected-fragment look is restored. Added optional `lower_loop` section at mesh arc 10.9→12.55 m, tested first in iteration 22 before other sections/full launch.

Iteration 22: build and core units pass; focused lower-loop section reaches 11.605369 m (73.926%), true pose [2.896900856948031, 1.6388186975488226, -0.7316732941751428], p95/max lateral 0.034388/0.037459 m. It follows the correct loop entry rather than the prior shortcut, but stalls before completing it (`stalled: image line lost/ambiguous or image tracking budget exhausted`), so other sections/full do not start. Bag `/ws/bags/drive/lower_loop-211096`, artifacts `docs/d0809/round2_iter22_section/`, log `log_lab/d0809-r2-iter22-lower_loop.log`.

Iteration 23 changes: far-only/peripheral positive observations no longer count as current near-line support (same 0.45 m / ±0.20 m corridor already used by root search); they can still be mapped without claiming VISION recovery. A new test fails at valid uncertainty 0.055 m because the old speed is only 0.008333 m/s (`log_lab/d0809-r2-memory-authority-red2.log`). The uncertainty factor now saturates at 0.5 while strictly below the unchanged 0.06 m cutoff; existing SLOWDOWN cap remains 0.04 m/s, low-uncertainty speeds are unchanged, braking-distance limits remain, and expired/uncertain/stale inputs still stop. Existing monotone-speed and positive stopping-margin tests pass unchanged (`log_lab/d0809-r2-memory-authority-green.log`). Rotation/time/distance uncertainty growth and the 30 s capture lifetime are **unchanged**. Candidate metadata identifies the held connected route when it overrides a partial current candidate, rather than selecting that deferred candidate in the UI. Lower-loop test runs first again.

Iteration 23 lower-loop result: 11.549832 m / 73.572%, true pose [2.8427441298341107, 1.6221842265205646, -1.687067013102845], p95/max lateral 0.037151/0.052053 m; failed with `stalled: image line lost/ambiguous or image tracking budget exhausted`, bag `/ws/bags/drive/lower_loop-212079`, artifacts `docs/d0809/round2_iter23_section/`. It does not improve the focused gate and increases maximum lateral error versus iteration 22. **Reverted both the uncertainty-factor floor and far-only support/data-accept experiment, including their dedicated experimental tests.** The original linear uncertainty speed envelope, rotation/time/distance uncertainty growth, 0.06 m cutoff, 30 s capture retention, braking reserve and safety guard limits remain. Retained the tested selected-route ordering protection and truthful held-candidate metadata (an infeasible held route is not marked selected). Best full remains iteration 21, 73.600%. Mandatory frozen regressions and a clean-output default drive gate (four required sections + two full competitions) follow; no finish is claimed.

Round 2 frozen regression commands (full log paths below):

| Command | Exit | Log |
|---|---|---|
| `tools/lab_env.sh build` | 0 | `log_lab/d0809-r2-final-build.log` |
| `tools/lab_env.sh test` | 0 | `log_lab/d0809-r2-final-test.log` |
| `tools/lab_env.sh check-d07` | 0 | `log_lab/d0809-r2-final-check-d07.log` |
| `tools/lab_env.sh check-d06` | 0 | `log_lab/d0809-r2-final-check-d06.log` |
| `tools/lab_env.sh check-d05` | 0 | `log_lab/d0809-r2-final-check-d05.log` |
| `tools/lab_env.sh checkpoint-a` | 0 | `log_lab/d0809-r2-final-checkpoint-a.log` |
| `tools/lab_env.sh check-gcs` | 0 | `log_lab/d0809-r2-final-check-gcs.log` |
| `tools/lab_env.sh check-bag` | 0 | `log_lab/d0809-r2-final-check-bag.log` |
| `tools/lab_env.sh check-camera-rate` | 0 | `log_lab/d0809-r2-final-check-camera-rate.log` |
| `tools/lab_env.sh check-teardown` | 0 | `log_lab/d0809-r2-final-check-teardown.log` |
| `docker build -t line-follower-d0809-r2:local .` | 0 | `log_lab/d0809-r2-final-docker.log` |

Frozen Round 2 regression key evidence (all commands exit 0; see the preceding exact-command/log table):

- `build`: 8 packages; `test`: **200 tests, 0 errors, 0 failures, 0 skipped**.
- D-05: all six flat poses pass; recall 1.0 each; corner p95/max 0.007724/0.008000 m. Hill remains nongated and projection-invalid.
- D-06: straight/corner/rotation pass; rotation spread 0.002166 m, localization XY error 0, heading max 0.000290 rad, snapshot pose max 0.001999 rad; UNKNOWN=0.
- D-07: heading/slope/blind-end gates pass; 322 slope frames, 0 confirmed; blind memory p95 0.004725 m, UNKNOWN/uncertain/expired confirmed points 0; STOPPING on exhausted verified distance.
- Checkpoint A: PASS; controller-loss command zero 0.197 s within 0.300 s and physical stop 0.256 s; guard exit last command exactly zero and physical stop 0.039 s; pre-RUNNING nonzero commands 0, displacement 0.
- GCS: **26/26**; snapshot 2.000 Hz, link LOST in 2.019 s, all rover nodes alive; live UI 36 snapshots / 76 camera frames over 19.76 s.
- Bag/replay: `/cmd_vel` publishers/messages **0/0**; `/replay/cmd_vel_blocked` 611 (454 nonzero); regenerated perception line/status 549 each, path/local and path/status 602 each.
- Camera: composed reliable recording **29.601 Hz**, 684 frames / 23.074 s; external C++ observer 22.249 Hz, showing the previously documented observer discrepancy.
- Teardown: **10/10** clean; production Docker: six packages, Release, BUILD_TESTING=OFF, success.

The default drive gate starts with `docs/d0809`, `log_lab/d0809`, and `bags/drive` absent; all three absence flags are true in `log_lab/d0809-r2-final-clean-inputs.json`. Original evidence is preserved and will be restored after the command. All four required sections have passed; two real competitions are in progress. No full-track acceptance is claimed yet.

Round 2 default drive gate from absent output directories: `tools/lab_env.sh check-drive`, exit 1. Previous evidence was moved to `log_lab/d0809-r2-clean-inputs-20261121-043523` before the command; directory absence is recorded in `log_lab/d0809-r2-final-clean-inputs.json`. Final run artifacts are copied under `docs/d0809/round2_final_clean/`.

| Run | PASS | Progress m / % | Wall s | Lateral p95 / max m | Reason |
|---|---|---|---|---|---|
| hill | True | 1.902853 / 12.121 | 46.573 | 0.000754 / 0.000779 | section traversed |
| corner | True | 6.142597 / 39.128 | 37.101 | 0.025380 / 0.030670 | section traversed |
| curve | True | 2.992293 / 19.061 | 40.040 | 0.012131 / 0.014638 | section traversed |
| crossing | True | 13.672046 / 87.091 | 24.400 | 0.005664 / 0.018359 | section traversed |
| full_1 | False | 11.642818 / 74.165 | 478.235 | 0.036519 / 0.039455 | stalled: image line lost/ambiguous or image tracking budget exhausted |
| full_2 | False | 2.639246 / 16.812 | 95.245 | 0.027913 / 0.040146 | stalled: image line lost/ambiguous or image tracking budget exhausted |

### Final failure diagnosis and next blocker

Final full 1 stopped on the Ω loop at world `(2.907673, 1.595858)`, yaw `-1.109880`. Its last metric forward path had nine points / 0.112486 m, uncertainty **0.059958 m**, capture age **28.356 s**, and **1.644 s** remaining before capture expiry (`log_lab/d0809-r2-final-full1-memory.log`). It then loses usable memory at the unchanged 0.06 m uncertainty gate while `image_tracking_valid=false`. Final perception still reports a projected line, so the availability/association of that visible geometry with the selected loop needs investigation. The original linear uncertainty speed reduction and all hard limits remain; the unsuccessful speed-floor/near-support experiment was reverted.

Final full 2 stopped at the first curve, world `(2.932981, 0.384129)`, yaw `-2.544550`. Its last forward metric path had three points / 0.064416 m (uncertainty 0.047604–0.051421 m); the final stopped-look path had only two points / 0.037193 m, uncertainty up to **0.059773 m**, capture age **22.444 s**, with **7.556 s** before expiry (`log_lab/d0809-r2-final-full2-memory.log`). Final projection and image tracking are invalid. The ending heading roughly opposes the local track tangent; this supports investigating directed segment/corner-target association under partial curved observations, rather than assuming the rotation watchdog alone explains the stop. This is a diagnosis to verify, not a completed fix.

Next work: stabilize the directed observed graph/selected outgoing leg through partial curve/loop observations and confirm why fresh projected observations do not refresh the required near route. Then demonstrate the blind Ω traversal inside the existing uncertainty/expiry budget. Do not increase the off-line tolerance or bypass source, stopping, startup, KILL, candidate-loss or guard-exit checks. Raw wheel-odom/gyro heading disagreement (~0.107 rad in iteration 21) is a secondary measurement; translation integration was not changed or proven causal. Full completion and D-08/D-09 acceptance remain outstanding.

### Rerun commands and final artifacts

```bash
tools/lab_env.sh check-drive
tools/lab_env.sh check-drive --section lower_loop --limit 120
tools/lab_env.sh check-drive --section crossing_east --limit 120
tools/lab_env.sh check-drive --section left_bend --limit 120
```

The first command is the required four-section + two-full gate. The additional sections are diagnostic starts, not claimed full-track completions. Final PNGs/JSON: `docs/d0809/round2_final_clean/` (`hill`, `corner`, `curve`, `crossing`, `full_1`, `full_2`, each camera/top-down/timeline plus JSON). Full bags: `bags/drive/full_1-219833` and `bags/drive/full_2-219833`; event lists are in their JSON reports and mode timelines in the PNGs. Exact final gate output: `log_lab/d0809-r2-final-drive.log`; exit code: `log_lab/d0809-r2-final-drive-exit.txt`; absence proof: `log_lab/d0809-r2-final-clean-inputs.json`. Regression exits/logs: `log_lab/d0809-r2-final-status.json`.

### Round 2 final source/document files

The following files contain the final implementation/document changes; temporary disconnected-look and uncertainty-floor approaches were reverted. Generated visual evidence is grouped under `docs/d0809/`; scratch runners and raw diagnostics are under ignored `log_lab/` and are not required by any one-line check.

- `README.md`
- `docs/test_plan.md`
- `docs/d0809_evidence.md`
- `tools/check_drive.py`
- `src/line_follower_bringup/config/competition.yaml`
- `src/line_follower_control/include/line_follower_control/pure_pursuit.hpp`
- `src/line_follower_control/src/pure_pursuit.cpp`
- `src/line_follower_control/src/motion_controller_node.cpp`
- `src/line_follower_control/test/test_safety_policy.cpp`
- `src/line_follower_navigation/include/line_follower_navigation/memory_policy.hpp`
- `src/line_follower_navigation/include/line_follower_navigation/selected_path_memory.hpp`
- `src/line_follower_navigation/include/line_follower_navigation/trajectory_planner.hpp`
- `src/line_follower_navigation/src/local_path_memory.cpp`
- `src/line_follower_navigation/src/path_planner_node.cpp`
- `src/line_follower_navigation/src/trajectory_planner.cpp`
- `src/line_follower_navigation/test/test_mapping_memory.cpp`
- `src/line_follower_navigation/test/test_memory_policy.cpp`
- `src/line_follower_navigation/test/test_trajectory_planner.cpp`
- `src/line_follower_perception/CMakeLists.txt`
- `src/line_follower_perception/src/line_detector_backend.cpp`
- `src/line_follower_perception/test/test_line_detector_backend.cpp`
- `src/line_follower_perception/test/data/curved_stroke_gazebo.png`
- `src/line_follower_tests/CMakeLists.txt`
- `src/line_follower_tests/test/test_controller_rotate_launch.py`
- `src/line_follower_tests/test/test_slow_memory_launch.py`
- `src/line_follower_tests/test/test_gcs_launch.py`

Final cleanup: container process inspection contains only docker-init, its sleep process and the inspection command; no ROS/Gazebo/launch/GCS/recorder/player remains (`log_lab/d0809-r2-final-cleanup.log`). Preserved historical and final bag metadata files exist after restoration. Final outcome remains **FAILED**, with all non-driving regression gates green and both full-track attempts incomplete.

# Round 3

Active dispatch: reliability first, then Ω-loop/full-track extension; driving code freeze 2026-11-22 12:00 KST. Frozen Round 2 baseline: 200 tests and all non-drive gates pass; full results 74.1647% and 16.8120%, best 11.642818 m. Changes will be kept only with measured reliability/best-progress evidence; safety thresholds and provided assets remain unchanged.

Round 3 baseline first-curve 1, offset +0.000 m: exit 0, PASS=True, arc 2.994281 m, 21.776 s, pose [2.945996924754561, 0.7315245034949349, 1.5457075419269006], lateral p95/max 0.030970/0.033718 m; section traversed; bag `/ws/bags/drive/curve-221258`; artifacts `docs/d0809/round3_baseline_1/`.

Round 3 baseline first-curve 2, offset -0.030 m: exit 0, PASS=True, arc 2.964520 m, 27.110 s, pose [2.956880016033792, 0.7017630566695906, 1.619201095992041], lateral p95/max 0.018408/0.020026 m; section traversed; bag `/ws/bags/drive/curve-221549`; artifacts `docs/d0809/round3_baseline_2/`.

Round 3 baseline first-curve 3, offset +0.030 m: exit 0, PASS=True, arc 3.024354 m, 37.424 s, pose [2.966972560319544, 0.7615970493807853, 1.5805276638425294], lateral p95/max 0.013032/0.015074 m; section traversed; bag `/ws/bags/drive/curve-221871`; artifacts `docs/d0809/round3_baseline_3/`.

Round 3 baseline first-curve 4, offset -0.015 m: exit 0, PASS=True, arc 2.978961 m, 27.567 s, pose [2.9362331882363235, 0.7162038694356008, 1.5923802641637417], lateral p95/max 0.038051/0.042720 m; section traversed; bag `/ws/bags/drive/curve-222194`; artifacts `docs/d0809/round3_baseline_4/`.

Round 3 baseline first-curve 5, offset +0.015 m: exit 0, PASS=True, arc 3.005797 m, 37.664 s, pose [2.9643660896744595, 0.7430399082367286, 1.5759094106682054], lateral p95/max 0.014589/0.017643 m; section traversed; bag `/ws/bags/drive/curve-222578`; artifacts `docs/d0809/round3_baseline_5/`.

Round 3 fix 1: failed full_2 bag `bags/drive/full_2-219833` contains a near path at sim 87.5 s with points `(0.007,-0.027,id2) → (0.030,-0.017,id3) → (0.008,-0.032,id3) → (-0.027,-0.046,id3)`. It requests relative heading -2.6693 rad / absolute -2.5198 rad. The exact synthetic trace fails the new regression: old planner reports valid ROTATE_IN_PLACE; the selected-route test also fails to preserve the prior direction (`log_lab/r3-retrace-red.log`). New logic rejects >2.2 rad retraces in the near 0.20 m prefix and retains a still-valid selected route longer than 0.12 m over such a partial fold. It does not create a new outgoing direction; if no usable prior route exists it stops/reacquires. Right-angle and smooth-arc tests remain unchanged and pass (`log_lab/r3-retrace-green.log`).

Baseline first-curve starts passed **5/5** at offsets 0, -0.030, +0.030, -0.015, +0.015 m. Therefore the reversed-heading defect is reproduced by the recorded full-run trace, not by claiming baseline section failure. Post-fix five-offset repetition and three full competitions follow; no driving improvement is claimed until those results exist.

Round 3 fix1 first-curve 1, offset +0.000 m: exit 0, PASS=True, arc 2.990702 m, 36.838 s, pose [2.966618849328189, 0.7279455930317826, 1.5941746095042442], lateral p95/max 0.013612/0.016400 m; section traversed; bag `/ws/bags/drive/curve-223838`; artifacts `docs/d0809/round3_fix1_1/`.

Round 3 fix1 first-curve 2, offset -0.030 m: exit 1, PASS=False, arc 2.924534 m, 51.646 s, pose [2.939638232328142, 0.6642534217761825, 1.6287301359011863], lateral p95/max 0.032476/0.041302 m; stalled: near path retraces; outgoing direction is not established; bag `/ws/bags/drive/curve-224106`; artifacts `docs/d0809/round3_fix1_2/`.

Round 3 fix1 first-curve 3, offset +0.030 m: exit 1, PASS=False, arc 2.988321 m, 49.051 s, pose [2.959261151132925, 0.7284875020713683, 1.6129595429883101], lateral p95/max 0.011098/0.015499 m; stalled: image line lost/ambiguous or image tracking budget exhausted; bag `/ws/bags/drive/curve-224374`; artifacts `docs/d0809/round3_fix1_3/`.

Round 3 fix1 first-curve 4, offset -0.015 m: exit 0, PASS=True, arc 2.975693 m, 35.617 s, pose [2.9633855596634056, 0.7129362624971544, 1.5977793839269514], lateral p95/max 0.015827/0.017907 m; section traversed; bag `/ws/bags/drive/curve-224642`; artifacts `docs/d0809/round3_fix1_4/`.

Round 3 fix1 first-curve 5, offset +0.015 m: exit 0, PASS=True, arc 3.009469 m, 37.606 s, pose [2.9651040095676438, 0.746711881906404, 1.5856189374844674], lateral p95/max 0.012455/0.014778 m; section traversed; bag `/ws/bags/drive/curve-224910`; artifacts `docs/d0809/round3_fix1_5/`.

Round 3 fix 1 build/test exit 0; first-curve offsets after fix: 3/5 PASS.
