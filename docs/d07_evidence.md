# D-07 — blind zone and hill path memory evidence

Status: **PASS** — frozen-source build, 142 tests, all eleven acceptance commands, and an additional joint clean-directory D-07/D-06 run passed. No owned Gazebo/launch/GCS/recorder/player processes remain.

The coordinator independently verified the D-06 output-directory fix twice on a clean copy (128 tests and the listed regressions passed). This report also records the D-07 work authorized in the same dispatch; historical notes below are followed by the final measured results.

## Plan and constraints

Test the D-06 geometry follow-ups and independent memory-validity policy first. Measure odom-only versus gyro heading at 0.25/0.5 rad/s against Gazebo truth before selecting a default. Hill traversal uses a dev-only velocity publisher; autonomous hill following remains D-09. Runtime geometry comes only from observations, never the track mesh.

## Part 0 first measurement (provisional implementation)

`tools/lab_env.sh check-d07 --heading-only` exited 0. Native Gazebo Pose_V truth (dev capture only), synchronized source stamps, same ±0.5 rad excursions. Odom-only p95 heading error / map spread: 0.25 rad/s 0.042183 rad / 0.020177 m; 0.5 rad/s 0.169138 rad / 0.050841 m. Gyro increments: 0.000534 rad / 0.002435 m and 0.001064 rad / 0.003462 m respectively. This supports enabling `imu_yaw_enabled: true`; odom translation remains unchanged and no EKF or independent-signal double fusion is introduced. Raw measurements: `log_lab/d07-heading-initial.log`, `log_lab/d07-heading-initial.json`.

The first test-first build failed on the missing policy header and gyro API; after implementation all 3 new policy tests and 13 mapping/pose tests passed. The parallel-repeat test passed the previous algorithm already; it is a regression assertion, not evidence of a fixed failure. Maximum observed memory edge is now capped at 0.08 m; long-gap input is rejected rather than connected.

## Earlier implementation and test-first failures (historical record)

- Policy implements direct → memory → checked reacquisition, geometric position/heading/connectivity agreement, mismatch rejection, uncertainty/distance slowdown and stop. Braking envelope uses 0.15 m/s² and a 0.055 m reserve; uncertainty stops at 0.06 m, slows above 0.025 m; memory/reacquisition speed caps 0.06/0.03 m/s. No new unobserved geometry is created.
- The connected-parallel duplicate test reproduced a return path along the same line (test failed before the fix); output chain traversal now excludes a neighbor whose geometry substantially overlaps the already selected chain. Long (>0.08 m) internal edges are rejected/dropped. Invalid segment IDs are filtered before graph remapping.
- First hill check failed: level hilltop still sees lower ground, producing a wrongly projected distant curve (max 0.33 m). Capture pitch alone cannot establish a single plane. Added BEV stroke half-width checks (10th quantile <0.0065 m, median >0.018 m, or a connected thin run ≥8 pixels at 4 mm spacing), followed by five consecutive good frames before projection recovery. Pitch limit remains 2°. Low-confidence observations remain available but are not inserted into confirmed odom memory. Width and recovery tests both failed before implementation. Earlier intermediate width checks still admitted brief bad frames; final runs pending.
- Flat blind-zone exhaustion initially passed: error max observed 0.0090 m / memory 0.0064 m; STOPPING at 0.05454 m verified reserve. Plot/event data are provisional until final combined check.
- First D-06 regression run failed in the updated harness on a tuple assignment; the estimator reference now compares odom XY and IMU heading, and snapshots/local-path transforms use the localization basis. Fixed by copying the tuple before substituting the reference yaw. No tolerance was increased.
- Recovery gate checks an actual sustained return to VISION (>10 status samples beyond the exit plus 0.16 m), rather than demanding the final idle sample stay VISION: honest uncertainty of old blind-zone memory may subsequently require SLOWDOWN while stationary. Status capture now records every mode message so short REACQUIRING phases cannot be missed by display throttling.

Current build/test checkpoint: initial full test run 136 tests, 0 errors/failures/skipped; more tests added since then, final full run pending.

The D-06 regression exposed a reference issue after enabling gyro heading: comparing integrated gyro to the IMU's separate absolute-orientation estimate yielded a 0.00547 rad maximum against a 0.005 rad limit. The check now uses timestamp-interpolated Gazebo true heading (dev harness only) as the independent reference, keeping the 0.005 rad limit; odom XY and GCS-to-localization comparisons are unchanged.

A full intermediate test run reported 139 tests and 5 failures (3 launch assertions duplicated in CTest/xUnit reporting). The fixtures painted an unrealistic constant 16-pixel-wide vertical line, which varies dramatically in ground width and is correctly rejected by the new plane check. The fixtures now render a 2 cm perspective stroke using the same documented camera height; sensor-loss and pre-start safety assertions remain unchanged.

The next launch regression exposed an arrival-order issue: the GCS fixture sends camera and IMU at the same 20 Hz capture stamp, but the image callback can precede the matching IMU; a previous sample at ≈50 ms triggered repeated invalidation. The detector now holds at most one pending image when its capture attitude has not arrived and processes it from the IMU callback. It does not increase the 50 ms attitude-match limit; capture stamps remain unchanged and missing IMU still causes downstream stale-input stops. A formatter also removed a braced-return semicolon; the subsequent build failed, was corrected explicitly, and will be judged on the final build.

Latest pre-acceptance build succeeded (8 packages); `tools/lab_env.sh test` is green: **139 tests, 0 errors, 0 failures, 0 skipped** (`log_lab/d07-test-fifth.log`). Detector pending-image handling fixed the equal-stamp camera/IMU launch failure without widening attitude freshness. Full final acceptance and clean-output runs follow.

## Earlier acceptance and follow-ups (superseded by final results)

The first integrated clean-output run passed D-07 (122.56 s, both `log_lab/d07` and `docs/d07` initially absent) and D-06 (71.72 s, `log_lab/d06` initially absent), preserving prior/fresh artifacts under `log_lab/d07-final/*-directory-audit`. Build/test/D-05 also passed. During review I hardened one additional invariant: reacquisition proposals may advance the verification counter, but only the accepted, non-expired, sufficiently certain contiguous prefix may contribute to published verified distance and speed. A test-first compile failed on the missing `verified_prefix` API; all five policy tests now pass. Full production rebuild/regression rerun is pending this source addition.

Measurement caveat: conservative re-observation rejection can freeze memory, making retained-map spread artificially small (odom-only 0.5 rad/s had zero accepted observed points in the first final run). The heading comparison now additionally projects **all projection-valid raw observations** using timestamp-matched localization, and measures normal spread in fixed bins on the same long straight. The report will include raw spread, retained spread and accepted point counts separately; frozen memory is not claimed as an improvement. Independent true-heading errors still strongly favor gyro increments.

Further follow-up: a repeat fast-turn check correctly failed its accepted-observation gate even though gyro heading and raw mapped spread were good. Strictly treating a connection absent from a partial previous graph as forbidden was wrong. New observation connections are now checked for a ≤0.04 m endpoint gap; already recorded junctions must agree within 0.06 m, while absence in a partial view remains UNKNOWN. Tests reproduced both a physically distant connected-ID edge and an impossible fresh connection before the fixes. Memory also removes stale connected IDs when merged endpoints move farther than 0.04 m apart; this directly addresses the D-06 long-connector concern. All six policy and fifteen mapping/pose tests pass.

The first full acceptance ledger has one failed bag metadata row: I changed `src/` test code while the recorder was running, so the start-time fingerprint correctly differed from the later verification hash. The bag closed cleanly and other recording/event/UI rows passed; this failure is retained in `log_lab/d07-final/check-bag.log`. The next complete run uses frozen source files throughout recording. No fingerprint assertion is changed.

## Final implementation and acceptance

Final source was frozen for the complete acceptance run: **139 source/tool files unchanged**, verified in `log_lab/d07-final-v2/source-freeze-verification.json`. All runs used the lab container; each simulator used its dedicated ROS domain and unique Gazebo partition. No supplied world/model/checksum file or design checkbox was edited.

Capture-time odom history now optionally uses gyro-integrated heading increments, anchored once to wheel heading; translation remains wheel odom. There is no EKF or repeated fusion of correlated absolute yaw estimates. `imu_yaw_enabled: true` is the competition/dev/replay YAML default after the independent Gazebo comparison; setting it false retains the wheel-heading baseline. Mission snapshots use localization for robot pose and selected-path conversion. Snapshot subscriptions remain disabled in competition by default.

The detector keeps capture stamps and rejects untrusted flat-plane projections using capture-time pitch/roll, projected stroke width and five consecutive good recovery frames. It retains diagnostic low-confidence points but the mapper does not confirm them. At equal camera/IMU stamps it buffers at most one pending image until capture attitude arrives; the 50 ms match limit was not widened. Fast-SCNN remains the D-15 stub/fallback.

The memory policy uses VISION_TRACKING → MEMORY_TRACKING → REACQUIRING → VISION_TRACKING, with SLOWDOWN/STOPPING overriding insufficient distance or uncertainty. Three consistent observations verify reacquisition; fifteen unresolved mismatch frames stop. Position/heading and recorded junction geometry must agree; a new connection absent from an earlier partial view is allowed only with nearby endpoints. Internal memory edges >0.08 m and connected-ID endpoint gaps >0.04 m are rejected/removed; a recorded junction may move no more than 0.06 m during re-observation. A substantially overlapping parallel neighbor cannot double back through the local path.

Only an accepted, non-expired contiguous point prefix with uncertainty <0.06 m contributes to published path validity/speed. A reacquisition proposal cannot inflate the published verified distance. Speed uses a 0.055 m reserve, 0.15 m/s² braking envelope and uncertainty factor; memory/reacquisition caps are 0.06/0.03 m/s, slowdown cap 0.04 m/s, and uncertainty >0.025 m or distance <0.25 m requires slowdown. No UNKNOWN points or new unobserved geometry are output. Stale odom/IMU stops; remaining trusted memory can be updated during observation loss.

### Re-run commands and final results

| Exact command | Exit | Wall s | Evidence |
|---|---:|---:|---|
| `tools/lab_env.sh build` | 0 | 21.85 | `log_lab/d07-final-v2/build.log` |
| `tools/lab_env.sh test` | 0 | 19.12 | `log_lab/d07-final-v2/test.log` |
| `tools/lab_env.sh check-d07` | 0 | 122.46 | `log_lab/d07-final-v2/check-d07.log` |
| `tools/lab_env.sh check-d06` | 0 | 72.66 | `log_lab/d07-final-v2/check-d06.log` |
| `tools/lab_env.sh check-d05` | 0 | 52.75 | `log_lab/d07-final-v2/check-d05.log` |
| `tools/lab_env.sh checkpoint-a` | 0 | 113.49 | `log_lab/d07-final-v2/checkpoint-a.log` |
| `tools/lab_env.sh check-gcs` | 0 | 26.36 | `log_lab/d07-final-v2/check-gcs.log` |
| `tools/lab_env.sh check-bag` | 0 | 37.20 | `log_lab/d07-final-v2/check-bag.log` |
| `tools/lab_env.sh check-camera-rate` | 0 | 65.25 | `log_lab/d07-final-v2/check-camera-rate.log` |
| `tools/lab_env.sh check-teardown` | 0 | 52.89 | `log_lab/d07-final-v2/check-teardown.log` |
| `docker build -t line-follower-d07:local .` | 0 | 0.45 | `log_lab/d07-final-v2/docker-build.log` |

Key exact output lines from the frozen-source run:

```text
Summary: 8 packages finished [21.3s]
Summary: 142 tests, 0 errors, 0 failures, 0 skipped
CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)
GCS D-11/D-12: PASS (26/26)
D-13 RECORD/REPLAY: PASS; artifacts=/ws/bags/check-stage2/67593c4d
CAMERA RATE: PASS
TEARDOWN: PASS (10/10 clean)
D07 FINAL ACCEPTANCE: PASS
```

The production Docker build succeeded for the unchanged six-package `--packages-up-to line_follower_bringup`, Release, BUILD_TESTING=OFF configuration. The final identical-source build reused its verified cache (the preceding matching uncached build took 52.16 s). Image: `sha256:1b4d54afb72fa1dfba9d96212baffbbdfb5f53d23d87e912a34265b6bf4c171d`. No Qt/GCS/recorder dependency was added to production.

### Joint clean-directory proof

After the complete regression run, **all three** `log_lab/d07`, `log_lab/d06`, `docs/d07` directories were moved aside together before starting the sequence. D-07 did not create the D-06 output directory, so D-06 also started without its output. Both commands passed; prior directories were restored and final generated tables/plots refreshed. Prior/fresh evidence is preserved under the earlier v2 directory audits and `log_lab/d07-clean-joint/directory-audit/fresh-*`. The initial small directory-fix audit remains in `log_lab/d06-directory-audit-20261119T132148Z`.

```json
{
  "all_three_initially_absent": true,
  "prior_directories_restored": true,
  "results": [
    {
      "command": "tools/lab_env.sh check-d07",
      "exit_code": 0,
      "wall_sec": 122.00676193705294,
      "d06_output_absent_at_start": true,
      "log": "log_lab/d07-clean-joint/check-d07.log"
    },
    {
      "command": "tools/lab_env.sh check-d06",
      "exit_code": 0,
      "wall_sec": 71.77475429407787,
      "d06_output_absent_at_start": true,
      "log": "log_lab/d07-clean-joint/check-d06.log"
    }
  ]
}
```

### Final heading / mapping measurements

Latest joint clean run, independent native Gazebo model-pose truth, source-stamp alignment. Raw map spread projects every projection-valid camera observation with the chosen capture heading and takes the worst normal standard deviation in fixed 5 cm bins on the same long straight. Retained-map spread is reported separately because conservative rejection can freeze memory and artificially lower that statistic. All chosen-source fast-turn gates passed (raw/retained spread <0.015 m, heading p95 <0.01 rad, accepted observed geometry present).

| Source | Command rad/s | True-heading p95 rad | Raw map spread m | Retained spread m | Accepted observed points |
|---|---:|---:|---:|---:|---:|
| wheel odom | 0.25 | 0.042061 | 0.017660 | 0.014760 | 1365 |
| wheel odom | 0.5 | 0.160899 | 0.067395 | 0.007839 | 153 |
| gyro increments | 0.25 | 0.000559 | 0.001605 | 0.001373 | 1731 |
| gyro increments | 0.5 | 0.001032 | 0.003995 | 0.002831 | 767 |

Decision: enable gyro increments. At 0.5 rad/s the measured true-heading p95 fell from about 0.161 to 0.00103 rad, and raw map spread from 0.0674 to 0.00399 m. This claims an improvement in measured heading/mapping, not an EKF or moving-turn translation improvement. Actual true heading extrema and observation counts are retained in `docs/d07/measurements.json`.

### Final D-07 PASS tables

| Scenario | Confirmed error median/p95/max m | Memory error median/p95/max m | Largest continuous observation loss s |
|---|---|---|---:|
| hill | 0.001643/0.011272/0.025170 | 0.000097/0.025417/0.034348 | 16.896 |
| blind | 0.000761/0.005162/0.009803 | 0.000009/0.004828/0.005744 | 9.008 |

Gates: confirmed median/p95/max <0.025/0.045/0.080 m; memory p95/max <0.055/0.100 m; no UNKNOWN, uncertain or expired confirmed output points; capture-pitch-invalid slope frames; checked return to VISION beyond the exit; positive reserve at blind exhaustion. **All PASS**. The blind-end first STOPPING retained **0.097573 m** verified distance. The harness drives at 0.1 m/s over the hill and 0.06 m/s at the flat end, deliberately continuing after the planner requests a stop; autonomous following is D-09.

Exact final check output:

```text
PASS | dev graph command ownership    | publishers=['checkpoint_a_d05']; no motion_controller/safety_guard
PASS | heading measured               | {"raw_mapped_frames": 190, "raw_mapped_points": 4897, "raw_map_spread_m": 0.017659867611223578, "mapped_observed_points": 1365, "observed_p95_m": 0.021411315765775374, "true_yaw_min_max": [-0.47101037554786385, 0.5050372332555007], "rate": 0.25, "imu_yaw": false, "heading_error_p95_rad": 0.042060539140236275, "map_spread_m": 0.014760364079597802, "max_memory_edge_m": 0.07295066370151888, "localization_samples": 603, "truth_samples": 610}
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-d05-sumwwf1g/launch.log
PASS | dev graph command ownership    | publishers=['checkpoint_a_d05']; no motion_controller/safety_guard
PASS | heading measured               | {"raw_mapped_frames": 93, "raw_mapped_points": 2438, "raw_map_spread_m": 0.06739534390237978, "mapped_observed_points": 153, "observed_p95_m": 0.05014958999192674, "true_yaw_min_max": [-0.3808358852856661, 0.5142565217800378], "rate": 0.5, "imu_yaw": false, "heading_error_p95_rad": 0.1608987035327654, "map_spread_m": 0.00783901447782373, "max_memory_edge_m": 0.06478469519919157, "localization_samples": 323, "truth_samples": 426}
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-d05-sew55yzb/launch.log
PASS | dev graph command ownership    | publishers=['checkpoint_a_d05']; no motion_controller/safety_guard
PASS | heading measured               | {"raw_mapped_frames": 154, "raw_mapped_points": 3987, "raw_map_spread_m": 0.0016048791193291059, "mapped_observed_points": 1731, "observed_p95_m": 0.0116498956347993, "true_yaw_min_max": [-0.4728985823277722, 0.5011365447616022], "rate": 0.25, "imu_yaw": true, "heading_error_p95_rad": 0.000558653152606571, "map_spread_m": 0.001373325153658942, "max_memory_edge_m": 0.07130514055256133, "localization_samples": 602, "truth_samples": 634}
PASS | fast turn map and heading gate | raw spread=0.001605m and retained spread=0.001373m <.015; true heading p95=0.000559rad <.01; edge max=0.071305m
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-d05-t78xt18c/launch.log
PASS | dev graph command ownership    | publishers=['checkpoint_a_d05']; no motion_controller/safety_guard
PASS | heading measured               | {"raw_mapped_frames": 86, "raw_mapped_points": 2238, "raw_map_spread_m": 0.0039946407155896065, "mapped_observed_points": 767, "observed_p95_m": 0.013005332579677951, "true_yaw_min_max": [-0.38262745829565453, 0.5080341427301656], "rate": 0.5, "imu_yaw": true, "heading_error_p95_rad": 0.0010318372837093194, "map_spread_m": 0.0028311511443000694, "max_memory_edge_m": 0.07837818099994386, "localization_samples": 323, "truth_samples": 432}
PASS | fast turn map and heading gate | raw spread=0.003995m and retained spread=0.002831m <.015; true heading p95=0.001032rad <.01; edge max=0.078378m
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-d05-7ib66ku_/launch.log
PASS | dev graph command ownership    | publishers=['checkpoint_a_d05']; no motion_controller/safety_guard
PASS | confirmed and memory ground truth | {"scenario": "hill", "confirmed_error_m": {"median": 0.0016428744726521005, "p95": 0.011272020451243536, "max": 0.02516969729019891}, "memory_error_m": {"median": 9.66252636954712e-05, "p95": 0.02541677413633771, "max": 0.0343483668332496}}
PASS | no UNKNOWN or uncertain/expired confirmed points | paths=130 UNKNOWN=0 uncertain/expired=0
PASS | loss and checked recovery      | modes=[0, 1, 3, 5, 2, 0, 3]; VISION samples beyond exit+0.16m=97; final mode=3
PASS | capture pitch validity gate    | slope frames=323; confirmed=0
PASS | mode timeline                  | [{"t": 4.064, "mode": 0, "remaining": 1.1799999475479126, "uncertainty": 0.00800000037997961, "detail": "trusted current observation"}, {"t": 5.26, "mode": 1, "remaining": 1.1610525846481323, "uncertainty": 0.014922389760613441, "detail": "trusted memory; projection or near line lost"}, {"t": 6.84, "mode": 3, "remaining": 1.0030525922775269, "uncertainty": 0.025192823261022568, "detail": "trusted memory; projection or near line lost; verified distance/uncertainty requires slowdown"}, {"t": 13.22, "mode": 5, "remaining": 0.0, "uncertainty": 0.0, "detail": "no sufficiently certain stoppable observed path"}, {"t": 22.188, "mode": 2, "remaining": 0.0, "uncertainty": 0.0, "detail": "checking position heading and connectivity"}, {"t": 22.26, "mode": 0, "remaining": 1.2677172422409058, "uncertainty": 0.00800000037997961, "detail": "consistent re-observation verified"}, {"t": 25.56, "mode": 3, "remaining": 1.097815752029419, "uncertainty": 0.02514694444835186, "detail": "trusted current observation; verified distance/uncertainty requires slowdown"}]
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-d05-bhhzly8f/launch.log
PASS | dev graph command ownership    | publishers=['checkpoint_a_d05']; no motion_controller/safety_guard
PASS | confirmed and memory ground truth | {"scenario": "blind", "confirmed_error_m": {"median": 0.0007607983662101248, "p95": 0.005161603590713016, "max": 0.009802594478988014}, "memory_error_m": {"median": 8.908025298737243e-06, "p95": 0.004827751968345728, "max": 0.00574442933357544}}
PASS | no UNKNOWN or uncertain/expired confirmed points | paths=95 UNKNOWN=0 uncertain/expired=0
PASS | verified distance exhaustion   | SLOWDOWN=203 STOPPING=169 first stop remaining=0.09757301211357117; remaining range=0.0002..0.7000
PASS | mode timeline                  | [{"t": 3.604, "mode": 0, "remaining": 0.699999988079071, "uncertainty": 0.00800000037997961, "detail": "trusted current observation"}, {"t": 7.008, "mode": 3, "remaining": 0.5, "uncertainty": 0.025066746398806572, "detail": "trusted current observation; verified distance/uncertainty requires slowdown"}, {"t": 13.7, "mode": 5, "remaining": 0.09757301211357117, "uncertainty": 0.03611525148153305, "detail": "trusted memory; projection or near line lost; verified distance/uncertainty requires slowdown"}]
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-d05-_g0rdsi1/launch.log
D-07: PASS
```


hill mode timeline (simulation seconds from scenario first status):

| Elapsed s | Mode | Verified m | Uncertainty m |
|---:|---|---:|---:|
| 0.000 | VISION_TRACKING | 1.180000 | 0.008000 |
| 1.196 | MEMORY_TRACKING | 1.161053 | 0.014922 |
| 2.776 | SLOWDOWN | 1.003053 | 0.025193 |
| 9.156 | STOPPING | 0.000000 | 0.000000 |
| 18.124 | REACQUIRING | 0.000000 | 0.000000 |
| 18.196 | VISION_TRACKING | 1.267717 | 0.008000 |
| 21.496 | SLOWDOWN | 1.097816 | 0.025147 |

blind mode timeline (simulation seconds from scenario first status):

| Elapsed s | Mode | Verified m | Uncertainty m |
|---:|---|---:|---:|
| 0.000 | VISION_TRACKING | 0.700000 | 0.008000 |
| 3.404 | SLOWDOWN | 0.500000 | 0.025067 |
| 10.096 | STOPPING | 0.097573 | 0.036115 |

### Design §14.2 measurement columns

Values below are **reported trusted flat-plane observation ranges**, measured during the hill traversal; unavailable means no confirmed projection, not absence of image-space line detections. The level top looking down at lower ground was deliberately untrusted. No 3D hill reconstruction is claimed.

| Measure | Flat / before entry | Entry / uphill | Top | Downhill / exit |
|---|---|---|---|---|
| Nearest valid m, mean [min,max] | 0.316 mean [0.316, 0.316] | unavailable | unavailable | 0.315 mean [0.300, 0.316] |
| Farthest valid m, mean [min,max] | 1.178 mean [1.176, 1.180] | unavailable | unavailable | 0.823 mean [0.769, 0.990] |
| Right-angle first detection | not scanned; D-06 corner already seen at 0.60 m | N/A | N/A | N/A |
| Confirmed ground error max m | 0.004013 | unavailable | unavailable | 0.025170 |
| Continuous invalid projection max s within region | 2.148 | 4.456 | 4.388 | 5.808 |
| Confirmed / total frames | 36/102 | 0/136 | 0/134 | 114/290 |

Maximum continuous loss across region boundaries: 16.896 s. The right-angle onset distance was not scanned in this straight-hill scenario; D-06 confirms the corner at its initial 0.60 m approach distance, which is a lower bound rather than a claimed first-onset limit. Per-frame capture time/pitch/ranges/errors: `log_lab/d07/hill_frames.json`.

### D-06 follow-up moving results

Latest joint run retained the existing geometry/pose/spread tolerances. Heading is checked against Gazebo truth because the estimator now uses gyro increments; XY remains checked against reliable odometry and snapshot pose against localization.

| Scenario | Observed median/p95/max m | Memory median/p95/max m | Points peak/end | Normal spread m |
|---|---|---|---:|---:|
| straight | 0.000078/0.011207/0.022946 | 0.004429/0.012831/0.026790 | 238/226 | 0.000015 |
| corner | 0.011539/0.020107/0.024486 | 0.010835/0.017331/0.019953 | 123/120 | 0.000000 |
| rotate | 0.003207/0.013612/0.029909 | 0.004482/0.012723/0.027135 | 304/194 | 0.010948 |

All D-06 rows PASS; exact output: `log_lab/d07-clean-joint/check-d06.log`. Storage ≤1,200 points/32 segments, UNKNOWN=0, blind-zone memory and sharp corner checks passed, each owned group cleaned.

### Unit / node tests

- `test_memory_policy.cpp` (6): loss/reacquisition/mismatch transitions; monotonic speed with distance/uncertainty; pitch/expiry/health rejection; geometric heading/junction agreement; accepted non-expired contiguous prefix; physically local new connections.
- `test_mapping_memory.cpp` (15): capture interpolation/wrapping/age, both-sensor freshness and frozen-clock receipt health, gyro integration/reset, known frame transform, dedup/caps, expiry/travel uncertainty, run/fault resets, blind sharp-corner memory, fixed old geometry under moving views, shared corner anchor, long-edge rejection, parallel repeated views, connected duplicate retracing, distant connected-ID removal.
- Detector backend (16 total, two new): projected-width inconsistency rejection and stable projection recovery after a slope; existing straight/corner/crossing/T/arc/edge/faint-line/fallback/projection tests remain green. The straight unit/launch fixtures now use a physical 2 cm perspective stroke.
- Existing GCS launch test exercises equal-stamp 20 Hz image/IMU arrival ordering; composed pipeline tests preserve capture stamps, parameter loading, five-node messages and safety assertions. Full `colcon test` reports 142 tests, zero errors/failures/skips.

### Regression key measurements

```text
camera dimensions and rate: 640x480; measured 30.28 Hz wall (world target 30 Hz)
controller loss physical stop: 0.232s; safety reason command candidate stale
safety_guard exit: post-exit messages=5; last_zero=True; physical stop=0.056s
GCS snapshot rate: 6 samples; 2.000 Hz; max relay=0.0015s
link loss: LOST after 2.022s; all five rover nodes alive
UI: 36 snapshots / 78 camera frames / 19.99s; 1.80 / 3.90 Hz
camera reliable bags: 702 frames / 23.123s and 708 / 23.321s; both 30.316 Hz
replay never publishes cmd_vel: max publishers=0; received=0
replay cmd_vel_blocked: 509 messages; nonzero=353
regenerated: path/local=480, path/memory=432, path/status=480, perception/line=428, perception/status=428
TEARDOWN: PASS (10/10 clean)
```

T-02 checksums and T-05 static policy remain green. The final bag metadata fingerprint matched unchanged source; the earlier changed-source failure is preserved above. The UI layout and one-line competition launch were not changed.

### Artifacts and files changed

D-07 PNGs: `docs/d07/heading_{false,true}_{0.25,0.5}_topdown.png`, `docs/d07/hill_topdown.png`, `docs/d07/hill_timeline.png`, `docs/d07/blind_topdown.png`, `docs/d07/blind_timeline.png`. Structured final table: `docs/d07/measurements.json`. Earlier heading/progress JSON was moved into `log_lab/d07/diagnostic-*` to keep the final artifact unambiguous. D-05/D-06 plots and the existing GCS capture were regenerated by regression checks.

Code/document files added or changed (27; generated plots/JSON above are additional):

- `README.md`
- `docs/test_plan.md`
- `docs/d06_evidence.md`
- `docs/d07_evidence.md`
- `src/line_follower_navigation/CMakeLists.txt`
- `src/line_follower_navigation/include/line_follower_navigation/memory_policy.hpp`
- `src/line_follower_navigation/include/line_follower_navigation/pose_estimator.hpp`
- `src/line_follower_navigation/src/pose_estimator.cpp`
- `src/line_follower_navigation/src/local_path_memory.cpp`
- `src/line_follower_navigation/src/path_planner_node.cpp`
- `src/line_follower_navigation/test/test_mapping_memory.cpp`
- `src/line_follower_navigation/test/test_memory_policy.cpp`
- `src/line_follower_mission/src/mission_manager_node.cpp`
- `src/line_follower_perception/include/line_follower_perception/line_detector_backend.hpp`
- `src/line_follower_perception/src/line_detector_backend.cpp`
- `src/line_follower_perception/src/line_detector_node.cpp`
- `src/line_follower_perception/test/test_line_detector_backend.cpp`
- `src/line_follower_bringup/config/competition.yaml`
- `src/line_follower_tests/test/test_gcs_launch.py`
- `src/line_follower_tests/test/test_pipeline_launch.py`
- `tools/check_d07.py`
- `tools/check_d06.py`
- `tools/check_d05.py`
- `tools/d05_capture.launch.py`
- `tools/check_camera_rate.py`
- `tools/check_bag.py`
- `tools/lab_env.sh`

### Limits / remaining scope

No required D-07 gate remains unverified. Projection trust is a conservative flat-plane heuristic; there is no reconstructed 3D hill surface. The hilltop and slopes may offer no confirmed projection, and the planner correctly stops when trusted memory expires/becomes uncertain; the dev harness, not autonomous control, carries the rover through that interval. Gyro heading was validated on the stated flat rotations; wheel translation remains unchanged, and real gyro bias/moving-turn localization/EKF require later evaluation. D-08 decisions, D-09 autonomous hill/corner following and D-15 learning/EKF remain outside this task. First-onset right-angle visibility was not scanned in the straight-hill run.

Final cleanup: `docker exec line_follower_lab ps -eo pid,ppid,pgid,args` showed only container init, its persistent sleep, and the inspection command. No owned Gazebo, launch, GCS, recorder or player remained (`log_lab/d07-final-v2/final-processes.txt`).
