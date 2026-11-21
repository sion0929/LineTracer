# D-10 finish judgement evidence

Work in progress on rebased submission candidate v2; Round 3 driving edits were superseded by the lead. Driving modules are outside this task's edit scope. No completion claim until Gazebo checks run.

## Design and validation

Initial build/test started with `tools/lab_env.sh build` and `tools/lab_env.sh test`; logs: `log_lab/d10-initial-{build,test}.log`.

### Implementation, first pass

- Added explicit terminal-bar position, directed normal and confidence to `LineObservation`. A three-leg junction must have opposite bar arms, a perpendicular approaching stem, trusted projection and sufficient confidence. Corners, crossings and bare ends are excluded by synthetic tests.
- FinishJudge stores a bounded five-second odometry history, interpolates capture poses, rejects stale/duplicate frames, requires three position/orientation-consistent observations spanning 0.1 s, and tracks the directed bar into the blind zone for at most 10 s. Center crossing requires lateral/heading agreement; the judgement latches once.
- Start suppression requires both default 1 m mission travel and departure >0.6 m from the mission start pose; candidates within 0.6 m of that pose are ignored. Tracking resets at RUNNING so setup teleports cannot arm it.
- Existing STOPPING → FINISHED state transitions disable driving and retain FINISHED. No planner/controller/guard logic changed.
- TDD: new FinishJudge test initially failed compilation against the stub (`log_lab/d10-red.log`); geometry contract tests were added before interface/backend changes.
- Initial rebased test result was 206 tests / 0 errors / 2 failures in an obsolete Round 3 retrace test; current source no longer contains that superseded test. No navigation change made for this result; a clean rebuild is in progress.

### First Gazebo approach: FAIL, and isolated hook

`tools/lab_env.sh check-d10 --only finish --limit 150` failed: progress 15.665809 m / 99.791%, robot center 0.037021 m before the bar, lateral 0.004836 m, no false finish; generic planner exhausted its path, then bounded rotation recovery exhausted. Bag: `bags/d10/finish-232783`; log: `log_lab/d10-finish-first.log`. There were 519 trusted T frames (bar range 1.192 → 0.352 m). The judge correctly did not call a pre-bar stop a crossing. Build/test before this run: 215 tests, zero errors/failures/skips (`d10-test-4.log`). Earlier overlapping test attempts are superseded; final testing is serial.

A minimal D-10 planner hook is therefore unavoidable for the center-crossing criterion: mission_manager publishes a short-lived `/mission/finish_approach` target only after the same repeated, armed finish evidence. A separate pure helper accepts only the last 0.25 m, within 0.06 m lateral / 0.3 rad heading, uncertainty <0.06 and the original 10 s observation expiry; it commands 0.04 m/s using the existing controller and guard. Only the observed bar is geometry; its stem normal supplies the steering control vector. The hook requires RUNNING, fresh mission messages, fresh actual camera observations and healthy odom/IMU; it clears on memory reset. No controller or guard logic changed. This is the sole isolated change to path_planner, with its before-hook source archived in `log_lab/d10-planner-before-hook.cpp`.

A fresh four-leg observation at a remembered candidate contradicts terminal-T evidence, preventing a partial crossing view from remaining armed after its through leg becomes visible.
