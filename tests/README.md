# Recovery policy checks

These standalone tests do not construct IOSDK, connect to DDS, or control a robot.
Build from the repository root with the bundled x64 ONNX Runtime 1.22.0:

```bash
cmake -S tests -B /tmp/wbc-recovery-tests
cmake --build /tmp/wbc-recovery-tests -j4
ctest --test-dir /tmp/wbc-recovery-tests --output-on-failure
/tmp/wbc-recovery-tests/recovery_state_test /tmp/wbc-recovery-trace.json
```

The state test checks persistent damping while waiting, explicit R2+Y activation
after release, rejection of pre-held/masked start commands, and blocked policy
switches while waiting. It also checks lying activation, all four entry routes, manual return to both
policies, orientation/angular-velocity guards, rejected held switch requests,
policy re-entry, non-finite observations/commands, invalid quaternions, incorrect
control periods, damping/SELECT, and gamepad command priority/release handling.
The FSM integration test uses an in-memory IO implementation to check actual
transition scheduling with the production entry latch, waiting damping publication,
fault damping publication, and the final damping send
before SELECT ends the main loop.

The startup test checks the production handoff orchestration with injected RPCs,
feedback, publisher creation and a fake clock. It covers default simulation mode,
explicit hardware/interface selection, simulation RPC bypass, release before
feedback becomes available, slow shutdown, transient query/release errors and
exceptions, consecutive empty-mode confirmation, reappearing services on the final
check, missing/stale feedback, cancellation and the total deadline (including late
RPC replies and reduced remaining RPC budgets). No hardware service is called.
The recovery FSM test starts directly from PASSIVE via R2+X, without START.

To independently compare the trace with the original ONNX and saved training
configuration, use a Python environment with numpy, onnxruntime and PyYAML:

```bash
python tests/compare_recovery_policy.py /tmp/wbc-recovery-trace.json \
    /path/to/2026-05-09_16-23-30_3090_g1_amp_get_up_1_12288
```

The oracle is pinned to the original model_40000 baseline. If the recovery config
selects another checkpoint, this comparison is not a validation of that model;
adapt the reference model/run before using it. The ordinary C++ state/FSM tests
exercise the model currently selected by the recovery config.

The oracle reads the original exported model, joint names from the run's
`policy.onnx` metadata and full-precision parameters from `params/env.yaml`.
It verifies the model checksum, reconstructs observations/history independently,
and compares joint targets and PD gains across 18 frames, including repositioning
between entry and activation, activation with a held stick, centering, slow/fast
commands, moving joints, and re-entry while lying. A held R2+Y must not reset history.

These are offline checks, not a physics simulation or validation of get-up success.

## Automatic Loco recovery

The same build/ctest commands also run `loco_recovery_state_test` and
`loco_recovery_fsm_test`. The state test uses the configured real ONNX policies to
check directional tilt-rate detection, yaw/inward-motion rejection, consecutive
sample debounce, immediate hard-tilt takeover, joint/IMU standing gates, interrupted
stability holds, repeated recovery, PD blending, re-fall during return, held joystick
suppression, recurrent/history resets, invalid feedback, long recoveries and manual stops.
Long recoveries keep joint support after the warning deadline and can still return
when standing. Straight knees and the wide/rotated hip stance observed in actual
Recovery simulation are accepted; sitting/kneeling and excessive joint motion
remain blocked.
Takeover regressions compare every q/dq/tau/Kp/Kd field against an independent
Recovery ONNX session. They check progressive live-target handoff, compressed
time budgets, immediate hard/fast-risk takeover, worsening risk during a handoff,
repeat falls starting from an unfinished return command, direct activation from
damping, and stop/SELECT/invalid-feedback priority during blending.
It verifies that the borrowed Recovery policy still waits for manual activation
when entered independently afterwards.

The composite FSM test uses fake IO and the actual 50 Hz production scheduler.
It checks manual startup, the fixed-stand completion gate, next-cycle publication
of takeover commands, automatic return within the composite, rejection of held
mode requests, routing of manual recovery back into the composite, fault damping,
stop cancellation of pending transitions, and SELECT's final damping flush.
The original recovery tests continue to run with automatic recovery disabled.
None of these tests publishes DDS commands or connects to a physical robot.

For the isolated physics regression with the adjacent `unitree_mujoco` checkout:

```bash
cmake -S tests -B /tmp/wbc-recovery-tests -DENABLE_MUJOCO_RECOVERY_TEST=ON
cmake --build /tmp/wbc-recovery-tests -j4
ctest --test-dir /tmp/wbc-recovery-tests --output-on-failure
```

`loco_recovery_physics_test` links MuJoCo directly, without its viewer or DDS bridge.
It runs the actual configured ONNX policies at 50 Hz and applies the bridge's PD
torque formula in a private G1 scene. It exercises four 0.5 s torso pushes:
forward/backward 200 N for adaptive handoff, and forward/sideways 250 N for
immediate takeover. Each case must enter Recovery, return to Loco, and sustain
upright walking in response to joystick input after return. The stronger pushes
also have to exercise a physical fall. Each case runs 25 seconds of simulated time.
An independent Recovery session sees the same feedback as the composite and
supplies the unmixed comparison commands. The test reports the first target/PD
torque jumps and the peak estimated PD torque jump in the first 200 ms. These
comparisons use the same feedback sequence, not separate baseline trajectories;
PD estimates exclude actuator saturation and are not measured motor torque.
Override `MUJOCO_ROOT` / `RECOVERY_TEST_SCENE` if the simulator checkout is elsewhere.
The test loads the current `config/mjamp_recovery.json` model. This regression
covers these simulated disturbances, not all falls or hardware behavior.
