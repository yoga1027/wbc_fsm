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

To independently compare the trace with the original ONNX and saved training
configuration, use a Python environment with numpy, onnxruntime and PyYAML:

```bash
python tests/compare_recovery_policy.py /tmp/wbc-recovery-trace.json \
    /path/to/2026-05-09_16-23-30_3090_g1_amp_get_up_1_12288
```

The oracle reads the original exported model, joint names from the run's
`policy.onnx` metadata and full-precision parameters from `params/env.yaml`.
It verifies the model checksum, reconstructs observations/history independently,
and compares joint targets and PD gains across 18 frames, including repositioning
between entry and activation, activation with a held stick, centering, slow/fast
commands, moving joints, and re-entry while lying. A held R2+Y must not reset history.

These are offline checks, not a physics simulation or validation of get-up success.
