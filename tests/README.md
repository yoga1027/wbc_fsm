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
