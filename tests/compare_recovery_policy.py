"""Compare a C++ state trace with the original ONNX and saved training parameters.

Requires numpy, onnxruntime and PyYAML in the training Python environment.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import onnxruntime as ort
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("training_run", type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    filename = "Unitree-G1-AMP-Flat_model_40000.onnx"
    original = args.training_run / "export" / filename
    deployed = repo / "model" / "loco" / filename
    assert hashlib.sha256(original.read_bytes()).digest() == hashlib.sha256(deployed.read_bytes()).digest()

    # BaseLoader treats Python-specific YAML tags as data without executing them.
    cfg = yaml.load((args.training_run / "params/env.yaml").read_text(), Loader=yaml.BaseLoader)
    actor = cfg["observations"]["actor"]
    assert list(actor["terms"]) == ["base_ang_vel", "projected_gravity", "command", "joint_pos", "joint_vel", "actions"]
    assert actor["history_length"] == "4" and actor["history_ordering"] == "time"
    assert float(cfg["sim"]["mujoco"]["timestep"]) * int(cfg["decimation"]) == 0.02
    robot = cfg["scene"]["entities"]["robot"]
    metadata = ort.InferenceSession(str(args.training_run / "policy.onnx"),
                                    providers=["CPUExecutionProvider"]).get_modelmeta().custom_metadata_map
    names = metadata["joint_names"].split(",")
    assert len(names) == 29

    def expand(pattern_values, default=0.0):
        result = np.full(29, default, dtype=np.float32)
        for pattern, value in pattern_values.items():
            for i, name in enumerate(names):
                if re.fullmatch(pattern, name):
                    result[i] = float(value)
        return result

    offset = expand(robot["init_state"]["joint_pos"])
    scale = expand(cfg["actions"]["joint_pos"]["scale"], np.nan)
    kp = np.full(29, np.nan, dtype=np.float32)
    kd = kp.copy()
    for actuator in robot["articulation"]["actuators"]:
        for i, name in enumerate(names):
            if any(re.fullmatch(pattern, name) for pattern in actuator["target_names_expr"]):
                kp[i] = float(actuator["stiffness"])
                kd[i] = float(actuator["damping"])
    assert np.isfinite(scale).all() and np.isfinite(kp).all() and np.isfinite(kd).all()
    session = ort.InferenceSession(str(original), providers=["CPUExecutionProvider"])
    history = None
    action = np.zeros(29, dtype=np.float32)
    worst_error = 0.0
    trace = json.loads(args.trace.read_text())
    for row in trace:
        if row["reset"]:
            action.fill(0)
        quat = np.array(row["quat"], dtype=np.float32)
        quat /= np.linalg.norm(quat)
        world_gravity = np.array([0, 0, -1], dtype=np.float32)
        # Independently calculate R(q)^T * gravity.
        xyz = quat[1:]
        cross = 2 * np.cross(xyz, world_gravity)
        gravity = world_gravity - quat[0] * cross + np.cross(xyz, cross)
        frame = np.concatenate((row["gyro"], gravity, row["command"],
                                np.array(row["q"], dtype=np.float32) - offset,
                                row["dq"], action)).astype(np.float32)
        history = np.tile(frame, (4, 1)) if row["reset"] else np.vstack((history[1:], frame))
        action = session.run(["actions"], {"obs": history.reshape(1, 384)})[0][0]
        target = offset + action * scale
        error = np.max(np.abs(target - row["target"]))
        worst_error = max(worst_error, float(error))
        np.testing.assert_allclose(row["target"], target, atol=2e-5, rtol=1e-5)
        np.testing.assert_allclose(row["kp"], kp, atol=2e-5, rtol=1e-6)
        np.testing.assert_allclose(row["kd"], kd, atol=2e-6, rtol=1e-6)
    print(f"PASS: {len(trace)} frames, original model SHA256 matches; "
          f"max target error {worst_error:.3g} rad; training Kp/Kd match")


if __name__ == "__main__":
    main()
