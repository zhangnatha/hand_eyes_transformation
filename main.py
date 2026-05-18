#!/usr/bin/env python3
"""仓库入口：默认打印演示输出；`--verify` 执行轻量自检。

导入 `pose_vector_math` 前，将 `interface/` 添加到模块路径。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_INTERFACE = _ROOT / "interface"
if str(_INTERFACE) not in sys.path:
    sys.path.insert(0, str(_INTERFACE))


def _run_verify() -> int:
    from pose_vector_math import runVerification  # noqa: E402

    return 0 if runVerification() else 1


def _run_demo() -> None:
    """使用与 `main.cpp` 相同流程和值的演示输出。"""
    import numpy as np
    from pose_vector_math import PoseVectorMath

    def _six(p, prec: int = 4) -> str:
        p = np.asarray(p, dtype=float).flatten()
        return ",".join(f"{x:.{prec}f}" for x in p)

    def _three(v, prec: int = 4) -> str:
        v = np.asarray(v, dtype=float).flatten()
        return ",".join(f"{x:.{prec}f}" for x in v)

    m = PoseVectorMath()
    label_width = 48

    target_in_camera = [130.51, 27.86, 515.003, 179.46, 4.58, 166.62]

    print("--- eyes to hand (ETH) ---")
    camera_in_base = [-687.029, -412.132, -406.297, -44.9606, -33.0434, -1.92313]
    target_in_base_eth = m.pose3dMultiply(camera_in_base, target_in_camera)
    print(f"{'target_in_base[ETH]':<{label_width}}: {_six(target_in_base_eth)}")

    print("\n--- eyes in hand (EIH) ---")
    camera_in_flange = [206.9900, -331.5936, -426.7618, 177.1207, 1.5365, 101.0783]
    flange_in_base = [566.417, 55.603, -78.360, -180.0, 0.0, 147.16]
    target_in_flange = m.pose3dMultiply(camera_in_flange, target_in_camera)
    target_in_base_eih = m.pose3dMultiply(flange_in_base, target_in_flange)
    print(f"{'target_in_base[EIH]':<{label_width}}: {_six(target_in_base_eih)}")

    pose_from_camera_to_target = m.pose3dInverse(target_in_camera)
    pose_from_base_to_camera = camera_in_base

    print("\n--- pose API (pose3dInverse(target_in_camera), camera_in_base) ---")
    print(f"{'pose3dInverse(target_in_camera)':<{label_width}}: {_six(pose_from_camera_to_target)}")
    pose_distance = m.pose3dDistance(pose_from_camera_to_target, pose_from_base_to_camera)
    print(f"{'pose3dDistance(target,camera)':<{label_width}}: {pose_distance:.4f}")
    pose_offset = m.pose3dOffset(pose_from_camera_to_target, pose_from_base_to_camera)
    print(f"{'pose3dOffset(target,camera)':<{label_width}}: {_six(pose_offset)}")
    angle_deg, rotation_axis = m.pose3dAngle(pose_from_camera_to_target, pose_from_base_to_camera)

    print(f"{'pose3dAngle(target,camera)':<{label_width}}: {angle_deg:.4f}° | {rotation_axis[0]:.4f},{rotation_axis[1]:.4f},{rotation_axis[2]:.4f}")
    pose_trans = m.pose3dGetTrans(pose_from_camera_to_target)
    print(f"{'pose3dGetTrans(target)':<{label_width}}: {_three(pose_trans)}")
    pose_rpy = m.pose3dGetRpy(pose_from_camera_to_target)
    print(f"{'pose3dGetRpy(target)':<{label_width}}: {_three(pose_rpy)}")

    print("\n--- pose6dToHomogeneous(pose_from_camera_to_target) ---")
    print(f"{'Input pose_from_camera_to_target':<{label_width}}: {_six(pose_from_camera_to_target)}")
    t = m.pose6dToHomogeneous(pose_from_camera_to_target)
    for i in range(4):
        print("  ", "  ".join(f"{float(t[i, j]):14.8f}" for j in range(4)))

    print("\n--- vector ---")
    v0 = [130.51, 27.86, 515.003]
    v1 = [566.417, 55.603, -78.360]
    print(f"{'v0':<{label_width}}: {_three(v0)}")
    print(f"{'v1':<{label_width}}: {_three(v1)}")
    v0_norm = m.vector3dNorm(v0)
    print(f"{'vector3dNorm(v0)':<{label_width}}: {v0_norm:.4f}")
    v0_normalized = m.vector3dNormalized(v0)
    print(f"{'vector3dNormalized(v0)':<{label_width}}: {_three(v0_normalized)}")
    v_cross = m.vector3dCross(v0, v1)
    print(f"{'vector3dCross(v0,v1)':<{label_width}}: {_three(v_cross)}")
    v_dot = m.vector3dDot(v0, v1)
    print(f"{'vector3dDot(v0,v1)':<{label_width}}: {v_dot:.4f}")

    print("\n--- rpy / rot ---")
    rpy = [179.46, 4.58, 166.62]
    print(f"{'rpy (deg)':<{label_width}}: {_three(rpy)}")
    rotation_x_axis, rotation_y_axis, rotation_z_axis = m.rpyToRot(rpy)
    print(f"{'rotation_x_axis':<{label_width}}: {_three(rotation_x_axis)}")
    print(f"{'rotation_y_axis':<{label_width}}: {_three(rotation_y_axis)}")
    print(f"{'rotation_z_axis':<{label_width}}: {_three(rotation_z_axis)}")
    rpy_back = m.rotToRpy(rotation_x_axis, rotation_y_axis, rotation_z_axis)
    print(f"{'rotToRpy(...)':<{label_width}}: {_three(rpy_back)}")

    axis, deg_aa = m.rpyToAxisAngle(rpy)
    print(f"{'rpyToAxisAngle':<{label_width}}: (deg | axis) = ({deg_aa:.0f}° | {axis[0]:.4f},{axis[1]:.4f},{axis[2]:.4f})")

    rpy_res = m.axisAngleToRpy(axis, 1.5)
    print(f"{'axisAngleToRpy(axis, 1.5)':<{label_width}}: (1.5°) -> {rpy_res[0]:.4f},{rpy_res[1]:.4f},{rpy_res[2]:.4f}")

    r3 = m.rpyDegToRotationMatrix(12.0, -34.0, 56.0)
    r3_rpy = m.rotationMatrixToRpyDeg(r3)
    r3_back = m.rpyDegToRotationMatrix(*r3_rpy)
    ok = np.allclose(r3, r3_back, atol=1e-9)
    print(f"{'rpyDeg <-> rotationMatrix check':<{label_width}}: {'ok' if ok else 'fail'}")

def main() -> None:
    parser = argparse.ArgumentParser(description="Pose6D / Vector3D / Euler and axis-angle demo or self-check")
    parser.add_argument(
        "--verify",
        action="store_true",
        help="run the built-in lightweight self-check and exit (0 pass, 1 fail)",
    )
    args = parser.parse_args()
    if args.verify:
        raise SystemExit(_run_verify())
    _run_demo()


if __name__ == "__main__":
    main()
