#!/usr/bin/env python3
"""仓库根入口：默认打印演示结果；`--verify` 运行轻量自检（退出码 0/1）。

将 `interface/` 加入模块路径后使用 `pose_vector_math` 模块。
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
    from pose_vector_math import run_verification  # noqa: E402

    return 0 if run_verification() else 1


def _run_demo() -> None:
    """与 `main.cpp` 同流程、同数值的演示输出。"""
    import numpy as np
    from pose_vector_math import PoseVectorMath

    def _six(p, prec: int = 4) -> str:
        p = np.asarray(p, dtype=float).flatten()
        return ",".join(f"{x:.{prec}f}" for x in p)

    def _three(v, prec: int = 4) -> str:
        v = np.asarray(v, dtype=float).flatten()
        return ",".join(f"{x:.{prec}f}" for x in v)

    m = PoseVectorMath()
    L_W = 48  # 打印标签宽度

    target_in_camera = [130.51, 27.86, 515.003, 179.46, 4.58, 166.62]

    print("--- eyes to hand (ETH) ---")
    camera_in_base = [-687.029, -412.132, -406.297, -44.9606, -33.0434, -1.92313]
    target_in_base_eth = m.pose3dMultiply(camera_in_base, target_in_camera)
    print(f"{'target_in_base[ETH]':<{L_W}}: {_six(target_in_base_eth)}")

    print("\n--- eyes in hand (EIH) ---")
    camera_in_flange = [206.9900, -331.5936, -426.7618, 177.1207, 1.5365, 101.0783]
    flange_in_base = [566.417, 55.603, -78.360, -180.0, 0.0, 147.16]
    target_in_flange = m.pose3dMultiply(camera_in_flange, target_in_camera)
    target_in_base_eih = m.pose3dMultiply(flange_in_base, target_in_flange)
    print(f"{'target_in_base[EIH]':<{L_W}}: {_six(target_in_base_eih)}")

    p1 = target_in_camera
    p2 = camera_in_base

    print("\n--- pose API (p1=target_in_camera, p2=camera_in_base) ---")
    print(f"{'pose3d_inverse(p1)':<{L_W}}: {_six(m.pose3dInverse(p1))}")
    print(f"{'pose3d_distance(p1, p2)':<{L_W}}: {m.pose3dDistance(p1, p2):.4f}")
    print(f"{'pose3d_offset(p1, p2)':<{L_W}}: {_six(m.pose3dOffset(p1, p2))}")
    ang, ax = m.pose3dAngle(p1, p2)

    print(f"{'pose3d_angle(p1, p2)':<{L_W}}: {ang:.4f}° | {ax[0]:.4f},{ax[1]:.4f},{ax[2]:.4f}")
    print(f"{'pose3d_get_trans(p1)':<{L_W}}: {_three(m.pose3dGetTrans(p1))}")
    print(f"{'pose3d_get_rpy(p1)':<{L_W}}: {_three(m.pose3dGetRpy(p1))}")

    print("\n--- pose3d_to_mat4(p1) ---")
    print(f"{'Input p1':<{L_W}}: {_six(p1)}")
    t = m.pose3dToMat4(p1)
    for i in range(4):
        print("  ", "  ".join(f"{float(t[i, j]):14.8f}" for j in range(4)))

    print("\n--- vector ---")
    v0 = [130.51, 27.86, 515.003]
    v1 = [566.417, 55.603, -78.360]
    print(f"{'v0':<{L_W}}: {_three(v0)}")
    print(f"{'v1':<{L_W}}: {_three(v1)}")
    print(f"{'vector3d_norm(v0)':<{L_W}}: {m.vector3dNorm(v0):.4f}")
    print(f"{'vector3d_normalized(v0)':<{L_W}}: {_three(m.vector3dNormalized(v0))}")
    print(f"{'vector3d_cross(v0,v1)':<{L_W}}: {_three(m.vector3dCross(v0, v1))}")
    print(f"{'vector3d_dot(v0,v1)':<{L_W}}: {m.vector3dDot(v0, v1):.4f}")

    print("\n--- rpy / rot ---")
    rpy = [179.46, 4.58, 166.62]
    print(f"{'rpy (deg)':<{L_W}}: {_three(rpy)}")
    rotation_x_axies, rotation_y_axies, rotation_z_axies = m.rpyToRot(rpy)
    print(f"{'rotation_x_axies':<{L_W}}: {_three(rotation_x_axies)}")
    print(f"{'rotation_y_axies':<{L_W}}: {_three(rotation_y_axies)}")
    print(f"{'rotation_z_axies':<{L_W}}: {_three(rotation_z_axies)}")
    rpy_back = m.rotToRpy(rotation_x_axies, rotation_y_axies, rotation_z_axies)
    print(f"{'rot_to_rpy(...)':<{L_W}}: {_three(rpy_back)}")

    axis, deg_aa = m.rpyToAxisAngle(rpy)
    print(f"{'rpy_to_axis_angle':<{L_W}}: (deg | axis) = ({deg_aa:.0f}° | {axis[0]:.4f},{axis[1]:.4f},{axis[2]:.4f})")

    rpy_res = m.axisAngleToRpy(axis, 1.5)
    print(f"{'axis_angle_to_rpy(axis, 1.5)':<{L_W}}: (1.5°) -> {rpy_res[0]:.4f},{rpy_res[1]:.4f},{rpy_res[2]:.4f}")

    # 自检
    r3 = m.rpyDegToRotationMatrix(12.0, -34.0, 56.0)
    ok = np.allclose(r3, m.rpyDegToRotationMatrix(*m.rotationMatrixToRpyDeg(r3)), atol=1e-9)
    print(f"{'rpyDeg <-> rotationMatrix check':<{L_W}}: {'ok' if ok else 'fail'}")

def main() -> None:
    parser = argparse.ArgumentParser(description="Pose6D / Vector3D / 欧拉与轴角演示或自检")
    parser.add_argument(
        "--verify",
        action="store_true",
        help="运行库内置轻量自检后退出（0 通过，1 失败）",
    )
    args = parser.parse_args()
    if args.verify:
        raise SystemExit(_run_verify())
    _run_demo()


if __name__ == "__main__":
    main()
