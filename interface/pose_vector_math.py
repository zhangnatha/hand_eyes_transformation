"""
@file pose_vector_math.py
@brief Pose6D 位姿与 Vector3D 三维向量、欧拉/轴角运算库

本模块提供了位姿变换、旋转矩阵转换、向量运算等功能，主要技术约定如下：

1. 数据类型约定：
   - Pose6D (Pose3D): 长度为 6 的序列 [x, y, z, rx, ry, rz]，其中旋转分量 (rx, ry, rz) 采用角度制 (Degrees)。
   - Vector3D (Vec3d): 长度为 3 的序列 [x, y, z]，代表三维向量或平移分量。

2. 旋转数学约定：
   - 支持 12 种轴序 × 2 种旋转方式（内旋/外旋），共 24 种旋转矩阵表示。
   - 默认轴序：RPYType.XYZ。
   - 默认旋转方式：ReferenceType.EXTRINSIC (外旋)。
   - 计算公式：默认情况下 R = Rz(rz) * Ry(ry) * Rx(rx)，即先绕固定轴 X 旋转，再 Y，最后 Z。

3. 坐标系类型：
   - INTRINSIC (内旋)：绕当前运动坐标系的轴旋转（动轴旋转）。
   - EXTRINSIC (外旋)：绕固定参考坐标系的轴旋转（定轴旋转）。

4. 兼容性说明：
   - 内部运算基于 NumPy 实现，支持输入 List、Tuple 或 np.ndarray。

@author zhangnatha
@email [zhangnatha1366560@gmail.com]
@date 2026-05-01
"""

from __future__ import annotations
import math
import sys
from enum import IntEnum
from typing import Sequence, Tuple, Union, List
import numpy as np

# 类型名定义
Vector3D = Union[Sequence[float], np.ndarray] # xyz 三元双精度向量
Pose6D = Union[Sequence[float], np.ndarray]   # xyz + rx,ry,rz（度）

class ReferenceType(IntEnum):
    """旋转参考坐标系类型"""
    INTRINSIC = 0  # 内旋：绕当前运动坐标系的轴旋转（动轴）
    EXTRINSIC = 1  # 外旋：绕固定参考坐标系的轴旋转（定轴）

class RPYType(IntEnum):
    """欧拉角/RPY 轴序枚举"""
    XYZ = 0; XZY = 1; YXZ = 2; YZX = 3; ZXY = 4; ZYX = 5
    XYX = 6; XZX = 7; YXY = 8; YZY = 9; ZXZ = 10; ZYZ = 11

class PoseVectorMath:
    """Pose6D 位姿与 Vector3D 三维向量、欧拉/轴角运算"""

    @staticmethod
    def _get_base_rot_matrix(axis: str, angle_rad: float) -> np.ndarray:
        """生成绕单一轴旋转的矩阵"""
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        if axis == 'X': return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)
        if axis == 'Y': return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
        if axis == 'Z': return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
        return np.eye(3)

    @staticmethod
    def rpyDegToRotationMatrix(r1_deg: float, r2_deg: float, r3_deg: float,
                               rpy_type: RPYType = RPYType.XYZ,
                               ref_type: ReferenceType = ReferenceType.EXTRINSIC) -> np.ndarray:
        """
        @brief 欧拉角 (度) → 旋转矩阵，XYZ + EXTRINSIC，R = Rz * Ry * Rx。
        @param[in] r1_deg,r2_deg,r3_deg 欧拉角分量 (度)。
        @param[in] rpy_type 轴序。
        @param[in] ref_type 内旋或外旋。
        @return 3×3 旋转矩阵。
        """
        rad = np.radians([r1_deg, r2_deg, r3_deg])
        order = rpy_type.name
        m1 = PoseVectorMath._get_base_rot_matrix(order[0], rad[0])
        m2 = PoseVectorMath._get_base_rot_matrix(order[1], rad[1])
        m3 = PoseVectorMath._get_base_rot_matrix(order[2], rad[2])

        if ref_type == ReferenceType.INTRINSIC:
            return m1 @ m2 @ m3 # 内旋：R = R1 * R2 * R3
        else:
            return m3 @ m2 @ m1 # 外旋：R = R3 * R2 * R1

    @staticmethod
    def rotationMatrixToRpyDeg(r: np.ndarray) -> np.ndarray:
        """
        @brief 旋转矩阵 → 欧拉角 (度)，XYZ + EXTRINSIC，R = Rz * Ry * Rx。
        @param[in] r 3×3 旋转矩阵。
        @return 欧拉角 (rx, ry, rz)，度。
        """
        r = np.asarray(r, dtype=float).reshape(3, 3)
        zer = 1e-8
        ry = math.atan2(-r[2, 0], math.sqrt(r[0, 0]**2 + r[1, 0]**2))
        if abs(ry - math.pi/2) < zer:
            ry, rx, rz = math.pi/2, math.atan2(r[0, 1], r[1, 1]), 0.0
        elif abs(ry + math.pi/2) < zer:
            ry, rx, rz = -math.pi/2, -math.atan2(r[0, 1], r[1, 1]), 0.0
        else:
            c = math.cos(ry)
            rz = math.atan2(r[1, 0] / c, r[0, 0] / c)
            rx = math.atan2(r[2, 1] / c, r[2, 2] / c)
        return np.degrees([rx, ry, rz])

    @staticmethod
    def pose3dToMat4(p: Pose6D) -> np.ndarray:
        """
        @brief 将位姿转为 4×4 齐次变换矩阵。
        @param[in] p 输入位姿，类型为 Pose6D。
        @return 齐次变换矩阵。
        """
        p = np.asarray(p, dtype=float).flatten()
        t = np.eye(4, dtype=float)
        t[:3, :3] = PoseVectorMath.rpyDegToRotationMatrix(p[3], p[4], p[5])
        t[:3, 3] = p[:3]
        return t

    @staticmethod
    def mat4ToPose(m: np.ndarray) -> np.ndarray:
        """
        @brief 将 4×4 齐次矩阵转为位姿。
        @param[in] m 齐次变换矩阵。
        @return 位姿 Pose6D。
        """
        m = np.asarray(m, dtype=float).reshape(4, 4)
        rpy = PoseVectorMath.rotationMatrixToRpyDeg(m[:3, :3])
        return np.concatenate([m[:3, 3], rpy])

    @staticmethod
    def pose3dMultiply(a: Pose6D, b: Pose6D) -> np.ndarray:
        """
        @brief 坐标变换：位姿 p1 经 p2 变换得到新位姿，等价于 T3 = T1 * T2。
        @param[in] a 第一个位姿（左侧），类型为 Pose6D。
        @param[in] b 第二个位姿（右侧），类型为 Pose6D。
        @return 复合后的位姿，类型为 Pose6D。
        """
        t = PoseVectorMath.pose3dToMat4(a) @ PoseVectorMath.pose3dToMat4(b)
        return PoseVectorMath.mat4ToPose(t)

    @staticmethod
    def pose3dInverse(p: Pose6D) -> np.ndarray:
        """
        @brief 坐标逆变换：求位姿的逆变换。
        @param[in] p 输入位姿，类型为 Pose6D，姿态按 Rz * Ry * Rx 与矩阵逆一致。
        @return 逆变换位姿，类型为 Pose6D。
        """
        t = PoseVectorMath.pose3dToMat4(p)
        inv = np.eye(4, dtype=float)
        inv[:3, :3] = t[:3, :3].T
        inv[:3, 3] = -inv[:3, :3] @ t[:3, 3]
        return PoseVectorMath.mat4ToPose(inv)

    @staticmethod
    def pose3dDistance(a: Pose6D, b: Pose6D) -> float:
        """
        @brief 计算两点之间的空间距离，姿态不参与计算。
        @param[in] a 点 1（取平移），类型为 Pose6D。
        @param[in] b 点 2（取平移），类型为 Pose6D。
        @return 欧氏距离。
        """
        diff = np.asarray(b)[:3] - np.asarray(a)[:3]
        return float(np.linalg.norm(diff))

    @staticmethod
    def pose3dOffset(a: Pose6D, b: Pose6D) -> np.ndarray:
        """
        @brief 计算两个位姿之间的偏移量（相对位姿 Ta^-1 * Tb）。
        @param[in] a 第一个位姿，类型为 Pose6D。
        @param[in] b 第二个位姿，类型为 Pose6D。
        @return 偏移位姿，类型为 Pose6D。
        """
        t_a_inv = np.linalg.inv(PoseVectorMath.pose3dToMat4(a))
        t_b = PoseVectorMath.pose3dToMat4(b)
        return PoseVectorMath.mat4ToPose(t_a_inv @ t_b)

    @staticmethod
    def pose3dAngle(p1: Pose6D, p2: Pose6D) -> Tuple[float, np.ndarray]:
        """
        @brief 计算两个位姿之间的轴角差（相对旋转）。
        @param[in] p1 第一个位姿，类型为 Pose6D。
        @param[in] p2 第二个位姿，类型为 Pose6D。
        @return 转角（度）与旋转轴单位向量，类型分别为 `float` 与 `Vector3D`。
        """
        rel = PoseVectorMath.pose3dOffset(p1, p2)
        r = PoseVectorMath.rpyDegToRotationMatrix(rel[3], rel[4], rel[5])
        cos_a = (np.trace(r) - 1.0) * 0.5
        angle = math.acos(max(-1.0, min(1.0, cos_a)))
        if angle < 1e-12: return 0.0, np.array([1.0, 0.0, 0.0])
        axis = np.array([r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1]])
        return math.degrees(angle), axis / np.linalg.norm(axis)

    @staticmethod
    def pose3dGetTrans(p: Pose6D) -> np.ndarray:
        """
        @brief 获得输入位姿的平移向量。
        @param[in] p 输入位姿，类型为 Pose6D。
        @return 平移向量，类型为 Vector3D。
        """
        return np.asarray(p)[:3].copy()

    @staticmethod
    def pose3dGetRpy(p: Pose6D) -> np.ndarray:
        """
        @brief 获得输入位姿的欧拉角描述（度）。
        @param[in] p 输入位姿，类型为 Pose6D。
        @return 欧拉角 (rx, ry, rz)，类型为 Vector3D。
        """
        return np.asarray(p)[3:6].copy()

    @staticmethod
    def vector3dNorm(v: Vector3D) -> float:
        """
        @brief 获得输入向量的模长。
        @param[in] v 输入三维向量，类型为 Vector3D。
        @return 模长。
        """
        return float(np.linalg.norm(v))

    @staticmethod
    def vector3dNormalized(v: Vector3D) -> np.ndarray:
        """
        @brief 对输入的向量进行归一化。
        @param[in] v 输入三维向量，类型为 Vector3D。
        @return 归一化后的向量，类型为 Vector3D。
        """
        v = np.asarray(v, dtype=float)
        n = np.linalg.norm(v)
        return v / n if n > 1e-15 else v.copy()

    @staticmethod
    def vector3dCross(a: Vector3D, b: Vector3D) -> np.ndarray:
        """
        @brief 两个向量叉乘。
        @param[in] a 第一个向量，类型为 Vector3D。
        @param[in] b 第二个向量，类型为 Vector3D。
        @return 叉乘结果，类型为 Vector3D。
        """
        return np.cross(a, b)

    @staticmethod
    def vector3dDot(a: Vector3D, b: Vector3D) -> float:
        """
        @brief 两个向量点乘。
        @param[in] a 第一个向量，类型为 Vector3D。
        @param[in] b 第二个向量，类型为 Vector3D。
        @return 点乘标量。
        """
        return float(np.dot(a, b))

    @staticmethod
    def rpyToRot(rpy_deg: Vector3D) -> List[np.ndarray]:
        """
        @brief 欧拉角转旋转矩阵的列（局部系三轴在世界系下的方向）。
        @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
        @return 分别为 X、Y、Z 轴方向向量，类型均为 Vector3D。
        """
        r = PoseVectorMath.rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2])
        return [r[:, 0], r[:, 1], r[:, 2]]

    @staticmethod
    def rotToRpy(v1: Vector3D, v2: Vector3D, v3: Vector3D) -> np.ndarray:
        """
        @brief 旋转矩阵转欧拉角描述（由三列轴向量构造旋转矩阵再逆解 RPY）。
        @param[in] v1 X 轴向量，类型为 Vector3D。
        @param[in] v2 Y 轴向量，类型为 Vector3D。
        @param[in] v3 Z 轴向量，类型为 Vector3D。
        @return 欧拉角 (度)，类型为 Vector3D。
        """
        r = np.column_stack([v1, v2, v3])
        # 简单校验正交性
        if abs(np.linalg.det(r) - 1.0) > 1e-3:
            raise ValueError("输入向量不满足正交规范约定")
        return PoseVectorMath.rotationMatrixToRpyDeg(r)

    @staticmethod
    def rpyToAxisAngle(rpy_deg: Vector3D) -> Tuple[np.ndarray, float]:
        """
        @brief 欧拉角转轴角。
        @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
        @return 单位旋转轴（Vector3D）与转角（度）。
        """
        r = PoseVectorMath.rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2])
        cos_a = (np.trace(r) - 1.0) * 0.5
        angle = math.acos(max(-1.0, min(1.0, cos_a)))
        if angle < 1e-12: return np.array([1.0, 0.0, 0.0]), 0.0
        axis = np.array([r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1]])
        return axis / np.linalg.norm(axis), math.degrees(angle)

    @staticmethod
    def axisAngleToRpy(axis: Vector3D, angle_deg: float) -> np.ndarray:
        """
        @brief 轴角转欧拉角。
        @param[in] axis 旋转轴，类型为 Vector3D（将被单位化）。
        @param[in] angle_deg 旋转角（度）。
        @return 欧拉角 (度)，类型为 Vector3D（RPY）。
        """
        axis = np.asarray(axis)
        n = np.linalg.norm(axis)
        if n < 1e-15: raise ValueError("转轴不能为零向量")

        # 罗德里格斯公式生成旋转矩阵
        axis = axis / n
        angle_rad = math.radians(angle_deg)
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        t = 1.0 - c
        x, y, z = axis
        r = np.array([[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
                      [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
                      [t*x*z-s*y, t*y*z+s*x, t*z*z+c]], dtype=float)
        return PoseVectorMath.rotationMatrixToRpyDeg(r)

    @staticmethod
    def poseToMat4(p: Pose6D) -> np.ndarray:
        """兼容性重定向"""
        return PoseVectorMath.pose3dToMat4(p)

# --- 模块级 API 别名导出，一遍调用更方便 ---
rpyDegToRotationMatrix = PoseVectorMath.rpyDegToRotationMatrix
rotationMatrixToRpyDeg = PoseVectorMath.rotationMatrixToRpyDeg
pose3dToMat4 = PoseVectorMath.pose3dToMat4
mat4ToPose = PoseVectorMath.mat4ToPose
pose3dMultiply = PoseVectorMath.pose3dMultiply
pose3dInverse = PoseVectorMath.pose3dInverse
pose3dDistance = PoseVectorMath.pose3dDistance
pose3dOffset = PoseVectorMath.pose3dOffset
pose3dAngle = PoseVectorMath.pose3dAngle
pose3dGetTrans = PoseVectorMath.pose3dGetTrans
pose3dGetRpy = PoseVectorMath.pose3dGetRpy
vector3dNorm = PoseVectorMath.vector3dNorm
vector3dNormalized = PoseVectorMath.vector3dNormalized
vector3dCross = PoseVectorMath.vector3dCross
vector3dDot = PoseVectorMath.vector3dDot
rpyToRot = PoseVectorMath.rpyToRot
rotToRpy = PoseVectorMath.rotToRpy
rpyToAxisAngle = PoseVectorMath.rpyToAxisAngle
axisAngleToRpy = PoseVectorMath.axisAngleToRpy
poseToMat4 = PoseVectorMath.poseToMat4

def run_verification():
    """库的功能自检逻辑"""
    p1 = [1.0, 2.0, 3.0, 10.0, 20.0, 30.0]
    off = pose3dOffset(p1, p1)
    success = np.allclose(pose3dToMat4(off), np.eye(4), atol=1e-6)
    print(f"自检验证结果: {'通过' if success else '失败'}")
    return success

if __name__ == "__main__":
    ok = run_verification()
    sys.exit(0 if ok else 1)