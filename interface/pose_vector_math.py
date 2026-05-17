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
    def _getBaseRotMatrix(rotation_axis: str, angle_rad: float) -> np.ndarray:
        """生成绕单一轴旋转的矩阵"""
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        if rotation_axis == 'X': return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)
        if rotation_axis == 'Y': return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
        if rotation_axis == 'Z': return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
        return np.eye(3)

    @staticmethod
    def rpyDegToRotationMatrix(rx_deg: float, ry_deg: float, rz_deg: float,
                               rpy_type: RPYType = RPYType.XYZ,
                               reference_type: ReferenceType = ReferenceType.EXTRINSIC) -> np.ndarray:
        """
        @brief 欧拉角 (度) → 旋转矩阵，XYZ + EXTRINSIC，R = Rz * Ry * Rx。
        @param[in] rx_deg,ry_deg,rz_deg 欧拉角分量 (度)。
        @param[in] rpy_type 轴序。
        @param[in] reference_type 内旋或外旋。
        @return 3×3 旋转矩阵。
        """
        rad = np.radians([rx_deg, ry_deg, rz_deg])
        order = rpy_type.name
        first_rotation = PoseVectorMath._getBaseRotMatrix(order[0], rad[0])
        second_rotation = PoseVectorMath._getBaseRotMatrix(order[1], rad[1])
        third_rotation = PoseVectorMath._getBaseRotMatrix(order[2], rad[2])

        if reference_type == ReferenceType.INTRINSIC:
            return first_rotation @ second_rotation @ third_rotation # 内旋：R = R1 * R2 * R3
        else:
            return third_rotation @ second_rotation @ first_rotation # 外旋：R = R3 * R2 * R1

    @staticmethod
    def rotationMatrixToRpyDeg(rotation_matrix: np.ndarray) -> np.ndarray:
        """
        @brief 旋转矩阵 → 欧拉角 (度)，XYZ + EXTRINSIC，R = Rz * Ry * Rx。
        @param[in] rotation_matrix 3×3 旋转矩阵。
        @return 欧拉角 (rx, ry, rz)，度。
        """
        rotation_matrix = np.asarray(rotation_matrix, dtype=float).reshape(3, 3)
        zer = 1e-8
        ry = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2))
        if abs(ry - math.pi/2) < zer:
            ry, rx, rz = math.pi/2, math.atan2(rotation_matrix[0, 1], rotation_matrix[1, 1]), 0.0
        elif abs(ry + math.pi/2) < zer:
            ry, rx, rz = -math.pi/2, -math.atan2(rotation_matrix[0, 1], rotation_matrix[1, 1]), 0.0
        else:
            c = math.cos(ry)
            rz = math.atan2(rotation_matrix[1, 0] / c, rotation_matrix[0, 0] / c)
            rx = math.atan2(rotation_matrix[2, 1] / c, rotation_matrix[2, 2] / c)
        return np.degrees([rx, ry, rz])

    @staticmethod
    def pose6dToHomogeneous(pose6d: Pose6D) -> np.ndarray:
        """
        @brief 将 6D 位姿转换为 4×4 齐次变换矩阵。
        @param[in] pose6d 输入位姿 [x, y, z, rx, ry, rz]，角度为度。
        @return 齐次变换矩阵。
        """
        x, y, z, rx, ry, rz = np.asarray(pose6d, dtype=float).flatten()

        homogeneous = np.eye(4, dtype=float)
        homogeneous[:3, :3] = PoseVectorMath.rpyDegToRotationMatrix(rx, ry, rz)
        homogeneous[:3, 3] = [x, y, z]
        return homogeneous

    @staticmethod
    def homogeneousToPose6d(homogeneous: np.ndarray) -> np.ndarray:
        """
        @brief 将 4×4 齐次变换矩阵转换为 6D 位姿。
        @param[in] homogeneous 齐次变换矩阵。
        @return 位姿 [x, y, z, rx, ry, rz]，角度为度。
        """
        homogeneous = np.asarray(homogeneous, dtype=float).reshape(4, 4)
        translation = homogeneous[:3, 3]
        euler_angles = PoseVectorMath.rotationMatrixToRpyDeg(homogeneous[:3, :3])
        return np.concatenate([translation, euler_angles])

    @staticmethod
    def pose3dMultiply(pose_a_in_b: Pose6D, pose_target_in_a: Pose6D) -> np.ndarray:
        """
        @brief 最小正向乘运算：计算 target 在 B 坐标系下的位姿。
        @details 数学关系：T_target2B = T_A2B * T_target2A。
        @param[in] pose_a_in_b A 坐标系相对于 B 坐标系的 6D 位姿。
        @param[in] pose_target_in_a target 在 A 坐标系下的 6D 位姿。
        @return target 在 B 坐标系下的 6D 位姿。
        """
        T_A2B = PoseVectorMath.pose6dToHomogeneous(pose_a_in_b)
        T_target2A = PoseVectorMath.pose6dToHomogeneous(pose_target_in_a)
        T_target2B = T_A2B @ T_target2A

        return PoseVectorMath.homogeneousToPose6d(T_target2B)

    @staticmethod
    def pose3dForceZDown(pose6d: Pose6D) -> np.ndarray:
        """
        @brief 将输入位姿的姿态重构为 Z 轴朝下，平移保持不变。
        @param[in] pose6d 输入位姿，类型为 Pose6D。
        @return Z 轴朝下后的 6D 位姿。
        """
        homogeneous = PoseVectorMath.pose6dToHomogeneous(pose6d)
        target_axis_x = homogeneous[:3, 0]
        target_new_z = np.array([0.0, 0.0, -1.0])
        target_new_y = np.cross(target_new_z, target_axis_x)
        target_new_x = np.cross(target_new_y, target_new_z)
        homogeneous[:3, :3] = np.column_stack([target_new_x, target_new_y, target_new_z])
        return PoseVectorMath.homogeneousToPose6d(homogeneous)

    @staticmethod
    def pose3dInverse(pose_a_in_b: Pose6D) -> np.ndarray:
        """
        @brief 最小逆运算：计算 B 坐标系相对于 A 坐标系的 6D 位姿。
        @details 数学关系：T_B2A = inv(T_A2B)。
        @param[in] pose_a_in_b A 坐标系相对于 B 坐标系的 6D 位姿。
        @return B 坐标系相对于 A 坐标系的 6D 位姿。
        """
        T_A2B = PoseVectorMath.pose6dToHomogeneous(pose_a_in_b)
        T_B2A = np.linalg.inv(T_A2B)
        return PoseVectorMath.homogeneousToPose6d(T_B2A)

    @staticmethod
    def pose3dDistance(start_pose6d: Pose6D, end_pose6d: Pose6D) -> float:
        """
        @brief 计算两点之间的空间距离，姿态不参与计算。
        @param[in] start_pose6d 点 1（取平移），类型为 Pose6D。
        @param[in] end_pose6d 点 2（取平移），类型为 Pose6D。
        @return 欧氏距离。
        """
        diff = np.asarray(end_pose6d)[:3] - np.asarray(start_pose6d)[:3]
        return float(np.linalg.norm(diff))

    @staticmethod
    def pose3dOffset(pose_from_base_to_a: Pose6D, pose_from_base_to_b: Pose6D) -> np.ndarray:
        """
        @brief 计算两个位姿之间的偏移量（相对位姿 Ta^-1 * Tb）。
        @param[in] pose_from_base_to_a 第一个位姿，类型为 Pose6D。
        @param[in] pose_from_base_to_b 第二个位姿，类型为 Pose6D。
        @return 偏移位姿，类型为 Pose6D。
        """
        return PoseVectorMath.pose3dMultiply(PoseVectorMath.pose3dInverse(pose_from_base_to_a), pose_from_base_to_b)

    @staticmethod
    def pose3dAngle(pose_from_base_to_a: Pose6D, pose_from_base_to_b: Pose6D) -> Tuple[float, np.ndarray]:
        """
        @brief 计算两个位姿之间的轴角差（相对旋转）。
        @param[in] pose_from_base_to_a 第一个位姿，类型为 Pose6D。
        @param[in] pose_from_base_to_b 第二个位姿，类型为 Pose6D。
        @return 转角（度）与旋转轴单位向量，类型分别为 `float` 与 `Vector3D`。
        """
        relative_pose_a_to_b = PoseVectorMath.pose3dOffset(pose_from_base_to_a, pose_from_base_to_b)
        relative_rotation = PoseVectorMath.rpyDegToRotationMatrix(relative_pose_a_to_b[3], relative_pose_a_to_b[4], relative_pose_a_to_b[5])
        cos_a = (np.trace(relative_rotation) - 1.0) * 0.5
        angle = math.acos(max(-1.0, min(1.0, cos_a)))
        if angle < 1e-12: return 0.0, np.array([1.0, 0.0, 0.0])
        axis = np.array([
            relative_rotation[2,1] - relative_rotation[1,2],
            relative_rotation[0,2] - relative_rotation[2,0],
            relative_rotation[1,0] - relative_rotation[0,1],
        ])
        return math.degrees(angle), axis / np.linalg.norm(axis)

    @staticmethod
    def pose3dGetTrans(pose6d: Pose6D) -> np.ndarray:
        """
        @brief 获得输入位姿的平移向量。
        @param[in] pose6d 输入位姿，类型为 Pose6D。
        @return 平移向量，类型为 Vector3D。
        """
        return np.asarray(pose6d)[:3].copy()

    @staticmethod
    def pose3dGetRpy(pose6d: Pose6D) -> np.ndarray:
        """
        @brief 获得输入位姿的欧拉角描述（度）。
        @param[in] pose6d 输入位姿，类型为 Pose6D。
        @return 欧拉角 (rx, ry, rz)，类型为 Vector3D。
        """
        return np.asarray(pose6d)[3:6].copy()

    @staticmethod
    def vector3dNorm(vector3d: Vector3D) -> float:
        """
        @brief 获得输入向量的模长。
        @param[in] vector3d 输入三维向量，类型为 Vector3D。
        @return 模长。
        """
        return float(np.linalg.norm(vector3d))

    @staticmethod
    def vector3dNormalized(vector3d: Vector3D) -> np.ndarray:
        """
        @brief 对输入的向量进行归一化。
        @param[in] vector3d 输入三维向量，类型为 Vector3D。
        @return 归一化后的向量，类型为 Vector3D。
        """
        vector3d = np.asarray(vector3d, dtype=float)
        vector_norm = np.linalg.norm(vector3d)
        return vector3d / vector_norm if vector_norm > 1e-15 else vector3d.copy()

    @staticmethod
    def vector3dCross(left_vector: Vector3D, right_vector: Vector3D) -> np.ndarray:
        """
        @brief 两个向量叉乘。
        @param[in] left_vector 第一个向量，类型为 Vector3D。
        @param[in] right_vector 第二个向量，类型为 Vector3D。
        @return 叉乘结果，类型为 Vector3D。
        """
        return np.cross(left_vector, right_vector)

    @staticmethod
    def vector3dDot(left_vector: Vector3D, right_vector: Vector3D) -> float:
        """
        @brief 两个向量点乘。
        @param[in] left_vector 第一个向量，类型为 Vector3D。
        @param[in] right_vector 第二个向量，类型为 Vector3D。
        @return 点乘标量。
        """
        return float(np.dot(left_vector, right_vector))

    @staticmethod
    def rpyToRot(rpy_deg: Vector3D) -> List[np.ndarray]:
        """
        @brief 欧拉角转旋转矩阵的列（局部系三轴在世界系下的方向）。
        @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
        @return 分别为 X、Y、Z 轴方向向量，类型均为 Vector3D。
        """
        rotation_matrix = PoseVectorMath.rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2])
        return [rotation_matrix[:, 0], rotation_matrix[:, 1], rotation_matrix[:, 2]]

    @staticmethod
    def rotToRpy(x_axis: Vector3D, y_axis: Vector3D, z_axis: Vector3D) -> np.ndarray:
        """
        @brief 旋转矩阵转欧拉角描述（由三列轴向量构造旋转矩阵再逆解 RPY）。
        @param[in] x_axis X 轴向量，类型为 Vector3D。
        @param[in] y_axis Y 轴向量，类型为 Vector3D。
        @param[in] z_axis Z 轴向量，类型为 Vector3D。
        @return 欧拉角 (度)，类型为 Vector3D。
        """
        rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
        # 简单校验正交性
        if abs(np.linalg.det(rotation_matrix) - 1.0) > 1e-3:
            raise ValueError("Input vectors do not satisfy the orthonormal rotation convention")
        return PoseVectorMath.rotationMatrixToRpyDeg(rotation_matrix)

    @staticmethod
    def rpyToAxisAngle(rpy_deg: Vector3D) -> Tuple[np.ndarray, float]:
        """
        @brief 欧拉角转轴角。
        @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
        @return 单位旋转轴（Vector3D）与转角（度）。
        """
        rotation_matrix = PoseVectorMath.rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2])
        cos_a = (np.trace(rotation_matrix) - 1.0) * 0.5
        angle = math.acos(max(-1.0, min(1.0, cos_a)))
        if angle < 1e-12: return np.array([1.0, 0.0, 0.0]), 0.0
        axis = np.array([
            rotation_matrix[2,1] - rotation_matrix[1,2],
            rotation_matrix[0,2] - rotation_matrix[2,0],
            rotation_matrix[1,0] - rotation_matrix[0,1],
        ])
        return axis / np.linalg.norm(axis), math.degrees(angle)

    @staticmethod
    def axisAngleToRpy(rotation_axis: Vector3D, angle_deg: float) -> np.ndarray:
        """
        @brief 轴角转欧拉角。
        @param[in] rotation_axis 旋转轴，类型为 Vector3D（将被单位化）。
        @param[in] angle_deg 旋转角（度）。
        @return 欧拉角 (度)，类型为 Vector3D（RPY）。
        """
        rotation_axis = np.asarray(rotation_axis)
        axis_norm = np.linalg.norm(rotation_axis)
        if axis_norm < 1e-15: raise ValueError("Rotation axis cannot be a zero vector")

        # 罗德里格斯公式生成旋转矩阵
        rotation_axis = rotation_axis / axis_norm
        angle_rad = math.radians(angle_deg)
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        t = 1.0 - c
        x, y, z = rotation_axis
        rotation_matrix = np.array([[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
                                    [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
                                    [t*x*z-s*y, t*y*z+s*x, t*z*z+c]], dtype=float)
        return PoseVectorMath.rotationMatrixToRpyDeg(rotation_matrix)

# --- 模块级 API 别名导出，一遍调用更方便 ---
rpyDegToRotationMatrix = PoseVectorMath.rpyDegToRotationMatrix
rotationMatrixToRpyDeg = PoseVectorMath.rotationMatrixToRpyDeg
pose6dToHomogeneous = PoseVectorMath.pose6dToHomogeneous
homogeneousToPose6d = PoseVectorMath.homogeneousToPose6d
pose3dMultiply = PoseVectorMath.pose3dMultiply
pose3dForceZDown = PoseVectorMath.pose3dForceZDown
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

def runVerification():
    """库的功能自检逻辑"""
    test_pose6d = [1.0, 2.0, 3.0, 10.0, 20.0, 30.0]
    identity_offset = pose3dOffset(test_pose6d, test_pose6d)
    success = np.allclose(pose6dToHomogeneous(identity_offset), np.eye(4), atol=1e-6)
    print(f"verification result: {'pass' if success else 'fail'}")
    return success

if __name__ == "__main__":
    ok = runVerification()
    sys.exit(0 if ok else 1)
