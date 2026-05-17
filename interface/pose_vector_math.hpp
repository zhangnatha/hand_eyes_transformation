#pragma once

/**
 * @file pose_vector_math.hpp
 * @brief Pose6D 位姿与 Vector3D 三维向量、欧拉/轴角运算库
 * * 本库提供了六自由度位姿变换、旋转矩阵转换及三维向量运算功能。
 *
 * * 1. 数据结构约定：
 * - Pose6D: 定义为 std::array<double, 6>，顺序为 [x, y, z, rx, ry, rz]。
 * - Vector3D: 采用 Eigen::Vector3d，表示三维空间点或方向。
 * - 角度单位：所有欧拉角分量 (rx, ry, rz) 均使用**度 (Degrees)**。
 *
 * * 2. 旋转数学约定：
 * - 支持 12 种标准轴序 × 2 种旋转方式（内/外旋），共 24 种组合。
 * - 默认配置：轴序为 RPYType::XYZ，旋转方式为 ReferenceType::EXTRINSIC (外旋)。
 * - 默认矩阵合成：R = Rz(rz) * Ry(ry) * Rx(rx)。
 *
 * * 3. 坐标系定义：
 * - INTRINSIC (内旋)：绕当前运动坐标系的轴旋转（动轴旋转）。
 * - EXTRINSIC (外旋)：绕固定参考坐标系的轴旋转（定轴旋转）。
 *
 * * 4. 跨平台一致性：
 * - 依赖于 Eigen 3 库进行高效率线性代数运算。
 *
 * @author zhangnatha
 * @email [zhangnatha1366560@gmail.com]
 * @date 2026-05-01
 *
 * 各公开接口的数学式与示意流程见仓库根目录下 `assets/API.md`。
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <string>

namespace pose_vector_math {

namespace custom {
inline constexpr double _pi() { return 3.14159265358979323846; }
}  // namespace custom

/** @brief 旋转参考坐标系类型 */
enum class ReferenceType {
    INTRINSIC = 0,  ///< 内旋：绕当前运动坐标系的轴旋转（动轴）
    EXTRINSIC = 1   ///< 外旋：绕固定参考坐标系的轴旋转（定轴）
};

/** @brief 欧拉角/RPY 轴序枚举 */
enum class RPYType {
    XYZ = 0, XZY = 1, YXZ = 2, YZX = 3, ZXY = 4, ZYX = 5,
    XYX = 6, XZX = 7, YXY = 8, YZY = 9, ZXZ = 10, ZYZ = 11
};

/**
 * @brief 三维向量类型。
 *
 * 表示欧氏空间中的三维点或方向，三个双精度分量 (x, y, z)。
 */
using Pose6D = std::array<double, 6>;                ///< xyz + rx,ry,rz（度）

/**
 * @brief 六自由度位姿类型
 *
 * 顺序为平移 (x, y, z) 与欧拉角 RPY (rx, ry, rz)，角度单位为**度**，默认欧拉角旋转约定
 *（XYZ + EXTRINSIC，即 \f$R = R_z R_y R_x\f$）。
 */
using Vector3D  = Eigen::Vector3d;                      ///< xyz 三元双精度向量


/**
 * @brief 生成绕单一轴旋转的矩阵
 * @param[in] rotation_axis 轴序字符，'X'、'Y'、'Z'。
 * @param[in] angle_rad 旋转角度，弧度。
 * @return 3×3 旋转矩阵。
 */
inline Eigen::Matrix3d _getBaseRotMatrix(char rotation_axis, double angle_rad) {
    if (rotation_axis == 'X') return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
    if (rotation_axis == 'Y') return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    if (rotation_axis == 'Z') return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return Eigen::Matrix3d::Identity();
}

/**
 * @brief 获取 RPYType 对应的字符串描述
 * @param[in] rpy_type 轴序枚举。
 * @return 字符串描述。
 */
inline std::string _getRpyOrderStr(RPYType rpy_type) {
    const char* names[] = {"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX", 
                           "XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ"};
    return names[static_cast<int>(rpy_type)];
}

/**
 * @brief 欧拉角 (度) → 旋转矩阵，XYZ + EXTRINSIC，\f$R = R_z R_y R_x\f$。
 * @param[in] rx_deg,ry_deg,rz_deg 欧拉角分量 (度)。
 * @param[in] rpy_type 轴序。
 * @param[in] reference_type 内旋或外旋。
 * @return 3×3 旋转矩阵。
 */
inline Eigen::Matrix3d rpyDegToRotationMatrix(
    double rx_deg, double ry_deg, double rz_deg,
    RPYType rpy_type = RPYType::XYZ,
    ReferenceType reference_type = ReferenceType::EXTRINSIC)
{
    Vector3D rads = Vector3D(rx_deg, ry_deg, rz_deg) * custom::_pi() / 180.0;
    std::string order = _getRpyOrderStr(rpy_type);

    Eigen::Matrix3d first_rotation = _getBaseRotMatrix(order[0], rads[0]);
    Eigen::Matrix3d second_rotation = _getBaseRotMatrix(order[1], rads[1]);
    Eigen::Matrix3d third_rotation = _getBaseRotMatrix(order[2], rads[2]);

    if (reference_type == ReferenceType::INTRINSIC) {
        return first_rotation * second_rotation * third_rotation; // 内旋：R = R1 * R2 * R3
    } else {
        return third_rotation * second_rotation * first_rotation; // 外旋：R = R3 * R2 * R1
    }
}

/**
 * @brief 旋转矩阵 → 欧拉角 (度)，XYZ + EXTRINSIC，\f$R = R_z R_y R_x\f$。
 * @param[in] rotation_matrix 3×3 旋转矩阵。
 * @return 欧拉角 (rx, ry, rz)，度。
 */
inline Vector3D rotationMatrixToRpyDeg(const Eigen::Matrix3d& rotation_matrix) {
    const double zer = 1e-8;
    double ry = std::atan2(
        -rotation_matrix(2, 0),
        std::sqrt(rotation_matrix(0, 0) * rotation_matrix(0, 0) + rotation_matrix(1, 0) * rotation_matrix(1, 0)));
    double rx, rz;
    if (std::abs(ry - custom::_pi() / 2.0) < zer) {
        ry = custom::_pi() / 2.0;
        rx = std::atan2(rotation_matrix(0, 1), rotation_matrix(1, 1));
        rz = 0;
    } else if (std::abs(ry + custom::_pi() / 2.0) < zer) {
        ry = -custom::_pi() / 2.0;
        rx = -std::atan2(rotation_matrix(0, 1), rotation_matrix(1, 1));
        rz = 0;
    } else {
        const double c = std::cos(ry);
        rz = std::atan2(rotation_matrix(1, 0) / c, rotation_matrix(0, 0) / c);
        rx = std::atan2(rotation_matrix(2, 1) / c, rotation_matrix(2, 2) / c);
    }
    return Vector3D(rx, ry, rz) * 180.0 / custom::_pi();
}

/**
 * @brief 将 6D 位姿转换为 4×4 齐次变换矩阵。
 * @param[in] pose6d 输入位姿 [x, y, z, rx, ry, rz]，角度为度。
 * @return 齐次变换矩阵。
 */
inline Eigen::Matrix4d pose6dToHomogeneous(const Pose6D& pose6d) {
    const double x = pose6d[0];
    const double y = pose6d[1];
    const double z = pose6d[2];
    const double rx = pose6d[3];
    const double ry = pose6d[4];
    const double rz = pose6d[5];

    Eigen::Matrix4d homogeneous = Eigen::Matrix4d::Identity();
    homogeneous.block<3, 3>(0, 0) = rpyDegToRotationMatrix(rx, ry, rz);
    homogeneous.block<3, 1>(0, 3) = Vector3D(x, y, z);
    return homogeneous;
}

/**
 * @brief 将 4×4 齐次变换矩阵转换为 6D 位姿。
 * @param[in] homogeneous 齐次变换矩阵。
 * @return 位姿 [x, y, z, rx, ry, rz]，角度为度。
 */
inline Pose6D homogeneousToPose6d(const Eigen::Matrix4d& homogeneous) {
    const Vector3D translation = homogeneous.block<3, 1>(0, 3);
    const Vector3D euler_angles = rotationMatrixToRpyDeg(homogeneous.block<3, 3>(0, 0));
    return Pose6D{{
        translation[0], translation[1], translation[2],
        euler_angles[0], euler_angles[1], euler_angles[2]
    }};
}

/**
 * @brief 最小正向乘运算：计算 target 在 B 坐标系下的位姿。
 * @details 数学关系：\f$T_{target2B}=T_{A2B}T_{target2A}\f$。
 * @param[in] pose_a_in_b A 坐标系相对于 B 坐标系的 6D 位姿。
 * @param[in] pose_target_in_a target 在 A 坐标系下的 6D 位姿。
 * @return target 在 B 坐标系下的 6D 位姿。
 */
inline Pose6D pose3dMultiply(
    const Pose6D& pose_a_in_b,
    const Pose6D& pose_target_in_a)
{
    Eigen::Matrix4d T_A2B = pose6dToHomogeneous(pose_a_in_b);
    Eigen::Matrix4d T_target2A = pose6dToHomogeneous(pose_target_in_a);
    Eigen::Matrix4d T_target2B = T_A2B * T_target2A;

    return homogeneousToPose6d(T_target2B);
}

/**
 * @brief 将输入位姿的姿态重构为 Z 轴朝下，平移保持不变。
 * @param[in] pose6d 输入位姿，类型为 Pose6D。
 * @return Z 轴朝下后的 6D 位姿。
 */
inline Pose6D pose3dForceZDown(const Pose6D& pose6d) {
    Eigen::Matrix4d homogeneous = pose6dToHomogeneous(pose6d);
    const Vector3D target_axis_x = homogeneous.block<3, 1>(0, 0);
    const Vector3D target_new_z(0.0, 0.0, -1.0);
    const Vector3D target_new_y = target_new_z.cross(target_axis_x);
    const Vector3D target_new_x = target_new_y.cross(target_new_z);
    homogeneous.block<3, 1>(0, 0) = target_new_x;
    homogeneous.block<3, 1>(0, 1) = target_new_y;
    homogeneous.block<3, 1>(0, 2) = target_new_z;
    return homogeneousToPose6d(homogeneous);
}

/**
 * @brief 最小逆运算：计算 B 坐标系相对于 A 坐标系的 6D 位姿。
 * @details 数学关系：\f$T_{B2A}=T_{A2B}^{-1}\f$。
 * @param[in] pose_a_in_b A 坐标系相对于 B 坐标系的 6D 位姿。
 * @return B 坐标系相对于 A 坐标系的 6D 位姿。
 */
inline Pose6D pose3dInverse(const Pose6D& pose_a_in_b) {
    const Eigen::Matrix4d T_A2B = pose6dToHomogeneous(pose_a_in_b);
    const Eigen::Matrix4d T_B2A = T_A2B.inverse();
    return homogeneousToPose6d(T_B2A);
}

/**
 * @brief 计算两点之间的空间距离，姿态不参与计算。
 * @param[in] start_pose6d 点 1（取平移），类型为 Pose6D。
 * @param[in] end_pose6d 点 2（取平移），类型为 Pose6D。
 * @return 欧氏距离。
 */
inline double pose3dDistance(const Pose6D& start_pose6d, const Pose6D& end_pose6d) {
    return (Vector3D(end_pose6d[0], end_pose6d[1], end_pose6d[2]) -
            Vector3D(start_pose6d[0], start_pose6d[1], start_pose6d[2])).norm();
}

/**
 * @brief 计算两个位姿之间的偏移量（相对位姿 \f$T_a^{-1} T_b\f$）。
 * @param[in] pose_from_base_to_a 第一个位姿，类型为 Pose6D。
 * @param[in] pose_from_base_to_b 第二个位姿，类型为 Pose6D。
 * @return 偏移位姿，类型为 Pose6D。
 */
inline Pose6D pose3dOffset(const Pose6D& pose_from_base_to_a, const Pose6D& pose_from_base_to_b) {
    return pose3dMultiply(pose3dInverse(pose_from_base_to_a), pose_from_base_to_b);
}

/**
 * @brief 计算两个位姿之间的轴角差（相对旋转）。
 * @param[in] pose_from_base_to_a 第一个位姿，类型为 Pose6D。
 * @param[in] pose_from_base_to_b 第二个位姿，类型为 Pose6D。
 * @return 转角（度）与旋转轴单位向量，类型分别为 `double` 与 `Vector3D`。
 */
inline std::pair<double, Vector3D> pose3dAngle(const Pose6D& pose_from_base_to_a, const Pose6D& pose_from_base_to_b) {
    const Pose6D relative_pose_a_to_b = pose3dOffset(pose_from_base_to_a, pose_from_base_to_b);
    Eigen::Matrix3d relative_rotation = rpyDegToRotationMatrix(relative_pose_a_to_b[3], relative_pose_a_to_b[4], relative_pose_a_to_b[5]);
    Eigen::AngleAxisd axis_angle(relative_rotation);
    return std::make_pair(axis_angle.angle() * 180.0 / custom::_pi(), Vector3D(axis_angle.axis()));
}

/**
 * @brief 获得输入位姿的平移向量。
 * @param[in] pose6d 输入位姿，类型为 Pose6D。
 * @return 平移向量，类型为 Vector3D。
 */
inline Vector3D pose3dGetTrans(const Pose6D& pose6d) { return Vector3D(pose6d[0], pose6d[1], pose6d[2]); }

/**
 * @brief 获得输入位姿的欧拉角描述（度）。
 * @param[in] pose6d 输入位姿，类型为 Pose6D。
 * @return 欧拉角 (rx, ry, rz)，类型为 Vector3D。
 */
inline Vector3D pose3dGetRpy(const Pose6D& pose6d) { return Vector3D(pose6d[3], pose6d[4], pose6d[5]); }

/**
 * @brief 获得输入向量的模长。
 * @param[in] vector3d 输入三维向量，类型为 Vector3D。
 * @return 模长。
 */
inline double vector3dNorm(const Vector3D& vector3d) { return vector3d.norm(); }

/**
 * @brief 对输入的向量进行归一化。
 * @param[in] vector3d 输入三维向量，类型为 Vector3D。
 * @return 归一化后的向量，类型为 Vector3D。
 */
inline Vector3D vector3dNormalized(const Vector3D& vector3d) {
    double vector_norm = vector3d.norm();
    return (vector_norm < 1e-15) ? vector3d : (vector3d / vector_norm);
}

/**
 * @brief 两个向量叉乘。
 * @param[in] left_vector 第一个向量，类型为 Vector3D。
 * @param[in] right_vector 第二个向量，类型为 Vector3D。
 * @return 叉乘结果，类型为 Vector3D。
 */
inline Vector3D vector3dCross(const Vector3D& left_vector, const Vector3D& right_vector) { return left_vector.cross(right_vector); }

/**
 * @brief 两个向量点乘。
 * @param[in] left_vector 第一个向量，类型为 Vector3D。
 * @param[in] right_vector 第二个向量，类型为 Vector3D。
 * @return 点乘标量。
 */
inline double vector3dDot(const Vector3D& left_vector, const Vector3D& right_vector) { return left_vector.dot(right_vector); }

/**
 * @brief 欧拉角转旋转矩阵的列（局部系三轴在世界系下的方向）。
 * @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
 * @return 分别为 X、Y、Z 轴方向向量，类型均为 Vector3D。
 */
inline std::array<Vector3D, 3> rpyToRot(const Vector3D& rpy_deg) {
    Eigen::Matrix3d rotation_matrix = rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2]);
    return {rotation_matrix.col(0), rotation_matrix.col(1), rotation_matrix.col(2)};
}

/**
 * @brief 旋转矩阵转欧拉角描述（由三列轴向量构造旋转矩阵再逆解 RPY）。
 * @param[in] x_axis X 轴向量，类型为 Vector3D。
 * @param[in] y_axis Y 轴向量，类型为 Vector3D。
 * @param[in] z_axis Z 轴向量，类型为 Vector3D。
 * @return 欧拉角 (度)，类型为 Vector3D。
 */
inline Vector3D rotToRpy(const Vector3D& x_axis, const Vector3D& y_axis, const Vector3D& z_axis) {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x_axis.normalized();
    rotation_matrix.col(1) = y_axis.normalized();
    rotation_matrix.col(2) = z_axis.normalized();
    // 简单校验正交性
    if (std::abs(rotation_matrix.determinant() - 1.0) > 1e-3)
        throw std::invalid_argument("Input vectors do not satisfy the orthonormal rotation convention");
    return rotationMatrixToRpyDeg(rotation_matrix);
}

/**
 * @brief 欧拉角转轴角。
 * @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
 * @return 单位旋转轴（Vector3D）与转角（度）。
 */
inline std::pair<Vector3D, double> rpyToAxisAngle(const Vector3D& rpy_deg) {
    Eigen::Matrix3d rotation_matrix = rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2]);
    Eigen::AngleAxisd axis_angle(rotation_matrix);
    return std::make_pair(Vector3D(axis_angle.axis()), axis_angle.angle() * 180.0 / custom::_pi());
}

/**
 * @brief 轴角转欧拉角。
 * @param[in] rotation_axis 旋转轴，类型为 Vector3D（将被单位化）。
 * @param[in] angle_deg 旋转角（度）。
 * @return 欧拉角 (度)，类型为 Vector3D（RPY）。
 */
inline Vector3D axisAngleToRpy(const Vector3D& rotation_axis, double angle_deg) {
    if (rotation_axis.norm() < 1e-15) throw std::invalid_argument("Rotation axis cannot be a zero vector");
    Eigen::Matrix3d rotation_matrix =
        Eigen::AngleAxisd(angle_deg * custom::_pi() / 180.0, rotation_axis.normalized()).toRotationMatrix();
    return rotationMatrixToRpyDeg(rotation_matrix);
}

} // namespace pose_vector_math
