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
inline constexpr double pi() { return 3.14159265358979323846; }
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
 * @param[in] axis 轴序字符，'X'、'Y'、'Z'。
 * @param[in] angle_rad 旋转角度，弧度。
 * @return 3×3 旋转矩阵。
 */
inline Eigen::Matrix3d getBaseRotMatrix(char axis, double angle_rad) {
    if (axis == 'X') return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
    if (axis == 'Y') return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    if (axis == 'Z') return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return Eigen::Matrix3d::Identity();
}

/**
 * @brief 获取 RPYType 对应的字符串描述
 * @param[in] type 轴序枚举。
 * @return 字符串描述。
 */
inline std::string getRPYOrderStr(RPYType type) {
    const char* names[] = {"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX", 
                           "XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ"};
    return names[static_cast<int>(type)];
}

/**
 * @brief 欧拉角 (度) → 旋转矩阵，XYZ + EXTRINSIC，\f$R = R_z R_y R_x\f$。
 * @param[in] r1_deg,r2_deg,r3_deg 欧拉角分量 (度)。
 * @param[in] rpy_type 轴序。
 * @param[in] ref_type 内旋或外旋。
 * @return 3×3 旋转矩阵。
 */
inline Eigen::Matrix3d rpyDegToRotationMatrix(
    double r1_deg, double r2_deg, double r3_deg,
    RPYType rpy_type = RPYType::XYZ,
    ReferenceType ref_type = ReferenceType::EXTRINSIC)
{
    Vector3D rads = Vector3D(r1_deg, r2_deg, r3_deg) * custom::pi() / 180.0;
    std::string order = getRPYOrderStr(rpy_type);

    Eigen::Matrix3d m1 = getBaseRotMatrix(order[0], rads[0]);
    Eigen::Matrix3d m2 = getBaseRotMatrix(order[1], rads[1]);
    Eigen::Matrix3d m3 = getBaseRotMatrix(order[2], rads[2]);

    if (ref_type == ReferenceType::INTRINSIC) {
        return m1 * m2 * m3; // 内旋：R = R1 * R2 * R3
    } else {
        return m3 * m2 * m1; // 外旋：R = R3 * R2 * R1
    }
}

/**
 * @brief 旋转矩阵 → 欧拉角 (度)，XYZ + EXTRINSIC，\f$R = R_z R_y R_x\f$。
 * @param[in] r 3×3 旋转矩阵。
 * @return 欧拉角 (rx, ry, rz)，度。
 */
inline Vector3D rotationMatrixToRpyDeg(const Eigen::Matrix3d& r) {
    const double zer = 1e-8;
    double ry = std::atan2(-r(2, 0), std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));
    double rx, rz;
    if (std::abs(ry - custom::pi() / 2.0) < zer) {
        ry = custom::pi() / 2.0;
        rx = std::atan2(r(0, 1), r(1, 1));
        rz = 0;
    } else if (std::abs(ry + custom::pi() / 2.0) < zer) {
        ry = -custom::pi() / 2.0;
        rx = -std::atan2(r(0, 1), r(1, 1));
        rz = 0;
    } else {
        const double c = std::cos(ry);
        rz = std::atan2(r(1, 0) / c, r(0, 0) / c);
        rx = std::atan2(r(2, 1) / c, r(2, 2) / c);
    }
    return Vector3D(rx, ry, rz) * 180.0 / custom::pi();
}

/**
 * @brief 将位姿转为 4×4 齐次变换矩阵。
 * @param[in] p 输入位姿，类型为 Pose6D。
 * @return 齐次变换矩阵。
 */
inline Eigen::Matrix4d pose3dToMat4(const Pose6D& p) {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = rpyDegToRotationMatrix(p[3], p[4], p[5]);
    t.block<3, 1>(0, 3) = Vector3D(p[0], p[1], p[2]);
    return t;
}

/**
 * @brief 将 4×4 齐次矩阵转为位姿。
 * @param[in] m 齐次变换矩阵。
 * @return 位姿 Pose6D。
 */
inline Pose6D mat4ToPose(const Eigen::Matrix4d& m) {
    const Vector3D rpy = rotationMatrixToRpyDeg(m.block<3, 3>(0, 0));
    return Pose6D{{m(0, 3), m(1, 3), m(2, 3), rpy[0], rpy[1], rpy[2]}};
}

/**
 * @brief 坐标变换：位姿 p1 经 p2 变换得到新位姿，等价于 \f$T_3 = T_1 T_2\f$。
 * @param[in] a 第一个位姿（左侧），类型为 Pose6D。
 * @param[in] b 第二个位姿（右侧），类型为 Pose6D。
 * @return 复合后的位姿，类型为 Pose6D。
 */
inline Pose6D pose3dMultiply(const Pose6D& a, const Pose6D& b) {
    return mat4ToPose(pose3dToMat4(a) * pose3dToMat4(b));
}

/**
 * @brief 坐标逆变换：求位姿的逆变换。
 * @param[in] p 输入位姿，类型为 Pose6D，姿态按 \f$R_z \cdot R_y \cdot R_x\f$ 与矩阵逆一致。
 * @return 逆变换位姿，类型为 Pose6D。
 */
inline Pose6D pose3dInverse(const Pose6D& p) {
    Eigen::Matrix4d t = pose3dToMat4(p);
    Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
    inv.block<3, 3>(0, 0) = t.block<3, 3>(0, 0).transpose();
    inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * t.block<3, 1>(0, 3);
    return mat4ToPose(inv);
}

/**
 * @brief 计算两点之间的空间距离，姿态不参与计算。
 * @param[in] a 点 1（取平移），类型为 Pose6D。
 * @param[in] b 点 2（取平移），类型为 Pose6D。
 * @return 欧氏距离。
 */
inline double pose3dDistance(const Pose6D& a, const Pose6D& b) {
    return (Vector3D(b[0], b[1], b[2]) - Vector3D(a[0], a[1], a[2])).norm();
}

/**
 * @brief 计算两个位姿之间的偏移量（相对位姿 \f$T_a^{-1} T_b\f$）。
 * @param[in] a 第一个位姿，类型为 Pose6D。
 * @param[in] b 第二个位姿，类型为 Pose6D。
 * @return 偏移位姿，类型为 Pose6D。
 */
inline Pose6D pose3dOffset(const Pose6D& a, const Pose6D& b) {
    return mat4ToPose(pose3dToMat4(a).inverse() * pose3dToMat4(b));
}

/**
 * @brief 计算两个位姿之间的轴角差（相对旋转）。
 * @param[in] p1 第一个位姿，类型为 Pose6D。
 * @param[in] p2 第二个位姿，类型为 Pose6D。
 * @return 转角（度）与旋转轴单位向量，类型分别为 `double` 与 `Vector3D`。
 */
inline std::pair<double, Vector3D> pose3dAngle(const Pose6D& p1, const Pose6D& p2) {
    const Pose6D rel = pose3dOffset(p1, p2);
    Eigen::Matrix3d r = rpyDegToRotationMatrix(rel[3], rel[4], rel[5]);
    Eigen::AngleAxisd aa(r);
    return std::make_pair(aa.angle() * 180.0 / custom::pi(), Vector3D(aa.axis()));
}

/**
 * @brief 获得输入位姿的平移向量。
 * @param[in] p 输入位姿，类型为 Pose6D。
 * @return 平移向量，类型为 Vector3D。
 */
inline Vector3D pose3dGetTrans(const Pose6D& p) { return Vector3D(p[0], p[1], p[2]); }

/**
 * @brief 获得输入位姿的欧拉角描述（度）。
 * @param[in] p 输入位姿，类型为 Pose6D。
 * @return 欧拉角 (rx, ry, rz)，类型为 Vector3D。
 */
inline Vector3D pose3dGetRpy(const Pose6D& p) { return Vector3D(p[3], p[4], p[5]); }

/**
 * @brief 获得输入向量的模长。
 * @param[in] v 输入三维向量，类型为 Vector3D。
 * @return 模长。
 */
inline double vector3dNorm(const Vector3D& v) { return v.norm(); }

/**
 * @brief 对输入的向量进行归一化。
 * @param[in] v 输入三维向量，类型为 Vector3D。
 * @return 归一化后的向量，类型为 Vector3D。
 */
inline Vector3D vector3dNormalized(const Vector3D& v) {
    double n = v.norm();
    return (n < 1e-15) ? v : (v / n);
}

/**
 * @brief 两个向量叉乘。
 * @param[in] a 第一个向量，类型为 Vector3D。
 * @param[in] b 第二个向量，类型为 Vector3D。
 * @return 叉乘结果，类型为 Vector3D。
 */
inline Vector3D vector3dCross(const Vector3D& a, const Vector3D& b) { return a.cross(b); }

/**
 * @brief 两个向量点乘。
 * @param[in] a 第一个向量，类型为 Vector3D。
 * @param[in] b 第二个向量，类型为 Vector3D。
 * @return 点乘标量。
 */
inline double vector3dDot(const Vector3D& a, const Vector3D& b) { return a.dot(b); }

/**
 * @brief 欧拉角转旋转矩阵的列（局部系三轴在世界系下的方向）。
 * @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
 * @return 分别为 X、Y、Z 轴方向向量，类型均为 Vector3D。
 */
inline std::array<Vector3D, 3> rpyToRot(const Vector3D& rpy_deg) {
    Eigen::Matrix3d r = rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2]);
    return {r.col(0), r.col(1), r.col(2)};
}

/**
 * @brief 旋转矩阵转欧拉角描述（由三列轴向量构造旋转矩阵再逆解 RPY）。
 * @param[in] v1 X 轴向量，类型为 Vector3D。
 * @param[in] v2 Y 轴向量，类型为 Vector3D。
 * @param[in] v3 Z 轴向量，类型为 Vector3D。
 * @return 欧拉角 (度)，类型为 Vector3D。
 */
inline Vector3D rotToRpy(const Vector3D& v1, const Vector3D& v2, const Vector3D& v3) {
    Eigen::Matrix3d r;
    r.col(0) = v1.normalized();
    r.col(1) = v2.normalized();
    r.col(2) = v3.normalized();
    // 简单校验正交性
    if (std::abs(r.determinant() - 1.0) > 1e-3) 
        throw std::invalid_argument("输入向量不满足正交规范约定");
    return rotationMatrixToRpyDeg(r);
}

/**
 * @brief 欧拉角转轴角。
 * @param[in] rpy_deg 欧拉角 (度)，类型为 Vector3D。
 * @return 单位旋转轴（Vector3D）与转角（度）。
 */
inline std::pair<Vector3D, double> rpyToAxisAngle(const Vector3D& rpy_deg) {
    Eigen::Matrix3d r = rpyDegToRotationMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2]);
    Eigen::AngleAxisd aa(r);
    return std::make_pair(Vector3D(aa.axis()), aa.angle() * 180.0 / custom::pi());
}

/**
 * @brief 轴角转欧拉角。
 * @param[in] axis 旋转轴，类型为 Vector3D（将被单位化）。
 * @param[in] angle_deg 旋转角（度）。
 * @return 欧拉角 (度)，类型为 Vector3D（RPY）。
 */
inline Vector3D axisAngleToRpy(const Vector3D& axis, double angle_deg) {
    if (axis.norm() < 1e-15) throw std::invalid_argument("转轴不能为零向量");
    Eigen::Matrix3d r = Eigen::AngleAxisd(angle_deg * custom::pi() / 180.0, axis.normalized()).toRotationMatrix();
    return rotationMatrixToRpyDeg(r);
}

/** @brief 兼容性重定向。 */
inline Eigen::Matrix4d poseToMat4(const Pose6D& p) { return pose3dToMat4(p); }

} // namespace pose_vector_math
