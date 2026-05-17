/**
 * ETH / EIH 手眼变换与位姿/向量 API 演示入口。
 * 具体实现位于 `interface/pose_vector_math.hpp`。
 */

#include <pose_vector_math.hpp>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <tuple>

using namespace pose_vector_math;

const int label_width = 48;

void print_line6(const std::string& label, const Pose6D& pose6d) {
    std::cout << std::left << std::setw(label_width) << label << ": ";
    std::cout << std::fixed << std::setprecision(4);
    for (int i = 0; i < 6; ++i)
        std::cout << pose6d[static_cast<std::size_t>(i)] << (i < 5 ? "," : "");
    std::cout << "\n" << std::defaultfloat;
}

void print_vec3(const std::string& label, const Vector3D& vector3d) {
    std::cout << std::left << std::setw(label_width) << label << ": ";
    std::cout << std::fixed << std::setprecision(4);
    for (int i = 0; i < 3; ++i)
        std::cout << vector3d[i] << (i < 2 ? "," : "");
    std::cout << "\n" << std::defaultfloat;
}

int main() {
    const Pose6D target_in_camera{{
        130.51, 27.86, 515.003, 179.46, 4.58, 166.62}};

    std::cout << "--- eyes to hand (ETH) ---\n";
    const Pose6D camera_in_base{{
        -687.029, -412.132, -406.297, -44.9606, -33.0434, -1.92313}};
    const Pose6D target_in_base_eth = pose3dMultiply(camera_in_base, target_in_camera);
    print_line6("target_in_base[ETH]", target_in_base_eth);

    std::cout << "\n--- eyes in hand (EIH) ---\n";
    const Pose6D camera_in_flange{{
        206.9900, -331.5936, -426.7618, 177.1207, 1.5365, 101.0783}};
    const Pose6D flange_in_base{{
        566.417, 55.603, -78.360, -180.0, 0.0, 147.16}};
    const Pose6D target_in_flange = pose3dMultiply(camera_in_flange, target_in_camera);
    const Pose6D target_in_base_eih = pose3dMultiply(flange_in_base, target_in_flange);
    print_line6("target_in_base[EIH]", target_in_base_eih);

    const Pose6D& pose_from_camera_to_target = target_in_camera;
    const Pose6D& pose_from_base_to_camera = camera_in_base;

    std::cout << "\n--- pose API (target_in_camera, camera_in_base) ---\n";
    print_line6("pose3dInverse(target_in_camera)", pose3dInverse(pose_from_camera_to_target));
    std::cout << std::left << std::setw(label_width) << "pose3dDistance(target,camera)" << ": "
              << std::fixed << std::setprecision(4) << pose3dDistance(pose_from_camera_to_target, pose_from_base_to_camera) << "\n"
              << std::defaultfloat;
    print_line6("pose3dOffset(target,camera)", pose3dOffset(pose_from_camera_to_target, pose_from_base_to_camera));
    double angle_deg = 0;
    Vector3D rotation_axis;
    std::tie(angle_deg, rotation_axis) = pose3dAngle(pose_from_camera_to_target, pose_from_base_to_camera);
    std::cout << std::left << std::setw(label_width) << "pose3dAngle(target,camera)" << ": "
              << std::fixed << std::setprecision(4) << angle_deg << "° | "
              << rotation_axis[0] << "," << rotation_axis[1] << "," << rotation_axis[2]
              << "\n" << std::defaultfloat;
    print_vec3("pose3dGetTrans(target)", pose3dGetTrans(pose_from_camera_to_target));
    print_vec3("pose3dGetRpy(target)", pose3dGetRpy(pose_from_camera_to_target));

    std::cout << "\n--- pose6dToHomogeneous(target_in_camera) ---\n";
    print_line6("Input target_in_camera", pose_from_camera_to_target);
    const Eigen::Matrix4d T = pose6dToHomogeneous(pose_from_camera_to_target);
    std::cout << std::fixed << std::setprecision(8);
    for (int i = 0; i < 4; ++i) {
        std::cout << " ";
        for (int j = 0; j < 4; ++j) std::cout << (j ? "  " : "") << std::setw(14) << T(i, j);
        std::cout << "\n";
    }
    std::cout << std::defaultfloat;

    std::cout << "\n--- vector ---\n";
    const Vector3D v0(130.51, 27.86, 515.003);
    const Vector3D v1(566.417, 55.603, -78.360);
    print_vec3("v0", v0);
    print_vec3("v1", v1);
    std::cout << std::left << std::setw(label_width) << "vector3dNorm(v0)" << ": "
              << std::fixed << std::setprecision(4) << vector3dNorm(v0) << "\n"
              << std::defaultfloat;
    print_vec3("vector3dNormalized(v0)", vector3dNormalized(v0));
    print_vec3("vector3dCross(v0,v1)", vector3dCross(v0, v1));
    std::cout << std::left << std::setw(label_width) << "vector3dDot(v0,v1)" << ": "
              << std::fixed << std::setprecision(4) << vector3dDot(v0, v1) << "\n"
              << std::defaultfloat;

    std::cout << "\n--- rpy / rot ---\n";
    const Vector3D rpy(179.46, 4.58, 166.62);
    print_vec3("rpy (deg)", rpy);
    const auto axes = rpyToRot(rpy);
    print_vec3("rotation_x_axis", axes[0]);
    print_vec3("rotation_y_axis", axes[1]);
    print_vec3("rotation_z_axis", axes[2]);
    const Vector3D rpy_back = rotToRpy(axes[0], axes[1], axes[2]);
    print_vec3("rotToRpy(...)", rpy_back);

    Vector3D axis;
    double deg = 0;
    std::tie(axis, deg) = rpyToAxisAngle(rpy);
    std::cout << std::left << std::setw(label_width) << "rpyToAxisAngle" << ": "
              << "(deg | axis) = ("
              << std::fixed << std::setprecision(0) << deg << "° | "
              << std::setprecision(4) << axis.x() << "," << axis.y() << "," << axis.z()
              << ")\n";

    auto rpy_res = axisAngleToRpy(axis, 1.5);
    std::cout << std::left << std::setw(label_width) << "axisAngleToRpy(axis, 1.5)" << ": "
              << "(1.5°) -> "
              << std::fixed << std::setprecision(4)
              << rpy_res.x() << "," << rpy_res.y() << "," << rpy_res.z()
              << "\n" << std::defaultfloat;

    const Eigen::Matrix3d r3 = rpyDegToRotationMatrix(12, -34, 56);
    std::cout << std::left << std::setw(label_width) << "rpyDeg <-> rotationMatrix check" << ": ";
    const Vector3D mid = rotationMatrixToRpyDeg(r3);
    const bool ok = (r3 - rpyDegToRotationMatrix(mid[0], mid[1], mid[2])).norm() < 1e-9;
    std::cout << (ok ? "ok" : "fail") << "\n";

    return EXIT_SUCCESS;
}
