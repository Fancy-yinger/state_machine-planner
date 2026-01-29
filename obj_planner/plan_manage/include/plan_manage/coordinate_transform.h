/*
 * 坐标转换工具类
 * 提供ENU、NED、Body坐标系之间的转换
 */

#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

#include <Eigen/Dense>

/*
 * 坐标转换工具类
 * 提供ENU、NED、Body坐标系之间的转换
 * Author: Fancy
 * Date: 2026 01
 */

namespace obj_planner {

class CoordinateTransform {
public:
    // ENU到NED坐标系转换
    // ENU: East-North-Up
    // NED: North-East-Down
    static Eigen::Vector3d enuToNed(const Eigen::Vector3d& enu) {
        // NED = [ENU_y, ENU_x, -ENU_z]
        return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
    }

    // NED到ENU坐标系转换
    static Eigen::Vector3d nedToEnu(const Eigen::Vector3d& ned) {
        // ENU = [NED_y, NED_x, -NED_z]
        return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
    }

    // ENU偏航角到NED偏航角转换
    // ENU: 北向为0°，东向为90°，逆时针为正
    // NED: 北向为0°，东向为90°，顺时针为正（通常定义）
    static double enuYawToNed(double enu_yaw) {
        // 将ENU偏航角转换为NED偏航角
        return -enu_yaw; // 简化转换，实际可能需要更复杂的计算
    }

    // 速度向量ENU到NED转换
    static Eigen::Vector3d velocityEnuToNed(const Eigen::Vector3d& enu_vel) {
        return enuToNed(enu_vel);
    }

    // 加速度向量ENU到NED转换
    static Eigen::Vector3d accelerationEnuToNed(const Eigen::Vector3d& enu_acc) {
        return enuToNed(enu_acc);
    }

    // 偏航角标准化到 [-π, π] 范围
    static double normalizeYaw(double yaw) {
        while (yaw > M_PI) yaw -= 2 * M_PI;
        while (yaw < -M_PI) yaw += 2 * M_PI;
        return yaw;
    }

    // 计算从ENU位置向量指向目标位置的偏航角
    static double calculateYawFromVector(const Eigen::Vector3d& direction) {
        return normalizeYaw(atan2(direction.y(), direction.x()));
    }

    // 计算从ENU速度向量得到的偏航角
    static double calculateYawFromVelocity(const Eigen::Vector3d& velocity) {
        if (velocity.norm() < 0.1) {
            return 0.0; // 速度太小时返回0
        }
        return calculateYawFromVector(velocity);
    }
};

} // namespace obj_planner

#endif // COORDINATE_TRANSFORM_H