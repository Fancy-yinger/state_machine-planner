#ifndef OBSTACLE_TYPES_H
#define OBSTACLE_TYPES_H

#include <vector>
#include <string>


// 【新增】 加入命名空间开始
namespace obj_planner {
    struct Point {
        double x, y, z;
    };

    typedef std::vector<Point> PointArray;

    struct Rect3d {
        Point center;
        Point centerCovariance;
        Point size;
        Point sizeCovariance;
        Point orientation;
        Point orientationCovariance;
        PointArray corners;
    };

    struct Object3d {
        // 我们只需要位置和大小信息，其他字段可以忽略或设为默认值
        double existProb{ 0.0 };
        uint32_t id{ 0U };
        uint8_t cls{ 0U };
        std::string clsDescription;
        double clsConfidence{ 0.0 };
        Point velocity;
        Point absVelocity;
        PointArray pointsInObject;
        uint8_t cipvFlag{ 0U };
        Point acceleration;
        uint8_t cameraStatus{ 0U };
        std::string coordinate;
        Rect3d rect3d;
        uint8_t blinkerStatus{ 0U };
    };
}
#endif