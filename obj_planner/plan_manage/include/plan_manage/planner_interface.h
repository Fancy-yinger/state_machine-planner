/*
 * @Function:Obj Planner:Trajectory Optimize Base Bspline
 * @Author:Fancy
 * @Date:2026 01
 */

#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/plan_container.hpp>
#include "Bspline.h"
#include <chrono>
#include "obstacle_types.h"



namespace  obj_planner
{
    struct PathPoint
    {
        double x;
        double y;
        double z;
        double vx;      // x方向速度
        double vy;      // y方向速度
        double vz;      // z方向速度
        double speed;   // 总速度大小
        double yaw;     // 偏航角（弧度）
        double pitch;   // 俯仰角（弧度）
        double yaw_rate; // 偏航角速度（弧度/秒）- [新增] 用于前馈控制
        double ax;      // x方向加速度
        double ay;      // y方向加速度
        double az;      // z方向加速度
        double time_from_start; // [新增] 从轨迹开始的时间（秒）
    };

    struct ObstacleInfo
    {
        float x;
        float y;
        float z;
        double size_x;  // 障碍物在x方向的尺寸
        double size_y;  // 障碍物在y方向的尺寸
        double size_z;  // 障碍物在z方向的尺寸
        int type;        // 0: 长方体, 1: 圆柱体 (可选)
    };

    class PlannerInterface
    {
           
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                                Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                                Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set);
            PlanParameters pp_;
            LocalTrajData local_data_;
            shared_ptr<SDFMap> grid_map_;

        private:

            BsplineOptimizer::Ptr bspline_optimizer_rebound_;

            int continous_failures_count_{0};

            void updateTrajInfo(const UniformBspline &position_traj);

            void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                            double &time_inc);

            bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

            // 将Object3d转换为内部的ObstacleInfo
            ObstacleInfo convertObject3dToObstacleInfo(const Object3d& obj);

        private:

           
            std::vector<PathPoint> _global_plan_traj_;
            std::vector<PathPoint> _plan_traj_results_;


        public:
            PlannerInterface();

            ~PlannerInterface();

            void initParam(double max_vel,double max_acc,double max_jerk);

            void initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d org,double inflate_values);

            void setPathPoint(std::vector<PathPoint> &plan_traj);

            void setObstacles(std::vector<ObstacleInfo> &obstacle, Eigen::Vector3d current_drone_pos);

            void makePlan();

            void getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results);  

            void getTraj();

            // 新的障碍物设置接口
            void setObstacles(const std::vector<Object3d>& objects, Eigen::Vector3d current_drone_pos);

            // 新增：带状态信息的规划函数
            bool makePlanWithState(const Eigen::Vector3d& start_pt, 
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& start_acc,
                          const Eigen::Vector3d& target_pt,
                          const Eigen::Vector3d& target_vel,
                          const Eigen::Vector3d& target_acc);


    };

    // 在 planner_interface.h 中添加
    class LargeScalePlanner {
    private:
        std::shared_ptr<PlannerInterface> local_planner_;
        std::vector<PathPoint> global_path_;
        double segment_length_;
        int current_segment_;
        
    public:
        LargeScalePlanner();
        void setGlobalPath(const std::vector<PathPoint>& global_path);
        bool planNextSegment();
        std::vector<PathPoint> getCurrentTrajectory();
        bool isFinished() const;
    };



}


#endif
