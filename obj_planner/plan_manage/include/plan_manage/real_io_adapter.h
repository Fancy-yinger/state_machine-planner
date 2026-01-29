#include "io_adapter.h"
#include "obstacle_types.h"
// #include "coordinate_transform.h"  // 暂时注释，直接实现转换
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

// 共享内存中存储的最大障碍物数量
#define MAX_OBSTACLES 100

// 共享内存中的障碍物原始结构
struct SharedObstacle_t {
    double x, y, z;           // 中心位置
    double size_x, size_y, size_z; // 尺寸
    double velocity_x, velocity_y, velocity_z; // 速度
    int type;                 // 类型
};

// 障碍物列表头部
struct ObstacleListHeader_t {
    int count;                              // 障碍物数量
    SharedObstacle_t obstacles[MAX_OBSTACLES]; // 障碍物数组
    uint64_t timestamp;                      // 时间戳
};

// 【新增】共享内存中的遥控器数据结构
// 假设共享内存名为 "/uav_rc_data"
struct SharedRC_t {
    uint8_t connected;    // 遥控器是否连接
    uint8_t armed;        // 0:锁定, 1:解锁
    uint8_t mode;         // 飞行模式开关
    // 可以根据实际情况添加更多通道
    uint64_t timestamp;
};

// 发送给飞控的控制指令结构
struct ControlCommand_t {
    double x, y, z;           // 目标位置
    double vx, vy, vz;        // 目标速度
    double ax, ay, az;        // 目标加速度
    double yaw;               // 目标偏航角
    int32_t command_id;       // 【新增】对应 interface.h 的 CmdIdx
    uint64_t timestamp;       // 时间戳
};

// 【新增】基于本体坐标系的控制指令结构
struct BodyControlCommand_t {
    double vx_body, vy_body, vz_body;  // 目标速度 (Body Frame)
    double yaw_error;                  // 目标航向角误差 (rad)
    int32_t command_id;                 // 指令ID/模式
    uint64_t timestamp;                 // 时间戳
};

class RealIOAdapter : public IOAdapter {
private:
    // 共享内存文件描述符
    int ins_shm_fd_;
    int obs_shm_fd_;
    int cmd_shm_fd_;
    int rc_shm_fd_; // 【新增】RC共享内存句柄

    // 共享内存指针
    PlanningInertialData_t* ins_data_ptr_;
    ObstacleListHeader_t* obs_data_ptr_;
    ControlCommand_t* cmd_data_ptr_;
    BodyControlCommand_t* body_cmd_data_ptr_; // 【新增】本体坐标系控制指令指针
    SharedRC_t* rc_data_ptr_; // 【新增】RC数据指针

    // 【新增】BodyControlCommand 共享内存文件描述符
    int body_cmd_shm_fd_;

    // ENU坐标系偏移（记录起飞时的ENU坐标）
    Eigen::Vector3d enu_offset_;
    bool offset_set_;

    // 【新增】感知健康状态跟踪
    bool sensor_active_;               // 感知是否活跃
    uint64_t last_obstacle_timestamp_; // 上次收到的障碍物时间戳
    static const uint64_t OBSTACLE_TIMEOUT_MS = 300; // 障碍物数据超时阈值 (300ms)

    // 【新增】惯导健康状态跟踪
    bool ins_active_;                  // 惯导是否活跃
    uint64_t last_ins_timestamp_;      // 上次收到的惯导时间戳
    static const uint64_t INS_TIMEOUT_MS = 100;    // 惯导数据超时阈值 (100ms)

    // 【新增】坐标系转换配置
    bool use_ned_output_ = false;      // 输出是否使用NED坐标系（默认ENU）
    bool convert_yaw_to_ned_ = false;  // 是否转换偏航角到NED坐标系

public:
    RealIOAdapter() : ins_shm_fd_(-1), obs_shm_fd_(-1), cmd_shm_fd_(-1), body_cmd_shm_fd_(-1), rc_shm_fd_(-1),
                     ins_data_ptr_(nullptr), obs_data_ptr_(nullptr), cmd_data_ptr_(nullptr),
                     body_cmd_data_ptr_(nullptr), rc_data_ptr_(nullptr),
                     offset_set_(false), sensor_active_(false), last_obstacle_timestamp_(0),
                     ins_active_(false), last_ins_timestamp_(0) {
        std::cout << "🔥 Initializing RealIOAdapter..." << std::endl;

        // 初始化共享内存
        if (!initializeSharedMemory()) {
            std::cerr << "❌ Failed to initialize shared memory!" << std::endl;
            return;
        }

        std::cout << "✅ RealIOAdapter initialized successfully" << std::endl;
    }

    ~RealIOAdapter() {
        cleanupSharedMemory();
    }

    bool initializeSharedMemory() {
        // 打开惯导数据共享内存
        ins_shm_fd_ = shm_open("/uav_ins_data", O_RDONLY, 0666);
        if (ins_shm_fd_ == -1) {
            std::cerr << "❌ Failed to open INS shared memory: " << strerror(errno) << std::endl;
            return false;
        }

        // 映射惯导数据
        ins_data_ptr_ = (PlanningInertialData_t*)mmap(nullptr, sizeof(PlanningInertialData_t),
                                                      PROT_READ, MAP_SHARED, ins_shm_fd_, 0);
        if (ins_data_ptr_ == MAP_FAILED) {
            std::cerr << "❌ Failed to map INS memory: " << strerror(errno) << std::endl;
            return false;
        }

        // 打开障碍物数据共享内存
        obs_shm_fd_ = shm_open("/uav_obstacle_data", O_RDONLY, 0666);
        if (obs_shm_fd_ == -1) {
            std::cerr << "⚠️ Warning: Failed to open obstacle shared memory, using empty obstacles" << std::endl;
            // 创建空的障碍物数据
            obs_data_ptr_ = new ObstacleListHeader_t();
            memset(obs_data_ptr_, 0, sizeof(ObstacleListHeader_t));
        } else {
            // 映射障碍物数据
            obs_data_ptr_ = (ObstacleListHeader_t*)mmap(nullptr, sizeof(ObstacleListHeader_t),
                                                        PROT_READ, MAP_SHARED, obs_shm_fd_, 0);
            if (obs_data_ptr_ == MAP_FAILED) {
                std::cerr << "⚠️ Warning: Failed to map obstacle memory, using empty obstacles" << std::endl;
                obs_data_ptr_ = new ObstacleListHeader_t();
                memset(obs_data_ptr_, 0, sizeof(ObstacleListHeader_t));
                close(obs_shm_fd_);
                obs_shm_fd_ = -1;
            }
        }

        // 创建或打开控制指令共享内存
        cmd_shm_fd_ = shm_open("/uav_control_command", O_CREAT | O_RDWR, 0666);
        if (cmd_shm_fd_ == -1) {
            std::cerr << "❌ Failed to create control command shared memory: " << strerror(errno) << std::endl;
            return false;
        }

        // 设置共享内存大小
        ftruncate(cmd_shm_fd_, sizeof(ControlCommand_t));

        // 映射控制指令数据
        cmd_data_ptr_ = (ControlCommand_t*)mmap(nullptr, sizeof(ControlCommand_t),
                                                PROT_READ | PROT_WRITE, MAP_SHARED, cmd_shm_fd_, 0);
        if (cmd_data_ptr_ == MAP_FAILED) {
            std::cerr << "❌ Failed to map control command memory: " << strerror(errno) << std::endl;
            return false;
        }

        // 【新增】初始化遥控器共享内存
        // 注意：这里假设外部驱动程序已经创建了这个共享内存
        rc_shm_fd_ = shm_open("/uav_rc_data", O_RDONLY, 0666);
        if (rc_shm_fd_ == -1) {
            // 如果打不开，可能是驱动没起，为了安全先报错，或者在仿真测试时创建假的
            std::cerr << "⚠️ Warning: Failed to open RC shared memory (/uav_rc_data)" << std::endl;
        } else {
            rc_data_ptr_ = (SharedRC_t*)mmap(nullptr, sizeof(SharedRC_t), PROT_READ, MAP_SHARED, rc_shm_fd_, 0);
            if (rc_data_ptr_ == MAP_FAILED) {
                std::cerr << "❌ Failed to map RC memory" << std::endl;
                rc_data_ptr_ = nullptr;
            }
        }

        // 【新增】创建或打开本体坐标系控制指令共享内存
        body_cmd_shm_fd_ = shm_open("/uav_body_control_command", O_CREAT | O_RDWR, 0666);
        if (body_cmd_shm_fd_ == -1) {
            std::cerr << "❌ Failed to create body control command shared memory: " << strerror(errno) << std::endl;
            return false;
        }

        // 设置共享内存大小
        ftruncate(body_cmd_shm_fd_, sizeof(BodyControlCommand_t));

        // 映射本体坐标系控制指令数据
        body_cmd_data_ptr_ = (BodyControlCommand_t*)mmap(nullptr, sizeof(BodyControlCommand_t),
                                                          PROT_READ | PROT_WRITE, MAP_SHARED, body_cmd_shm_fd_, 0);
        if (body_cmd_data_ptr_ == MAP_FAILED) {
            std::cerr << "❌ Failed to map body control command memory: " << strerror(errno) << std::endl;
            return false;
        }

        // 初始化数据为0
        memset(body_cmd_data_ptr_, 0, sizeof(BodyControlCommand_t));

        std::cout << "✅ Body control command shared memory initialized" << std::endl;

        return true;
    }

    void cleanupSharedMemory() {
        // 清理惯导数据
        if (ins_data_ptr_ && ins_data_ptr_ != MAP_FAILED) {
            munmap(ins_data_ptr_, sizeof(PlanningInertialData_t));
        }
        if (ins_shm_fd_ != -1) {
            close(ins_shm_fd_);
        }

        // 清理障碍物数据
        if (obs_data_ptr_ && obs_data_ptr_ != MAP_FAILED) {
            munmap(obs_data_ptr_, sizeof(ObstacleListHeader_t));
        }
        if (obs_shm_fd_ != -1) {
            close(obs_shm_fd_);
        }

        // 清理控制指令数据
        if (cmd_data_ptr_ && cmd_data_ptr_ != MAP_FAILED) {
            munmap(cmd_data_ptr_, sizeof(ControlCommand_t));
        }
        if (cmd_shm_fd_ != -1) {
            close(cmd_shm_fd_);
        }

        // 【新增】清理遥控器数据
        if (rc_data_ptr_ && rc_data_ptr_ != MAP_FAILED) {
            munmap(rc_data_ptr_, sizeof(SharedRC_t));
        }
        if (rc_shm_fd_ != -1) {
            close(rc_shm_fd_);
        }

        // 【新增】清理本体坐标系控制指令数据
        if (body_cmd_data_ptr_ && body_cmd_data_ptr_ != MAP_FAILED) {
            munmap(body_cmd_data_ptr_, sizeof(BodyControlCommand_t));
        }
        if (body_cmd_shm_fd_ != -1) {
            close(body_cmd_shm_fd_);
        }
    }

    PlanningInertialData_t GetInertialData() override {
        PlanningInertialData_t data;

        if (!ins_data_ptr_) {
            std::cerr << "❌ INS data pointer is null!" << std::endl;
            return data;
        }

        // 从共享内存复制数据（ENU坐标系）
        memcpy(&data, ins_data_ptr_, sizeof(PlanningInertialData_t));

        // ==================== 【新增：惯导超时检查】 ====================
        // 获取当前系统时间 (毫秒)
        uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // 计算延迟
        uint64_t ins_latency = (now >= data.timestamp) ? (now - data.timestamp) : 0;

        // 更新惯导健康状态
        ins_active_ = (ins_latency <= INS_TIMEOUT_MS);

        // 如果时间戳更新了，记录最新的时间戳
        if (data.timestamp > last_ins_timestamp_) {
            last_ins_timestamp_ = data.timestamp;
        }

        if (ins_latency > INS_TIMEOUT_MS) {
            static int ins_warn_count = 0;
            if (ins_warn_count++ % 50 == 0) { // 防止刷屏，每50次打印一次
                std::cout << "🚨 [RealIO] INS TIMEOUT! Latency: " << ins_latency
                          << "ms (threshold: " << INS_TIMEOUT_MS << "ms). "
                          << "INS data stale!" << std::endl;
            }

            // 惯导超时是致命错误，标记数据无效
            data.position_valid = false;
            data.attitude_valid = false;
            data.velocity_valid = false;
        }
        // ===============================================================

        // 记录ENU偏移（第一次收到有效数据时，即起飞时刻）
        if (!offset_set_ && data.position_valid) {
            enu_offset_ = Eigen::Vector3d(data.position_x, data.position_y, data.position_z);
            offset_set_ = true;
            std::cout << "🚁 Takeoff ENU position: " << enu_offset_.transpose() << std::endl;
            std::cout << "🌍 Setting takeoff as world origin (0,0,0)" << std::endl;
        }

        // 如果偏移已设置，将ENU坐标转换为世界坐标
        if (offset_set_ && data.position_valid) {
            // 世界坐标 = ENU坐标 - 起飞点ENU偏移
            data.position_x -= enu_offset_.x();  // 起飞点在世界坐标系中为(0,0,0)
            data.position_y -= enu_offset_.y();
            data.position_z -= enu_offset_.z();
        }

        return data;
    }

    std::vector<obj_planner::Object3d> GetObstacles() override {
        std::vector<obj_planner::Object3d> obstacles;

        if (!obs_data_ptr_) {
            return obstacles;
        }

        // ==================== 【新增：时间戳超时检查】 ====================
        // 获取当前系统时间 (毫秒)
        uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // 获取共享内存里的数据时间戳
        uint64_t data_time = obs_data_ptr_->timestamp;

        // 计算延迟 (注意防止无符号数溢出)
        uint64_t latency = (now >= data_time) ? (now - data_time) : 0;

        // 更新感知健康状态
        sensor_active_ = (latency <= OBSTACLE_TIMEOUT_MS);

        // 如果时间戳更新了，记录最新的时间戳
        if (data_time > last_obstacle_timestamp_) {
            last_obstacle_timestamp_ = data_time;
        }

        if (latency > OBSTACLE_TIMEOUT_MS) {
            static int warn_count = 0;
            if (warn_count++ % 50 == 0) { // 防止刷屏，每50次打印一次
                std::cout << "🚨 [RealIO] SENSOR TIMEOUT! Latency: " << latency
                          << "ms (threshold: " << OBSTACLE_TIMEOUT_MS << "ms). "
                          << "Ignoring stale obstacles." << std::endl;
            }

            // 返回空障碍物列表，让规划器进入安全模式
            return obstacles;
        }
        // ===============================================================

        // 从共享内存读取障碍物并转换
        int count = obs_data_ptr_->count;
        if (count > MAX_OBSTACLES) {
            count = MAX_OBSTACLES;
        }

        for (int i = 0; i < count; i++) {
            const auto& shared_obs = obs_data_ptr_->obstacles[i];

            obj_planner::Object3d obj;

            // ==================== 【关键修改开始】 ====================
            // 坐标系对齐：将感知的绝对ENU坐标转换为规划器的相对ENU坐标
            // 前提：外部感知模块发送的是基于地球的绝对ENU坐标（原点固定）

            if (offset_set_) {
                // 如果已经记录了起飞点偏移量（enu_offset_），则减去它
                // 结果 = 障碍物绝对坐标 - 起飞点绝对坐标
                // 这样障碍物就变换到了以起飞点为原点 (0,0,0) 的坐标系中
                obj.rect3d.center.x = shared_obs.x - enu_offset_.x();
                obj.rect3d.center.y = shared_obs.y - enu_offset_.y();
                obj.rect3d.center.z = shared_obs.z - enu_offset_.z();
            } else {
                // 如果还没有定位（offset_set_为false），通常意味着无人机还未准备好
                // 此时直接使用原始值，或者你可以选择暂时忽略障碍物
                // 这里为了安全，保留原始值，但规划器在没有 offset 前通常不会进入 autonomous 模式
                obj.rect3d.center.x = shared_obs.x;
                obj.rect3d.center.y = shared_obs.y;
                obj.rect3d.center.z = shared_obs.z;
            }
            // ==================== 【关键修改结束】 ====================

            obj.rect3d.size.x = shared_obs.size_x;
            obj.rect3d.size.y = shared_obs.size_y;
            obj.rect3d.size.z = shared_obs.size_z;

            // 设置速度数据（速度是相对量，不需要减位置偏移，只需确保方向一致）
            if (shared_obs.velocity_x != 0 || shared_obs.velocity_y != 0 || shared_obs.velocity_z != 0) {
                obj.velocity.x = shared_obs.velocity_x;
                obj.velocity.y = shared_obs.velocity_y;
                obj.velocity.z = shared_obs.velocity_z;
            }
            obstacles.push_back(obj);
        }

        return obstacles;
    }

    // 【修改】真正获取实机遥控器状态
    RCData_t GetRCStatus() override {
        RCData_t data;
        data.is_valid = false;
        data.is_armed = false;

        if (rc_data_ptr_) {
            // 假设 armed 字段 1 代表解锁
            data.is_armed = (rc_data_ptr_->armed == 1);
            data.is_valid = true;

            // 可选：检查时间戳是否超时（例如超过1秒没更新认为遥控器断连）
            // auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            //    std::chrono::system_clock::now().time_since_epoch()).count();
            // if (now - rc_data_ptr_->timestamp > 1000) data.is_valid = false;
        } else {
            // 【危险】如果没有连接RC共享内存（如纯代码测试），
            // 为了安全，默认返回 false，除非你确认是在做无遥控器测试
            // data.is_armed = true; // 仅调试时解开注释
        }
        return data;
    }

    void SendCommand(const Eigen::Vector3d& pos,
                     const Eigen::Vector3d& vel,
                     const Eigen::Vector3d& acc,
                     double yaw,
                     int command_id) override {
        if (!cmd_data_ptr_) {
            std::cerr << "❌ Command data pointer is null!" << std::endl;
            return;
        }

        // 【新增】坐标系转换处理
        Eigen::Vector3d output_pos = pos;
        Eigen::Vector3d output_vel = vel;
        Eigen::Vector3d output_acc = acc;
        double output_yaw = yaw;

        if (use_ned_output_) {
            // 转换为NED坐标系 (ENU: East-North-Up -> NED: North-East-Down)
            output_pos = Eigen::Vector3d(pos.y(), pos.x(), -pos.z());
            output_vel = Eigen::Vector3d(vel.y(), vel.x(), -vel.z());
            output_acc = Eigen::Vector3d(acc.y(), acc.x(), -acc.z());

            if (convert_yaw_to_ned_) {
                output_yaw = -yaw; // 简化的偏航角转换
            }
        }

        // 填充控制指令数据
        cmd_data_ptr_->x = output_pos.x();
        cmd_data_ptr_->y = output_pos.y();
        cmd_data_ptr_->z = output_pos.z();

        cmd_data_ptr_->vx = output_vel.x();
        cmd_data_ptr_->vy = output_vel.y();
        cmd_data_ptr_->vz = output_vel.z();

        cmd_data_ptr_->ax = output_acc.x();
        cmd_data_ptr_->ay = output_acc.y();
        cmd_data_ptr_->az = output_acc.z();

        cmd_data_ptr_->yaw = output_yaw;

        // 【新增】赋值命令ID
        cmd_data_ptr_->command_id = command_id;

        cmd_data_ptr_->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // 数据已经写入共享内存，飞控进程可以读取
    }

    // 【新增】发送本体坐标系控制指令
    virtual void SendBodyControlCommand(const Eigen::Vector3d& target_vel_body,
                                        double target_yaw_error,
                                        int command_id) override {
        if (!body_cmd_data_ptr_) {
            std::cerr << "❌ Body control command data pointer is null!" << std::endl;
            return;
        }

        // 📢 填充 Body Frame 控制指令数据
        body_cmd_data_ptr_->vx_body = target_vel_body.x();  // X-前向速度
        body_cmd_data_ptr_->vy_body = target_vel_body.y();  // Y-右向速度
        body_cmd_data_ptr_->vz_body = target_vel_body.z();  // Z-上/下向速度

        body_cmd_data_ptr_->yaw_error = target_yaw_error;    // 航向角误差

        body_cmd_data_ptr_->command_id = command_id;         // 指令ID/模式

        body_cmd_data_ptr_->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // 📢 调试输出（可选）
        static int debug_counter = 0;
        if (++debug_counter % 50 == 0) { // 每500ms打印一次
            std::cout << "🚀 Body Control: vel=(" << target_vel_body.transpose()
                      << "), yaw_err=" << target_yaw_error
                      << ", cmd=" << command_id << std::endl;
        }

        // 数据已经写入共享内存，飞控进程可以读取
    }

    // 获取ENU偏移（用于调试）
    Eigen::Vector3d getENUOffset() const {
        return enu_offset_;
    }

    bool isOffsetSet() const {
        return offset_set_;
    }

    // 【新增】配置坐标系转换
    void setCoordinateTransform(bool use_ned_output, bool convert_yaw = true) {
        use_ned_output_ = use_ned_output;
        convert_yaw_to_ned_ = convert_yaw;
        std::cout << "🔄 Coordinate transform set: "
                  << (use_ned_output ? "NED output" : "ENU output")
                  << ", yaw convert: " << (convert_yaw ? "enabled" : "disabled") << std::endl;
    }

    // 【新增】检查感知传感器是否健康
    bool IsSensorActive() const {
        return sensor_active_;
    }

    // 【新增】获取当前感知延迟（毫秒）
    uint64_t GetSensorLatency() const {
        if (!obs_data_ptr_) return UINT64_MAX;

        uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        uint64_t data_time = obs_data_ptr_->timestamp;
        return (now >= data_time) ? (now - data_time) : 0;
    }

    // 【新增】获取上次障碍物时间戳
    uint64_t GetLastObstacleTimestamp() const {
        return last_obstacle_timestamp_;
    }

    // 【新增】检查惯导是否健康
    bool IsINSActive() const {
        return ins_active_;
    }

    // 【新增】获取当前惯导延迟（毫秒）
    uint64_t GetINSLatency() const {
        if (!ins_data_ptr_) return UINT64_MAX;

        uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        uint64_t data_time = ins_data_ptr_->timestamp;
        return (now >= data_time) ? (now - data_time) : 0;
    }

    // 【新增】获取上次惯导时间戳
    uint64_t GetLastINSTimestamp() const {
        return last_ins_timestamp_;
    }
};