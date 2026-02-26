//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

/// SLAM 系统编排层实现：
/// - 负责创建/初始化前端 LIO（LaserMapping）与可选模块（回环、UI、2D 栅格）
/// - 在线模式下创建 ROS2 Node 并订阅传感器话题
/// - 在关键帧产生时，将关键帧分发给回环/栅格/UI
/// - 提供保存地图服务与离线保存入口
SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    /// 1) 初始化前端 LIO（建图/关键帧产生的主入口）
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    /// 2) 从 YAML 读取系统开关，按需创建可选模块
    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            /// 栅格更新回调：将地图转换为 CV 图像并显示
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// 3) 在线模式：创建 ROS2 Node 并订阅传感器话题（IMU/点云/Livox）
        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();
        if (yaml["ins"] && yaml["ins"]["topic"]) {
            ins_topic_ = yaml["ins"]["topic"].as<std::string>();
        } else {
            ins_topic_ = "/localization_info";
        }

        rclcpp::QoS qos(10);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        ins_sub_ = node_->create_subscription<bot_msg::msg::LocalizationInfo>(
            ins_topic_, qos, [this](bot_msg::msg::LocalizationInfo::SharedPtr msg) {
                if (!running_) {
                    return;
                }
                lio_->ProcessInsMsg(msg);
            });

        /// 保存地图服务（在线模式）：调用 SaveMap(req,res) 落盘 ./data/<map_id>/
        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    /// 标记系统进入运行态（未 Start 前传感器数据会被丢弃）
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    /// 由 ROS2 service 触发保存：map_id 作为目录名
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    /// 保存目录：优先使用传入 path；为空则落盘到 ./data/<map_name_>/
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    /// 为避免混用旧地图，若目录已存在则先清空再重建
    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    /// global_map：点云地图（是否使用回环优化结果由参数决定）
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    /// 将“整幅点云地图”转换为分块地图（大地图加载更友好）
    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    /// 同时输出一个整图 pcd 便于调试/可视化
    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    /// 保存轨迹
    SaveTrajectory(save_path + "trajectory.csv");

    if (options_.with_gridmap_) {
        /// 2D 栅格：以 nav_msgs/OccupancyGrid 为中间格式，输出 map.pgm + map.yaml
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::SaveTrajectory(const std::string& file_path, bool use_high_frequency) {
    /// 轨迹保存路径：优先使用传入 path；为空则使用配置文件中的路径
    std::string save_path = file_path;
    if (save_path.empty()) {
        save_path = "./data/trajectory.csv";
    }

    LOG(INFO) << "Saving trajectory to " << save_path;

    /// 打开文件
    std::ofstream ofs(save_path);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open trajectory file: " << save_path;
        return;
    }

    /// 写入 CSV 文件头
    ofs << "timestamp,tx,ty,tz,qx,qy,qz,qw\n";
    ofs << std::fixed << std::setprecision(12);

    if (use_high_frequency) {
        /// 使用高频率数据（所有帧位姿）
        auto poses = lio_->GetAllPoses();
        LOG(INFO) << "Saving high frequency trajectory with " << poses.size() << " poses to " << save_path;

        for (const auto& pose : poses) {
            SE3 T = pose.GetPose();
            Vec3d t = T.translation();
            Quatd q = T.unit_quaternion();

            ofs << pose.timestamp_ << "," << t.x() << "," << t.y() << "," << t.z() << "," << q.x() << "," << q.y()
                << "," << q.z() << "," << q.w() << "\n";
        }
    } else {
        /// 使用关键帧数据（低频率）
        auto keyframes = lio_->GetAllKeyframes();
        LOG(INFO) << "Saving keyframe trajectory with " << keyframes.size() << " keyframes to " << save_path;

        for (const auto& kf : keyframes) {
            const auto& pose = kf->GetOptPose();
            const auto& t = pose.translation();
            const auto& q = pose.unit_quaternion();
            double timestamp = kf->GetState().timestamp_;

            ofs << timestamp << "," << t.x() << "," << t.y() << "," << t.z() << "," << q.x() << "," << q.y() << ","
                << q.z() << "," << q.w() << "\n";
        }
    }

    ofs.close();
    LOG(INFO) << "Trajectory saved to " << save_path;
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    /// IMU 只进入前端 LIO（时间同步/积分/预测在前端内部完成）
    lio_->ProcessIMU(imu);
}
/**
 * @brief 处理激光雷达点云
 *
 * @param cloud 激光雷达点云消息指针
 */
void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    /// 点云进入前端：预处理 + 迭代优化 + 更新关键帧
    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    /// 仅当产生了“新关键帧”时才继续向后端/栅格/UI 分发
    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        /// 回环模块消费关键帧（内部异步）
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        /// 栅格模块消费关键帧（在线模式下内部可异步渲染）
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        /// 3D UI 更新关键帧与轨迹显示
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    /// Livox 点云路径与 PointCloud2 一致（内部会按消息类型走不同解析）
    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    /// 在线模式下进入 ROS2 事件循环；离线模式不需要 Spin()
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
