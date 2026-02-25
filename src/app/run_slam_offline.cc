//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "core/system/slam.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    SlamSystem::Options options;
    options.online_mode_ = false;

    SlamSystem slam(options);

    /// 实时模式好像掉帧掉的比较厉害？

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM("new_map");

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    std::string livox_topic = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rosbag
        /// IMU 的处理
        .AddImuHandle(imu_topic,
                      [&slam](IMUPtr imu) {
                          slam.ProcessIMU(imu);
                          return true;
                      })

        /// lidar 的处理
        .AddPointCloud2Handle(lidar_topic,
                              [&slam](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                  slam.ProcessLidar(msg);
                                  return true;
                              })

        /// livox 的处理
        .AddLivoxCloudHandle(livox_topic,
                             [&slam](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
                                 slam.ProcessLidar(msg);
                                 return true;
                             })
        .Go();

    slam.SaveMap("");

    /// 根据配置保存轨迹
    YAML::Node config = YAML::LoadFile(FLAGS_config);
    if (config["system"] && config["system"]["save_trajectory"] && config["system"]["save_trajectory"].as<bool>()) {
        std::string trajectory_path = config["system"]["trajectory_file_path"].as<std::string>();
        slam.SaveTrajectory(trajectory_path);
        LOG(INFO) << "Trajectory saved to: " << trajectory_path;
    }

    Timer::PrintAll();

    LOG(INFO) << "done";

    return 0;
}