# lightning-lm 代码结构说明

本文档面向 `lightning-lm`（ROS2 package 名称：`lightning`）的开发者，目标是用“从入口到模块”的方式说明代码组织、数据流与关键文件位置，便于快速定位功能与修改点。

## 1. 顶层目录速览

> 以仓库根目录为起点（`/home/ubuntu/slam_ws/src/lightning-lm`）。

- `CMakeLists.txt`：ROS2 ament/cmake 顶层构建脚本；同时生成 `srv` 接口代码并安装 `config/`。
- `package.xml`：ROS2 依赖声明。
- `config/`：运行参数（yaml），在线/离线程序都会读取这里的配置。
- `src/`：核心实现（库 + 可执行程序）。
- `srv/`：ROS2 服务定义（`.srv`），例如 `SaveMap.srv`。
- `thirdparty/`：第三方依赖（如 Sophus 头文件、Livox driver、Pangolin 压缩包）。
- `doc/`：演示图片/gif 等资料（本文档也放在此目录）。
- `scripts/`：辅助脚本（依赖安装、bag 合并等）。
- `bin/`：构建输出的可执行文件（由顶层 CMake 设置 `CMAKE_RUNTIME_OUTPUT_DIRECTORY` 指向此目录）。

## 2. 构建与产物

- 顶层构建入口：`CMakeLists.txt`
  - 通过 `rosidl_generate_interfaces()` 从 `srv/*.srv` 生成接口代码。
  - `add_subdirectory(src)` 编译主库与可执行程序。
- 主库：`src/CMakeLists.txt` 中的 `${PROJECT_NAME}.libs`（即 `lightning.libs`）
  - 把 `common/`、`core/`、`ui/`、`wrapper/`、`io/`、`utils/` 的实现编译进一个共享库。
- 可执行程序：`src/app/CMakeLists.txt`
  - `run_slam_online` / `run_slam_offline`：建图（在线/离线）
  - `run_loc_online` / `run_loc_offline`：定位（在线/离线）
  - `run_frontend_offline`、`run_loop_offline`、`test_ui`：专项离线或调试用途

## 3. 运行入口（建议从这里读起）

### 3.1 在线建图（mapping online）

- 入口：`src/app/run_slam_online.cc`
- 核心系统类：`src/core/system/slam.h` + `src/core/system/slam.cc`
  - `SlamSystem::Init(yaml_path)`：读取 `config/*.yaml`，创建/初始化各模块
  - 在线模式下在 `SlamSystem` 内创建 ROS2 Node，并订阅 IMU/点云话题
  - `SlamSystem::ProcessIMU()` / `SlamSystem::ProcessLidar()`：将传感器消息送入前端 LIO

### 3.2 离线建图（mapping offline）

- 入口：`src/app/run_slam_offline.cc`
- 读取配置：`src/io/yaml_io.h`（`YAML_IO yaml(yaml_path); yaml.GetValue<T>(section, key)`）
- bag 遍历与消息反序列化：`src/wrapper/bag_io.h` + `src/wrapper/bag_io.cc`
  - `RosbagIO::AddImuHandle()`、`AddPointCloud2Handle()`、`AddLivoxCloudHandle()` 将 topic 映射到回调
  - 回调里直接调用 `SlamSystem::ProcessIMU/ProcessLidar`
- 结束时保存地图：`SlamSystem::SaveMap()`（`src/core/system/slam.cc`）

### 3.3 在线定位（localization online）

- 入口：`src/app/run_loc_online.cc`
- 系统类：`src/core/system/loc_system.h` + `src/core/system/loc_system.cc`
  - `LocSystem::Init(yaml_path)`：创建 ROS2 Node 并订阅 IMU/点云话题
  - 初始化后将数据送入定位模块 `core/localization/Localization`
  - 可选 TF 输出（受 `LocSystem::Options::pub_tf_` 控制，默认开启）：定位结果通过 TF 广播输出

### 3.4 离线定位（localization offline）

- 入口：`src/app/run_loc_offline.cc`
- 用 `RosbagIO` 按 topic 遍历 bag，并直接喂给 `core/localization/Localization`（离线定位不经过 `LocSystem`）。

## 4. 核心模块分层（src/ 目录）

### 4.1 `src/common/`：基础数据结构与参数

定位/建图过程中跨模块共享的“基本类型”大多在这里：

- `nav_state.h` / `nav_state.cc`：ESKF 状态（pose/vel/bias/gravity 等），提供 `GetPose()`。
- `keyframe.h`：关键帧结构（含 `pose_lio_` 前端位姿与 `pose_opt_` 后端优化位姿）。
- `imu.h`、`odom.h`、`measure_group.h`：IMU/里程计观测与同步结构（注：车辆里程计输入目前属于 TODO/未接入主流程）。
- `params.h` / `params.cc`：一个“从 YAML 加载参数”的结构体（注意其 YAML key 命名与 `config/default*.yaml` 不同，属于另一套配置读取方式）。

### 4.2 `src/io/`：文件与配置 IO

- `yaml_io.h` / `yaml_io.cc`：轻量 YAML 读取/写回封装（两层/三层 key 读取）。
- `file_io.*`：通用文件 IO（用于地图/缓存等）。
- `dataset_type.h`：数据集枚举与一些数据集默认 topic 名称（用于离线场景更方便）。

### 4.3 `src/wrapper/`：ROS2/rosbag2 适配层

目标是把“ROS2 消息/rosbag2 序列化消息”转换为系统内部类型：

- `ros_utils.h`：时间戳转换工具（`ToSec` 等）。
- `bag_io.h` / `bag_io.cc`：rosbag2 顺序读取，并对常见 msg 做反序列化回调分发。

### 4.4 `src/utils/`：通用工具

- `timer.*`：性能统计/打点。
- `pointcloud_utils.*`：点云工具函数。
- `async_message_process.h`、`sync.h`：异步队列/同步工具。

### 4.5 `src/core/`：算法与系统实现

`core/` 是主要“业务”目录，按功能再分：

#### 4.5.1 `core/system/`：系统编排层

- `slam.*`：建图系统编排（创建 LIO/回环/栅格/UI；在线订阅；保存地图服务）。
- `loc_system.*`：定位系统编排（在线订阅；TF 输出；驱动 `Localization`）。
- `async_message_process.h`：系统层异步处理实现（与 `utils/async_message_process.h` 并存，使用前建议先确认 include 的那个版本）。

#### 4.5.2 `core/lio/`：LIO 前端（建图与定位的基础）

核心类：

- `laser_mapping.h` / `laser_mapping.cc`：LIO 前端主体
  - 接收 IMU 与点云（支持标准 `PointCloud2` 与 Livox 自定义点云）
  - 内部维护 ESKF、局部地图（IVox）、关键帧生成、去畸变/降采样等
  - 对外提供 `GetKeyframe()` 与 `GetAllKeyframes()`，供系统层/回环/栅格/UI 使用
- `eskf.hpp` / `eskf.cc`：误差状态卡尔曼滤波相关实现
- `pointcloud_preprocess.*`：点云预处理（时间戳、滤波、采样等）

#### 4.5.3 `core/loop_closing/`：回环检测与后端优化

- `loop_closing.h` / `loop_closing.cc`
  - 接收关键帧（`AddKF`），进行候选检测、位姿优化、回环闭合
  - 触发回调给其他模块（例如栅格地图重绘）

#### 4.5.4 `core/localization/`：定位管线

入口类：

- `localization.h` / `localization.cpp`：定位主流程
  - 接收 IMU/点云并产出定位结果（TF 输出由系统层决定）
  - 内含异步点云处理队列：`lidar_odom_proc_cloud_`、`lidar_loc_proc_cloud_`
- `localization_result.*`：定位结果结构（提供 `ToGeoMsg()` 等转 ROS 消息工具）
- `lidar_loc/`：激光定位（NDT/ICP 等），含 `pclomp` 的 NDT OMP 实现
- `pose_graph/`：定位用的图优化/外推/平滑（`pgo.*`、`pose_extrapolator.*` 等）

#### 4.5.5 `core/maps/`：地图存储与分块

- `tiled_map.*`、`tiled_map_chunk.*`：点云地图的分块存储、加载与转换（适配大场景）。

#### 4.5.6 `core/g2p5/`：3D->2D 栅格地图（可选）

- `g2p5.h/.cc`、`g2p5_map.*`、`g2p5_subgrid.*`
  - 接收关键帧并增量构建栅格地图
  - `SlamSystem::SaveMap()` 会把栅格另存为 ROS 兼容格式（`map.pgm` + `map.yaml`）

#### 4.5.7 `core/ivox3d/`：IVox 局部地图数据结构

- `ivox3d.h`、`ivox3d_node.hpp`、`hilbert.hpp`：用于高效近邻查询/局部地图组织。

#### 4.5.8 `core/lightning_math.hpp`：数学/转换工具集合

包含常见的 SE3/欧拉角转换、`FromSec()` 时间戳转换等通用工具函数，被多处引用。

### 4.6 `src/ui/`：可视化（Pangolin）

UI 作为可选组件由系统层按配置打开：

- `pangolin_window.*`、`pangolin_window_impl.*`：窗口与渲染线程
- `ui_cloud.*`、`ui_car.*`、`ui_trajectory.*`：点云、车辆、轨迹绘制

## 5. 配置文件（config/）与关键参数

以 `config/default.yaml` 为例（其他 `default_*.yaml` 通常是针对不同数据集/设备的变体）：

- `common.*`：输入话题（IMU/LiDAR/Livox）
- `system.*`：功能开关（回环、UI、2D 栅格等）、地图路径、是否发布 TF
- `fasterlio.*`：LIO 前端参数（采样、迭代、外参等）
- `loop_closing.*`、`lidar_loc.*`、`pgo.*`、`maps.*`：回环/定位/地图加载策略参数

建议定位参数读取位置时，从系统层开始看：

- 建图（SlamSystem）：`core/system/slam.cc` 直接 `YAML::LoadFile()` 读取 `system/common` 段。
- 定位（LocSystem）：`core/system/loc_system.cc` 使用 `YAML_IO` 读取 `system/common` 段。

## 6. 推荐阅读顺序（上手最快）

1. `src/app/run_slam_online.cc` → `src/core/system/slam.cc`（看在线数据如何进系统）
2. `src/core/lio/laser_mapping.*`（看前端状态/关键帧如何产生）
3. `src/core/loop_closing/loop_closing.*`（看回环如何消费关键帧并优化）
4. `src/core/system/slam.cc::SaveMap` + `src/core/maps/tiled_map.*`（看地图如何落盘/分块）
5. `src/app/run_loc_online.cc` → `src/core/system/loc_system.cc` → `src/core/localization/localization.*`（看定位链路）
