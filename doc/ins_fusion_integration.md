# INS/RTK 融合说明（LocalizationInfo）

## 1. 目的
- 将 `bot_msg/msg/LocalizationInfo` 作为绝对位姿观测，接入：
  - SLAM 前端 `LaserMapping + ESKF`
  - 定位端 `Localization + PGO`
- 增加统一开关：可配置“融合 INS”或“不融合 INS”。

## 2. 配置项
在 `config/default*.yaml` 中新增/使用 `ins` 配置：

```yaml
ins:
  enable_fusion: true        # true: 融合INS, false: 完全关闭INS融合
  topic: "/localization_info"
  use_llh: true
  base_longtitude: 117.37730582
  base_latitude: 31.96316616
  base_altitude: 0.0
  max_time_diff: 0.05
  rtk_other_noise_scale: 2.0
```

说明：
- `enable_fusion=false` 时，不订阅 INS 话题，也不做前端/PGO 融合。
- `use_llh=true` 时使用经纬高转 ENU；否则直接使用消息里的 `east/north/up`。
- `max_time_diff` 为 INS 与 LIO/PGO 对齐的时间阈值（秒）。

## 3. 消息与坐标约定
输入消息：`/home/ubuntu/slam_ws/src/bot_msg/msg/LocalizationInfo.msg`

实现中使用字段：
- 时间：`header.stamp`
- 位置：`latitude/longtitude/altitude`（或 `east/north/up`）
- 姿态：`yaw/pitch/roll`
- 状态：`rtk_status`

坐标与姿态转换：
- WGS84: `LLH -> ECEF -> ENU`
- 航向角转 ENU yaw：`yaw_enu = (90 - yaw_heading_deg) * deg2rad`
- 姿态构造顺序：`Rz(yaw) * Ry(pitch) * Rx(roll)`

## 4. rtk_status 与噪声策略
当前实现按业务约定：
- `status == 4`：固定解（`rtk_fix_*`）
- 其它状态：`rtk_other_* * rtk_other_noise_scale`

默认：
- `rtk_other_noise_scale = 2.0`
- 角度噪声会转为弧度参与协方差构建。

注意：`LocalizationInfo.msg` 内注释与当前业务约定可能存在差异（注释写 3/4，当前实现按 4/5 语义中的“4=固定”处理）。如上游定义变更，请同步调整转换逻辑。

## 5. 系统接入位置
### 5.1 订阅层
- `core/system/slam.*`：在线建图系统按 `ins.enable_fusion` 决定是否订阅 INS。
- `core/system/loc_system.*`：在线定位系统按 `ins.enable_fusion` 决定是否订阅 INS。

### 5.2 前端融合（LIO）
- `core/lio/laser_mapping.*`
  - 接收并缓存 INS。
  - 按 `max_time_diff` 找最近观测。
  - 触发 `ESKF::ObsType::GPS` 更新（使用 INS 自定义观测模型）。

### 5.3 定位融合（PGO）
- `core/localization/localization.*`
  - 接收 INS 并送入 `PGO::ProcessIns`。
- `core/localization/pose_graph/pgo*.{h,cc}`
  - 维护 INS 队列。
  - 为 PGO 帧匹配 INS 观测。
  - 添加 `EdgeSE3Prior` 绝对约束，信息矩阵由协方差逆得到。

## 6. 关闭融合时的行为
当 `ins.enable_fusion=false`：
- 不创建 INS 订阅。
- `LaserMapping` 不缓存/匹配 INS，不执行 INS 更新。
- `Localization` 不向 PGO 注入 INS 观测。
- 系统退化为原始“IMU + Lidar”流程。

## 7. 依赖与构建
新增依赖：
- `bot_msg`（`package.xml` / `cmake/packages.cmake` / `src/CMakeLists.txt`）

已验证构建命令：

```bash
cd /home/ubuntu/slam_ws
colcon build --packages-select bot_msg lightning --symlink-install
```

## 8. 建议的 A/B 使用方式
1. 复制同一份 `default_*.yaml`，仅切换 `ins.enable_fusion`。
2. 同路线录包，分别运行 `enable_fusion=true/false`。
3. 对比轨迹和地图质量（ATE/RPE、回环一致性、重叠区域误差）。
4. 结合日志关键字排查：
   - `INS fusion enabled/disabled`
   - `INS timestamp rollback`
   - PGO/ESKF 残差异常峰值。

