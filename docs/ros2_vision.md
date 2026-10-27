# ROS2 视觉链路

使用现有 `ros2_ws/src/drone_vision` 包，三个核心节点均为 C++：

- `camera_node`：IMX219 图像采集和 CameraInfo。
- `tag_detector_node`：AprilTag 36h11 / ID 0，像素中心、角点和标定位姿。
- `follow_preview_node`：最新图像、识别状态及预览建议的快照输出。

相机 NV12 亮度平面 → MONO8 灰度图 → AprilTag 检测与 PnP → `/dev/shm/drone_vision/status.json` → C++ 视觉 PID / 比赛调度 → MAVLink → PX4。
ROS 预览建议不会直接发送飞行命令，真实控制由既有 C++ 飞控服务执行。

## 配置与启动

配置统一位于工程 `config/vision.yaml`，相机内参位于 `config/camera_intrinsics.yaml`；标定来源见 `config/camera_intrinsics_source.json`。视觉默认 PID 与距离位于 `config/vision_control.json`，详见[距离 PID](vision_distance_pid.md)。

```bash
cd /home/pi/drone_project
bash scripts/start_vision.sh
```

通常通过 `scripts/start_runtime.py` 统一启动，已运行的节点不会重复启动。修改 YAML 后重启视觉节点；仅修改参数无需重新编译。源码修改后：

```bash
source /opt/ros/humble/setup.bash
cd /home/pi/drone_project/ros2_ws
colcon build --packages-select drone_vision
```

相机使用板上既有 Allwinner ISP 库和驱动，无新增底层采集模块。launch 负责视觉节点退出后的重启。

## 状态与网页

- 最新 JSON 和 JPEG 在 `/dev/shm/drone_vision/` 原子替换，慢连接不会积压历史图像。
- `/api/vision/status` 提供视觉状态，`/api/vision/frame.jpg` 提供单帧，`/api/vision/stream.mjpg` 提供实时画面。
- C++ 与网页检查快照单调时间；超过 0.5 s 的快照不作为新鲜识别结果。
- 靠近目标时短时丢失保持之前的前向方向，持续时间由任务的丢失参数决定；默认 3 s。达到该时间后停止并按稳定积木 ID 恢复搜索。
- 未标定或位姿无效时不能完成需要真实距离的控制任务；不会伪造成功。

## 离线检查

### 灰度与真实帧率

当前 YAML 启用 `camera_node.grayscale: true`，直接复制 NV12 的 Y 平面，省去 NV12→BGR→灰度的转换；ROS 图像数据量从每帧 921600 字节降为 307200 字节。关闭该参数即可恢复 BGR8；节点不带 YAML 启动时仍默认 BGR8。640×480、曝光 2000 微秒、旋转、标定、Tag 尺寸及控制接口均保留。

`denoise_sigma: 0.8` 使用 3×3 高斯滤波抑制高增益噪点，再提取候选；角点在原图精修，坐标不缩放。保留 3/13/23 三个阈值窗口。滤波可能降低极小、低对比度标签的检出率，优先改善照明；需要远距离极小标签时设置 `denoise_sigma: 0.0`，接受计算量增加。线程数仍为 4。`adaptive_thresh_win_min/max/step` 可调整窗口，但减少窗口的现场对比出现小目标漏检，当前不采用。

画框、灰度转预览彩色容器和 JPEG 编码均在独立预览线程，只有一个最新待处理帧；网页只显示板端 JPEG，不运行识别。`preview_fps: 10.0` 只限制画面刷新，不限制底层检测。无网页连接时识别继续运行。

每 5 秒运行日志记录 `Camera rate`（采集、发布、编码格式）和 `Vision rate`（处理率、检出率、平均检测耗时），两种帧率分别统计。相机丢失的心跳不再让 `camera_ready` 显示为真。只读测速并保存带时间戳的结果：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/measure_vision_rate.py --seconds 60
```

结果写入 `logs/performance/<UTC时间戳_运行标识>-vision-rate/summary.json`，区分处理、成功检出、有效位姿及 JPEG 帧率，附带参数和延迟。无目标场景的 30 Hz 处理率不能视为每秒 30 次有效检出。温度和场景复杂度会影响结果；不补帧或重复检测伪造计数。

`scripts/verify_ros2_camera.py --grayscale` 检查实机 MONO8→JPEG 链路；不带选项检查原 BGR8。`verify_ros2_vision.py` 覆盖 MONO8/BGR8、灰度过期拒绝、降噪及无网页时的识别链路；这些测试不会发送飞行命令。

`scripts/verify_vision_snapshots.py` 检查快照有效期和缺失文件；`scripts/verify_vision_control.py`、`scripts/verify_vision_distance.py` 使用 PTY 和合成视觉输入检查控制协议。真实相机测试需显式运行对应 ROS2 检查脚本。
