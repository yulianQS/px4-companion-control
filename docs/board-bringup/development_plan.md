# 无人机伴随计算平台开发计划

日期：2026-09-21。本文是分阶段实施方案；除诊断脚本外，以下节点尚未实现或部署。

## 1. 运行边界与架构

OrangePi 负责感知、数据整合与任务决策；飞控独立负责姿态控制、执行器输出及失效保护。当前只完成地面环境验证，不启动解锁、模式切换、位置目标或电机控制。

```text
Sony IMX219 → MIPI CSI-2 → Linux camera driver → Allwinner ISP → V4L2
                                                                  ↓
Camera → camera_node → sensor_msgs/Image → vision_node → target/quality
                      ↘ vio_node ← timestamped IMU
Flight controller ↔ mavlink_bridge → telemetry / diagnostics
                    ↑ future mission_manager (disabled initially)
All processes → health_monitor → bounded diagnostic logs
```

| 节点 | 输入与输出 | 执行方式与初始预算（待实测） |
|---|---|---|
| camera_node | V4L2 → sensor_msgs/Image、CameraInfo | 先 640×480、15 Hz；专用采集线程，最新帧队列 1–2 |
| vision_node | 图像 → 检测框、类别、置信度、源图时间戳 | 单独进程，模型只加载一次；先 320 输入，每秒 2–5 次目标预算 |
| mavlink_bridge | 唯一串口所有者；MAVLink → 遥测及连接状态 | pyserial/pymavlink 环境独立；超时读、有界队列、不阻塞 ROS executor |
| vio_node | 图像、同步 IMU → nav_msgs/Odometry、质量状态 | C++ 优先；先离线验证，再量测 ARM64 资源占用 |
| health_monitor | 心跳、帧龄、串口错误、CPU/内存/温度 | 1 Hz；检测进程失联和积压，触发降频而非直接控制电机 |
| mission_manager | 检测结果、里程计与飞控状态 | 后续实现；初始禁止发送控制指令 |

图像 QoS 使用兼容的 sensor-data / best-effort 配置，浅队列，允许丢弃过时图像。状态和确认消息采用 reliable、有限深度；飞行命令需要专门状态机与过期时间，不能仅依赖 DDS 可靠性。参考 [ROS2 Humble QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)。

## 2. 环境隔离

- 视觉和 pymavlink 使用 `/home/pi/venv/bin/python`；保留已有 OpenCV/NumPy/SciPy。
- ROS2 使用 `/usr/bin/python3` 和 `/opt/ros/humble/setup.bash`。先验证 rclpy，避免把虚拟环境 NumPy 2.x 混入系统 cv_bridge 的二进制依赖。
- 第一版可让视觉进程通过有界本地 IPC 与系统 Python ROS2 节点通信，或使用 C++ 相机节点；跨进程图像传输要计入复制开销。
- 不全局修改 `.bashrc`、PYTHONPATH 或系统 Python，不安装 PyTorch、ROS Desktop、Gazebo、RViz、Nav2。

## 3. 摄像头与 OpenCV

首先解决实际传感器枚举，核对型号、接口、排线方向及供电；设备树名称不能证明连接了同型号传感器。用 `v4l2-ctl --list-devices`、`--list-formats-ext` 记录能力；选择支持 capture 的节点而不是 metadata/ISP 节点。

通过 `camera_test.py` 保存真实快照后，进行 60 秒限时采集，记录成功帧、丢帧、实际 FPS、曝光及帧龄。默认不改变相机分辨率；后续参数经能力查询确定。记录 BGR/RGB、图像尺寸、畸变模型与 CameraInfo；VIO 采用尽可能接近曝光的时间戳，不能把 Python 接收时间当作硬件同步时间。

## 4. 串口与 MAVLink

UART7 从运行中的 device-tree `serial7` 别名和 tty 的 `of_node` 链接建立映射；禁止按 tty 数字猜测。电脑 COM25 是 Windows USB 串口名，须单独验证它与开发板引脚的对应关系。

阶段顺序：打开串口且不发送 → 台架已知字符串双向测试 → 接收飞控 HEARTBEAT → 记录 system/component ID 与协议版本 → 限量遥测 → 才考虑指令状态机。115200、8N1 仅是台架起始设置，飞控波特率必须两端核对。8N1 的理论字节上限为 baud/10，即 115200 时约 11.5 kB/s；规划负载先限制在 50% 左右，留出突发和协议开销。

仅一个进程打开 UART；ROS2、地面站共享时用明确的路由服务，不能分别抢占串口。连接前核对 TTL 电平、公共地与交叉 TX/RX，不能把 RS232 电平或电源针当作 TTL 信号。具体物理针号待对应版本原理图核实。

先用已安装 pymavlink 做接收解析、超时和断线重连。不自动发送 ARM/DISARM、COMMAND_LONG、参数写入、RC override 或目标位置。未来命令层必须确认目标 ID、当前模式、授权状态、ACK、限次重试、帧龄和链路失效策略。

## 5. PX4 / ArduPilot

- 通用基础选 MAVLink 串口，复用已有 pymavlink。飞控固件类型、版本、端口配置与数据率尚未获取。
- PX4 原生 ROS2 可选 uXRCE-DDS：飞控 client 与伴随计算机 agent 配对，固定 firmware、px4_msgs 和 agent 版本。DDS 与 MAVLink 不直接混用同一裸 UART。依据 [PX4 ROS2 指南](https://docs.px4.io/main/en/ros2/user_guide) 和 [uXRCE-DDS 文档](https://docs.px4.io/main/en/middleware/uxrce_dds) 核对所用固件版本后再部署。
- ArduPilot 首选 MAVLink；需要 AP_DDS 时先核实飞控固件构建和相应支持。参考 [ArduPilot companion computers](https://ardupilot.org/dev/docs/companion-computers.html)。当前不安装完整 MAVROS 或 DDS agent。
- 外部里程计上送需核对 ENU/NED、FLU/FRD、时间同步、协方差、重置计数和飞控接收参数；测试完成前不参与飞控融合。

## 6. YOLO 部署

已有 `/opt/yolov5` 和 `/opt/vpm_run` 厂商示例保持原位；先检查其模型说明、运行库、输入和输出，不能据目录名认定能处理自定义模型。

第一条可评估路线：在电脑导出小型模型 ONNX，开发板复用 OpenCV DNN。必须先验证当前 OpenCV 5 构建的 ONNX 导入和所用算子支持；这不是已经完成的推理测试。参考 [OpenCV 4→5 迁移说明](https://github.com/opencv/opencv/wiki/OpenCV-4-to-5-migration)。模型固定 batch=1，优先静态形状；NCHW/NHWC、dtype、RGB/BGR、归一化、letterbox、输出布局、NMS 和标签以该模型导出约定为准，不照搬其他 YOLO 版本。

只在电脑训练和转换，不在板上装大型训练框架。模型首次批准后存入 `models/`，附来源、许可证、SHA256、标签和预处理契约。先用固定样本对齐 PC/ARM64 输出，再测推理 P50/P95、RSS、温度和掉帧；不承诺未经测量的 FPS。

第二条路线：现有 A733 NPU 厂商工具链。先检查示例可执行文件与运行库、SDK 文档和模型转换支持，再决定是否移植。NPU 驱动存在不等于任意 ONNX 可直接加速。本次不改权重、不转换或重新训练已有模型。

## 7. VIO 研究

VIO 依赖可用相机、低延迟且可靠时间戳的 IMU、相机内参、IMU 噪声、相机/IMU 外参和时间偏移。当前没有已验证的同步 IMU 数据源，因此尚不具备实机 VIO 验收条件。

候选 OpenVINS，先在电脑用公开或自采录制数据评估，资源允许后再构建 ARM64 最小 C++ 组件。参考 [OpenVINS](https://github.com/rpng/open_vins/) 和 [标定要求](https://github.com/rpng/open_vins/blob/master/docs/gs-calibration.dox)。是否支持所选 ROS2 Humble 分支需构建前复核。

首轮目标图像 20 Hz、IMU 100–200 Hz（均为预算，不是已测能力）；以时间同步误差、漂移、重定位/重置、CPU 和内存为验收指标。延迟较大的普通 MAVLink 遥测 IMU 不能自动视为合格 VIO 输入。先关闭 YOLO 单独测 VIO，再评估并行负载；不在板上运行仿真。

## 8. 资源预算与验收顺序

实际系统约 5.72 GiB RAM；初始预留至少 1.5 GiB 给系统和峰值，不以 swap 作为实时内存。640×480 BGR 单帧约 0.88 MiB，15 FPS 单路约 13.2 MiB/s，跨进程复制会放大负载。视觉先用 2 个计算线程、单模型、浅队列，日志限时采样；长时间日志应按大小轮转并保留历史，策略需另行配置。

1. 系统和依赖自检通过，记录现存服务故障。
2. UART7 设备树映射、普通用户打开、COM25 双向收发验证。
3. 相机枚举、格式与帧率、真实快照及稳定采集。
4. ROS2 本机两进程消息传递，再测试网络 QoS。
5. 飞控只读 HEARTBEAT 与限量遥测。
6. YOLO 小模型输出对齐和资源实测。
7. 同步 IMU/相机数据采集、标定、离线 VIO 与 ARM64 实测。
8. 最后才进入移除桨叶的受控台架指令测试及飞行前验收。
