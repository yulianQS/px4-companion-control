# 软件环境与变更记录

## 已核实的现有软件

| 软件 | 已安装版本或位置 |
|---|---|
| 操作系统 | Orange Pi 1.0.0 / Ubuntu 22.04.5 LTS Jammy |
| Python | 3.10.12；视觉虚拟环境 /home/pi/venv |
| OpenCV | import cv2 显示 5.0.0；opencv-python 与 opencv-contrib-python 都为 5.0.0.93 |
| NumPy / SciPy | 2.2.6 / 1.15.3 |
| Pillow | pip list 为 12.3.0 |
| pyserial / pymavlink | 3.5 / 2.4.49 |
| ROS2 | /opt/ros/humble；ros-humble-ros-base 0.10.0-1jammy.20260910.003535 |
| 系统 NumPy | apt python3-numpy 1.21.5-1ubuntu22.04.1 |
| v4l-utils | 1.22.1-2build1 |
| DT 工具 | dtc、fdtoverlay、fdtget 已存在 |
| 厂商示例 | /opt/v4l2_opencv_demo、/opt/yolov5、/opt/vpm_run，保持原位 |

本次没有安装、升级、删除软件，也没有安装大型 GUI、仿真或训练框架。两个 OpenCV distribution 共享 cv2 命名空间，先保持现状并记录，不擅自卸载。Python 核心库导入已成功；模型推理没有执行。

## 实际系统变更

1. `pi` 原组为 pi、sudo；已加入 dialout 和 video，并在新登录会话中通过 `id` 验证。dialout 用于 UART7，video 用于 V4L2/media 节点。
2. `/boot/orangepiEnv.txt` 追加 `overlays=uart7`。修改前备份在开发板 `/home/pi/drone_project/config/backups/orangepiEnv.txt.20260921T105625476535Z`。
3. 系统原装 `sun60i-a733-uart7.dtbo` 已离线应用至原装 Zero 3W DTB，验证 UART7 status=okay。未替换原始 DTB。
4. 临时 `modprobe imx219`、`modprobe vin_v4l2` 用于诊断与当前 DT 匹配的摄像头；未写入开机加载配置，重启后不会由本次操作自动加载。
5. 上轮执行过一次受控重启后暂时失联；本轮已重新连接，UART7 运行时映射和 COM25 双向通信验收通过。本轮未再重启。

未修改系统 Python、ROS2 安装、网络、SSH 配置、`.bashrc` 或飞控参数。启动配置备份是恢复前提，恢复说明见 recovery.md。

## 现存异常

重启前系统为 degraded：dnsmasq.service、run-rpc_pipefs.mount 已失败。详情保存在开发板 logs/preexisting_service_failures.txt。没有足够证据认定它们与当前目标相关，未改动网络或 NFS 服务。

system_check.sh 与 run_checks.sh 已部署并执行。虚拟环境 pip check 输出 No broken requirements found。ros2 --help、rclpy 导入、本机两个独立进程的 DDS 消息传递全部通过，使用 rmw_fastrtps_cpp。首次 ros2 doctor --report 的 PackageReport / RosdistroReport 有警告；单独查询发行版包信息成功后复查，完整报告返回 0 且不再有警告，包括 PACKAGE VERSIONS 与 ROS 2 INFORMATION。没有为此更改系统设置或升级软件。见 ros2_status.txt、ros2_report_retry.txt 和 ros2_package_report_diagnostic.txt。本机消息测试不代表飞控集成或跨机 ROS2 网络测试通过。

当前已有 C++ OpenCV 4.5.4 开发库、g++、AWIspApi.h 和 libAWIspApi/libisp/libisp_ini。系统 Python 缺少 cv2 模块，但视觉 venv 的 cv2 5.0.0 导入正常；不因此重复安装 Python OpenCV。CAM1 的多平面 V4L2/ISP 采集使用独立 C++ helper 与现有 C++ 库，Python 脚本负责调用及快照检查，避免混用 C++ OpenCV 4.x 和 Python OpenCV 5.x 二进制接口。

CAM1 helper 已使用现有依赖成功编译为 ARM64 ELF，可运行并显示参数说明；无实际相机节点，硬件采集尚未通过。

新增 `imx219_check.sh`、`imx219_test.py`、`build_imx219_helper.sh` 以及 `vision/camera/imx219_grab.cpp`。检查脚本复现 MIPI CSI/Media/ISP 状态；测试脚本在媒体拓扑确认 sensor 和 capture 节点之前拒绝采集，不使用 `cv2.VideoCapture(0)`。采集 helper 只输出原始 BGR8 帧，最终 JPEG 由 Python `cv2.imwrite()` 写入。当前实际运行返回 3（sensor entity missing），没有创建假图片。
