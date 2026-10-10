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

1. `pi` 原组为 pi、sudo；已通过 `usermod -aG dialout pi` 加入 dialout，并使用 `id pi` 验证。
2. `/boot/orangepiEnv.txt` 追加 `overlays=uart7`。修改前备份在开发板 `/home/pi/drone_project/config/backups/orangepiEnv.txt.20260921T105625476535Z`。
3. 系统原装 `sun60i-a733-uart7.dtbo` 已离线应用至原装 Zero 3W DTB，验证 UART7 status=okay。未替换原始 DTB。
4. 临时 `modprobe imx219`、`modprobe vin_v4l2` 用于诊断与当前 DT 匹配的摄像头；未写入开机加载配置，重启后不会由本次操作自动加载。
5. 执行过一次受控重启以应用 UART7；之后 SSH 失联，尚未完成重启后验收。

未修改系统 Python、ROS2 安装、网络、SSH 配置、`.bashrc` 或飞控参数。启动配置备份是恢复前提，恢复说明见 recovery.md。

## 现存异常

重启前系统为 degraded：dnsmasq.service、run-rpc_pipefs.mount 已失败。详情保存在开发板 logs/preexisting_service_failures.txt。没有足够证据认定它们与当前目标相关，未改动网络或 NFS 服务。

system_check.sh 已在开发板运行并输出 logs/system_status.txt；其中调用了 ros2 --help。完整 ros2 doctor --report 和两进程 DDS 检查尚未执行，因此不能将 ROS2 标记为验收通过。run_checks.sh 已在本地准备，连接恢复后部署并执行。
