# CAM1 检查与后续拍照

依据用户提供的 `F:\zilao\OrangePi zero3w\OrangePi_Zero3W_A733_用户手册_v1.0.pdf` 第 3.31 节，印刷页 165–169（PDF 页 172–176）：支持 IMX219，接口为 22PIN 0.5mm，需匹配的 22PIN 转 15PIN 排线；CAM1 对应 /dev/video0，CAM2 对应 /dev/video8。

手册中的安装和桌面预览命令是参考步骤，不是本次用户要求；本次没有执行 apt 安装、make clean、递归 chown 或 GUI 预览。Jammy 不使用手册仅适用于 Debian Bullseye 的 GStreamer 流程。

## 当前阻塞

已确认连接位置为 CAM1，执行手册的 `sudo modprobe vin_v4l2`。内核仍显示 IMX219 芯片识别失败（`No such device`），没有 /dev/video0。硬件探测失败发生在 OpenCV 之前；本次没有生成 test_camera.jpg，也不能报告实际分辨率或帧率。只出现 /dev/media0 不代表相机已就绪。

进一步的媒体拓扑检查确认：`sunxi_csi.0/1/2`、`sunxi_mipi.0/1/2`、`sunxi_isp.0` 和 scaler 实体已经注册，但没有 `imx219` sensor 实体；MIPI 实体没有 sensor link，ISP pad 为 `fmt:unknown/0x0`。设备树中 CAM1 的 `sensor0_mname=imx219`、`status=okay`，对应 VIN core 0；驱动读取 CAM1 芯片 ID 时失败。CAM2 的 `imx219_2` 失败符合 CAM2 未连接现状，不能用它判断 CAM1。

因此当前结论是：CSI/ISP 软件控制器已注册，但相机端到端链路未建立；无法验证 CSI-2 电气数据传输和 ISP 出图。最早的失败点是 CAM1 IMX219 芯片识别，后续 V4L2/OpenCV 不能绕过这一点。

请在断电状态核对实物模组是否 IMX219、转换排线的针数与接触面方向、两端锁扣、完整插入及供电；按手册连接照片逐项比对。不要带电插拔 CSI 排线。若是其他传感器，需先核实其型号、驱动与设备树支持，不能直接套用 IMX219 配置。

## 已准备的软件路径

项目保留系统 `/opt/v4l2_opencv_demo` 原文件，在 `vision/camera/vendor/` 复制所需的 v4l2_camera.cpp/h；版权标记随源文件保留。`cam1_snapshot.cpp` 使用厂商多平面 V4L2 / ISP 接口及 C++ OpenCV，无 GUI，初始请求 640×480 RGB24，经厂商类转换为 BGR8，采集有限帧并保存 JPEG。

```bash
cd /home/pi/drone_project
bash scripts/imx219_check.sh
bash scripts/build_imx219_helper.sh
# 摄像头已经枚举后：
timeout 25s /home/pi/venv/bin/python scripts/imx219_test.py
```

编译脚本只使用已有头文件和动态库，不安装软件。Python 与 C++ 通过子进程/图片文件隔离，不借助旧版 pybind11 传递 NumPy 2.x 数组。JPEG 以独占创建方式写入，已有图片会使用新时间戳名称；日志追加。

初次枚举后，先运行 `v4l2-ctl -d /dev/video0 --all` 和 `--list-formats-ext` 确认能力，再调整分辨率。若节点为 root:video 且 pi 无权限，先检查组成员后再添加 video 组；不要用 sudo 运行全部视觉算法来掩盖权限问题。

编译成功不等于采集成功；CAM1 helper 的真实图像、ISP 3A 和长时间稳定性仍需硬件枚举后测试。本项目的 CAM1 流程只按 MIPI CSI-2/全志 ISP 处理。

`imx219_test.py` 不使用 `cv2.VideoCapture(0)`。它先要求媒体拓扑存在 IMX219 sensor 实体，再检查 `/dev/video0` 同时出现在拓扑中且声明 V4L2 capture 能力，查询格式后才调用全志多平面采集 helper。helper 输出一帧 BGR8 原始数据，Python 校验尺寸并用 `cv2.imwrite()` 保存 `logs/imx219_test.jpg`，全程无 GUI。当前实测因 sensor 实体缺失返回 3，未创建图片。

## 本轮构建验证

已在开发板编译成功，产物 vision/camera/bin/cam1_snapshot 为 ARM64 ELF，动态库解析和无参数启动正常。camera_test.py --backend sunxi --device /dev/video0 因节点不存在而明确返回 2，未生成照片。编译原始输出位于 logs/camera_build.txt。
