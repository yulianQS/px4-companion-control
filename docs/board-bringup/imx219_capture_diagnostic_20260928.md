# IMX219 拍照与断联诊断（2026-09-28）

## 实测结论

CAM1 的 IMX219 已识别，I²C 11 的 7 位地址 0x10 已被驱动占用，媒体拓扑包含 `imx219`，输出节点为 `/dev/video0`。使用板载全志多平面 V4L2/ISP 适配器采集，再用虚拟环境 OpenCV 5.0.0 的 `cv2.imwrite()` 保存 JPEG，两次均成功；没有 GUI。

| 检查 | 结果 |
|---|---|
| 系统 | Ubuntu 22.04 / ARM64 / Linux 6.6.98-sun60iw2 |
| 物理接口 | CAM1；sensor0 / MIPI0 / CSI0 / ISP0 / VIN0 |
| IMX219 控制总线 | I²C 11，地址 0x10（加载后扫描显示 UU） |
| 内核识别 | `[imx219]find the sony IMX219` |
| 视频节点 | `/dev/video0`，驱动 sunxi-vin，Video Capture Multiplanar |
| 协商输出格式 | RGB3，640×480，1 个内存平面，1920 字节/行，921600 字节/帧 |
| 传感器输入 | 内核记录 1920×1080；ISP 缩放为 640×480 输出 |
| 第一次采集 | 8 帧成功，保存最后一帧；2026-09-28 13:23:53 UTC |
| 第二次采集 | 60 帧成功，约 2 秒曝光调整，保存最后一帧；13:28:04 UTC |
| 网络与重启 | 两次采集后 SSH 正常；boot ID 始终为 `1af3e3b4-8d49-488e-ba1e-267afc069e16` |

第二张照片为 `logs/imx219_20260928/imx219_20260928T132804518717Z.jpg`，对应北京时间 21:28:04。已解码检查为 640×480 的 3 通道图像，87,027 字节。镜头朝向天花板灯具，强光区域过曝，周围物体可见；没有旋转、亮度修饰或生成画面。

## 上次断联：事实与推断

1. 上次执行 `v4l2-ctl -d /dev/video0 --all` 后，命令无输出，随后 Ping 和 SSH 不可达。后续单帧采集命令尚未开始，所以不能将上次故障归因于 OpenCV 或图像流。
2. 用户本轮确认当时手动重启了开发板。此前“开发板自动重启”的说法应撤回；现有证据是查询后失联，手动重启后恢复。
3. 系统 journal 没有跨启动持久保存，`/sys/fs/pstore` 不存在；上次的内核异常栈已无法恢复。因此不能声称已实证确定 panic、看门狗复位、供电故障或确切的异常指令。
4. 本轮第一次实际采集的记录表明 `open(/dev/video0)`、`VIDIOC_S_INPUT`、`VIDIOC_S_PARM`、`VIDIOC_S_FMT`、申请与映射缓冲区、ISP 初始化、`VIDIOC_STREAMON`、取帧、`VIDIOC_STREAMOFF` 均返回成功。这纠正了此前“打开视频节点本身导致故障”的过度判断。
5. 完成厂商顺序的初始化后，本轮以 ioctl 日志追踪再次执行同一 `--all` 查询，命令返回 0，SSH 正常，boot ID 不变。日志仍包含不支持的格式/控制请求返回 EINVAL 或 ENOTTY。

**最有依据的解释是全志 VIN 驱动在尚未通过 S_INPUT 初始化传感器管线时，完整控制项查询进入了未就绪的驱动路径。此解释有运行对照与源码依据，但上次没有异常栈，仍属于推断，不能写成已证实的唯一根因。**

对 OrangePi 官方 `orange-pi-6.6-sun60iw2` 分支的只读源码检查显示：

- `vin_open()` 设置 BUSY/LPM 状态和事件队列，未在这里选择、打开传感器管线。
- `__vin_s_input()` 才设置 sensor/CSI/ISP 链路、打开管线并初始化 ISP 和传感器。
- `vin_g_volatile_ctrl()` 的曝光、增益查询直接访问 `sensor->ctrl_handler`，相关分支没有先判断 sensor 是否为空。
- `vidioc_g_parm()` 将管线中的 sensor 指针传给 `sensor_g_parm()`；后者直接读取传感器结构字段，也没有入口空指针检查。

这使“先 --all 查全量状态，随后再初始化”的顺序存在风险。官方分支源码并非本机二进制逐字节重建结果，所以源码线索不替代上次缺失的异常栈。本轮没有为了复现失联而重置驱动后再次执行危险的全量查询。

源码依据：[vin_video.c](https://github.com/orangepi-xunlong/linux-orangepi/blob/orange-pi-6.6-sun60iw2/bsp/drivers/vin/vin-video/vin_video.c)、[sensor_helper.c](https://github.com/orangepi-xunlong/linux-orangepi/blob/orange-pi-6.6-sun60iw2/bsp/drivers/vin/modules/sensor/sensor_helper.c)。只读副本放在 `vision/camera/diagnostic/kernel_source/`，供后续比对。

## 测试程序的处理

- `scripts/imx219_snapshot.py`：推荐入口。通过 media topology 验证 IMX219 和 CAM1 节点；用已安装的厂商 V4L2/ISP 包装类采集 BGR8，再由 OpenCV 保存和重新解码核验 JPEG。未使用 `cv2.VideoCapture(0)`。
- `scripts/imx219_test.py`：保留原参数、日志和默认输出路径，移除初始化前 `--all` 和格式枚举；改用 media topology 与 sysfs 验证 CAM1，格式由厂商 helper 在 S_INPUT 后协商。
- `scripts/build_imx219_diagnostic.sh`：只使用现有编译器和库。生成带逐步追踪的厂商源码副本与 helper，60 帧曝光调整；保留原厂商源码和原 helper。
- 原始旧脚本保存在 `config/backups/imx219_20260928/`。
- 所有新测试文件均在 `/home/pi/drone_project` 内；未在 `/tmp` 或用户目录根部散放测试文件。

运行顺序：

```bash
cd /home/pi/drone_project
# 重启后摄像头模块未自动加载时，先执行：
sudo modprobe vin_v4l2
bash scripts/build_imx219_diagnostic.sh
timeout --kill-after=3 20 /home/pi/venv/bin/python -u scripts/imx219_snapshot.py --device /dev/video0
```

脚本本身有 15 秒子进程期限、输出尺寸/通道检查、唯一时间戳输出和逐步日志同步。用户态超时不能保护内核自身发生的死锁或异常。工程默认不再用启动后的 `--all` 作为拍照前置条件。

## 留存的日志与图片

```text
/home/pi/drone_project/
├── scripts/
│   ├── imx219_snapshot.py
│   ├── imx219_test.py
│   └── build_imx219_diagnostic.sh
├── vision/camera/
│   ├── diagnostic/{camera_trace.h,v4l2_camera_trace.cpp,imx219_grab_trace.cpp}
│   ├── diagnostic/{v4l2_ioctl_trace.c,v4l2_ioctl_trace.so,kernel_source/}
│   └── bin/imx219_grab_trace
├── logs/imx219_20260928/
│   ├── attempt1.txt / attempt2.txt
│   ├── capture_*.txt / media_*.txt
│   ├── query_after_init.txt
│   ├── kernel_before.txt / kernel_live.txt / kernel_final.txt
│   ├── serial_console.txt
│   ├── frame_*.bgr
│   └── imx219_*.jpg
├── docs/imx219_capture_diagnostic_20260928.md
└── config/backups/imx219_20260928/
```

COM25 在用户释放后可以打开。本轮监听没有收到串口文本；启动参数中的内核控制台是 `ttyS0`，此前 COM25 与 UART7 连接验证不能证明它当前接到调试控制台。因而本轮串口空日志不构成“内核未报错”的证据；主要依据为内核实时记录、采集程序记录与 boot ID。

## 剩余问题与限制

- ISP 用户库仍报告 TDM event subscription、部分 AEWB stats、翻转/温度/debug ioctls 不支持，并回退到其他 IMX219 tuning profile。它们没有阻止本轮取帧，但需要在连续视觉流、曝光和颜色质量测试前继续处理。60 帧后照片亮度明显改善，不等于全部 ISP 3A 功能已经验证。
- ISP 库尝试写 `/mnt/isp0_1920_1080_1_0_imx219_ctx_saved.bin` 被权限拒绝；没有通过 sudo 运行视觉程序，也未创建这个系统目录缓存。工程自行生成的文件全部保留在用户项目目录。
- 内核原有 PD5/PWM 冲突和 thermal cooling_device9 告警在拍照前已存在，本轮仍存在。当前温度约 55–57°C；没有证据证明这些告警导致上次失联，本轮未改引脚或散热配置。
- CAM2 未接入，`imx219_2` 未识别属于当前连接条件下的预期现象。
- 本轮是两次有限时长的取帧验证，尚不是长时间稳定性认证。

未安装、升级或删除软件；未修改 DTB、overlay、系统核心或网络配置；未向飞控发送消息。
