# 硬件核查记录

2026-09-21，SSH 用户 pi，主机 orangepizero3w。以下是本次实际观察，不把设备树描述当成硬件实测。

| 项目 | 实际证据与状态 |
|---|---|
| 平台 | aarch64，Linux 6.6.98-sun60iw2；启动 dtb 为 sun60i-a733-orangepi-zero3w.dtb；DT model 文本仅为 sun60iw2 |
| 内存 | 5.72 GiB；初查已用约 172–173 MiB，可用约 5.5 GiB |
| 存储 | 根分区 15 GiB，已用约 4.0 GiB，可用约 9.9 GiB |
| 温度 | SSH 登录摘要约 33°C，仅为空闲采样 |
| UART0 | /dev/ttyS0，MMIO 0x02500000，内核 console=ttyS0,115200；未用作业务串口 |
| UART1 | /dev/ttyS1，MMIO 0x02501000；已有占用及蓝牙相关日志，未触碰 |
| UART7 | DT serial7 → /soc@3000000/uart@7080000 → /dev/ttyS7，运行时 status=okay；引脚组 PL2/PL3，function=s_uart0 |
| UART7 配置 | overlays=uart7 已生效；pi 普通用户打开成功；手册第 100 页注明 TX=物理针 16、RX=物理针 18 |
| 电脑串口 | COM25，USB VID_2E3C PID_5740，序列号 0591D2A91E28；115200 8N1 与 /dev/ttyS7 双向 ASCII 标记通信通过 |
| USB 摄像头 | lsusb 只列出根 hub，未枚举外接 USB 摄像头 |
| CSI 摄像头 | 用户确认 CAM1 为 Sony IMX219/MIPI CSI-2；CAM1 的 DT sensor0=imx219、VIN core 0。驱动读取芯片 ID 失败，sensor 未注册 |
| Media/CSI | `/dev/media0` 为 Allwinner Vin；CSI/MIPI/ISP/scaler 实体存在，但无 IMX219 实体、无 sensor link |
| ISP | `sunxi_isp.0` 已注册，pad 格式为 `unknown/0x0`，端到端 ISP pipeline 未就绪 |
| 视频节点 | 无 `/dev/video*`，因此没有 V4L2 capture 输出，也未生成 `imx219_test.jpg` |

COM25 → UART7 收到两条完整 `DRONE_UART7_PC_PROBE_20260921` 标记（60 字节）；UART7 → COM25 收到完整 `DRONE_UART7_BOARD_REPLY_20260921` 标记。这里只验证台架线路和 115200 8N1 设置，不代表已连接飞控或测试 MAVLink。没有发送飞控指令。

用户已确认摄像头接 CAM1 且模组为 Sony IMX219。提供的《OrangePi Zero3W A733 用户手册 v1.0》第 165–169 页（PDF 页 172–176）规定 22PIN 0.5mm 转 15PIN 排线、CAM1=/dev/video0。执行 `sudo modprobe vin_v4l2` 后 CAM1 的 `imx219` 仍然芯片识别失败，只有 `/dev/media0`；这发生在 OpenCV 之前。实际分辨率、支持格式、帧率目前均不能实测确认；需断电核对排线接触面方向、锁扣、完全插入和相机供电。

本轮已重连，SSH 正常，UART7 配置保留并实测通过。上轮短暂失联原因未确定，不能据此归因于 UART7。没有再次重启或修改网络设置。
