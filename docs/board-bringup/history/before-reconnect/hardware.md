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
| UART7 | DT serial7 → /soc@3000000/uart@7080000；初始 disabled，无对应 tty 节点；引脚组 PL2/PL3，function=s_uart0 |
| UART7 配置 | 已追加 overlays=uart7；fdtoverlay 离线应用成功且节点 status=okay；已重启，但网络未恢复，实际 /dev/tty 名仍待验证 |
| 电脑串口 | COM25，USB VID_2E3C PID_5740，序列号 0591D2A91E28；115200 8N1 可打开，无接收数据 |
| USB 摄像头 | lsusb 只列出根 hub，未枚举外接 USB 摄像头 |
| CSI 摄像头 | DT 预设 imx219 / imx219_2；临时加载系统已有驱动后均报 chip found is not a target chip / No such device |
| 视频节点 | 初查无 /dev/video*；驱动诊断后出现 /dev/media0，但没有可采集视频节点。未生成 test_camera.jpg |

COM25 与 UART7 的物理收发关系尚未验证。两个方向均未完成已知数据的闭环测试。COM25 曾执行只接收测试（3 秒、8 秒），随后仅发送一个回车尝试查看控制台提示，仍无输出；没有发送飞控指令。

摄像头名称、分辨率、支持格式、帧率目前均不能实测确认。需要现场核对摄像头型号、USB/CSI 接口、排线方向、连接和供电，不能据 imx219 设备树断言实物就是 IMX219。

启动配置只增加 UART7，无内核/DTB 文件替换，无网络配置改动。重启后失联的原因尚未确定，不能确认是网络重连、启动故障或 UART overlay 导致；恢复步骤见 recovery.md。
