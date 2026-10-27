# 硬件与当前端口

更新：2026-10-09。以当前实测和用户确认的接线为准。

| 项目 | 当前信息 |
|---|---|
| 机载计算机 | Orange Pi Zero 3W A733，ARM64，Ubuntu 22.04，Linux 6.6.98-sun60iw2 |
| 飞控 | MicoAir NxtPX4v2，HW HKUST_NXT_DUAL |
| 固件 | PX4 1.14.3，micoair743-v1.14.3，git `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c` |
| 伴随链路 | 开发板/dev/ttyS7 ↔ 飞控TELEM1，115200、8N1、无软/硬件流控 |
| UART7映射 | serial7 → uart@7080000；PL2/PL3、s_uart0；物理针16/18为旧手册核对结果 |
| 飞控TELEM1 | 飞控侧/dev/ttyS1，MAVLink实例0，Onboard |
| 飞控USB | 电脑COM24，与开发板tty编号无数字对应关系 |
| COM25 | 用户确认连接开发板调试、与当前UART7无关，不承担供电；不沿用早期测试接线 |
| 光流/测距 | MTF-02/02P，飞控侧/dev/ttyS3，115200，MAVLink实例1、源1/88 |
| 前置相机 | IMX219，CAM1/MIPI、/dev/video0，识别结果经 C++ 视觉控制参与任务执行 |
| 电调 | 用户确认微空33A四合一、Bluejay 48k；未确认可用RPM反馈 |
| 磁力计 | 当前未发布sensor_mag，CAL_MAG IDs曾全为0，EKF航向未对齐 |

飞控目标1/1，工程源42/191。UART7只能由一个进程持有，网页、诊断、电机测试不能并行打开它。

用户的无桨确认仅适用于明确授权的本轮台架操作，不是未来动力测试的永久前提。停转需结合回执、心跳和现场观察。
