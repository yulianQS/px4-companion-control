# OrangePi Zero 3W 无人机伴随计算工程

2026-09-21。**当前为部分完成：UART7 配置重启后开发板失联，ROS2 完整检查、UART7 实际 tty 验证和摄像头拍照尚未完成。**

开发板目标目录 `/home/pi/drone_project`。本地交付目录 `F:\codex_pro\outputs\orangepi-drone-20260921\drone_project`。本地目录不是开发板日志的完整镜像；远程原始日志因失联尚未下载。

```text
drone_project/
├── README.md
├── docs/
│   ├── hardware.md
│   ├── software.md
│   ├── development_plan.md
│   └── recovery.md
├── scripts/
│   ├── system_check.sh
│   ├── camera_test.py
│   ├── uart_test.py
│   ├── hardware_probe.py
│   ├── ros2_smoke.py
│   ├── enable_uart7.py
│   └── run_checks.sh
├── ros2_ws/src/
├── vision/{camera,opencv,yolo,vio}/
├── mavlink/{uart,pymavlink}/
├── config/backups/
├── logs/
└── models/README.md
```

## 已在开发板创建

上述全部工程目录；scripts 中除 run_checks.sh 外的六个脚本。生成的原始日志：system_status.txt、precheck_console.txt、camera_status.txt、uart_status.txt、preexisting_service_failures.txt、configuration_changes.txt。config/backups 中有修改前启动配置与离线验证 DTB。

## 本地完成但尚未部署

README.md、docs 中四份文档、models/README.md、scripts/run_checks.sh。本地额外保存 logs/windows_com25_status.txt；这些文件不表示远程已完成对应验收。

## 已验证与待处理

- SSH/sudo、系统信息、已有 Python 库版本和导入已验证。没有新增软件。
- pi 已加入 dialout；已备份并启用系统自带 UART7 overlay。离线 DT 验证通过，重启后实际设备名未验证。
- 摄像头驱动探测 IMX219 失败，无 /dev/video*，没有 test_camera.jpg。
- COM25 可打开，无回传数据；尚未证明它与 UART7 双向连通。
- ROS2 Humble ros-base 已安装；doctor/DDS 测试尚待连接恢复。
- 已存在 dnsmasq 和 rpc_pipefs 服务失败，保留并记录。

先阅读 [恢复说明](docs/recovery.md)，再查看 [硬件状态](docs/hardware.md)、[软件状态](docs/software.md) 与 [开发方案](docs/development_plan.md)。

## 连接恢复后的诊断

```bash
cd /home/pi/drone_project
bash scripts/system_check.sh
timeout 20s /home/pi/venv/bin/python scripts/camera_test.py
timeout 10s /home/pi/venv/bin/python scripts/uart_test.py
# 部署 run_checks.sh 后执行完整日志采集和 ROS2 检查：
bash scripts/run_checks.sh
```

脚本不安装依赖；日志追加写入，快照不覆盖已有图片。UART 测试依据运行中设备树自动定位，拒绝猜测或打开系统控制台。不要用 sudo 运行相机/串口脚本来掩盖权限问题。enable_uart7.py 是本次受控配置脚本，不能代替故障诊断反复执行。
