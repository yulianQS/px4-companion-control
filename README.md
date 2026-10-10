# PX4 伴随计算机工程

更新：2026-10-10。开发板工程目录：`/home/pi/drone_project`。

## 日常启动

```bash
cd /home/pi/drone_project
python3 test.py
```

自动使用已有 `/home/pi/venv`，终端打印局域网网址，当前通常为 `http://192.168.137.3:8080/`。网页显示相机、AprilTag、飞控状态及视觉控制预览。Ctrl+C释放资源。默认不解锁、不切模式、不发送运动设定值、不生成常驻日志。

不要同时启动cv_work项目或其他占用UART7的程序。

## 真实飞控网页

`python3 web_control.py` 启动实机比赛控制台，默认8081端口；自动使用已有venv，直接连接UART7，不能与test.py或其他串口程序同时运行。展示真实遥测，离线显示未知。已接入普通解锁/地面上锁、未解锁模式切换、相对平移/转向/升降、起飞与PX4 LAND降落。运动完成由真实遥测判定。网页仅一个“启用实机控制 / 关闭实机控制”开关，无需口令、勾选或文字确认。v0.6取消网页额外的10秒自动上锁计时，PX4自身配置与检查保持原状。比赛编排、视觉闭环与PID写入仍未接通。详见[网页使用与验证说明](docs/web_prototype.md)。

Web历史实机验证完成未解锁LOITER→OFFBOARD→LOITER、ACK及新心跳确认。v0.4任务逻辑与MAVLink编码32项离线检查通过，不代表实际飞行跟踪通过；当前版本的起飞、悬停、平移、转向及降落仍需现场逐项验证。

v0.6双摇杆支持地面及未解锁OFFBOARD调试，无需先起飞：左杆升降/偏航、右杆前后/左右速度，PX4调节实际俯仰/横滚，松杆零速度。开发板20Hz统一发送；摇杆接管会取消当前自动运动或起飞任务，LAND由PX4完成。位置原点重置不再锁止手动速度摇杆。输入超过0.5秒未更新即停止旧杆量，实际解锁仍由用户手动操作。详见[双摇杆控制说明](docs/joystick_control.md)。

## 当前进度

双向MAVLink、未解锁Offboard接管/恢复、速度接收/坐标转换、无桨普通解锁/上锁及指令中断检测已实测通过。用户确认无桨电机转动且全部停转。

历史间歇性CRC/底层帧错误根因尚未确定；航向未对齐等定位限制仍需评估。台架通过不等于实际飞行通过，默认入口仍是监测与预览。

| 文件 | 作用 |
|---|---|
| settings.py | 串口、网页、帧率、预览限速 |
| link.py | 唯一串口入口、状态、回执、重连 |
| web_control.py / web_flight.py | 实机网页服务、遥测、控制权、模式、解锁和20Hz设定值发送 |
| motion_tasks.py | 任务参数、相对目标、真实反馈闭环、超时、取消及落地判定 |
| joystick_control.py / web/joystick.js | 板端摇杆输入与速度计算 / 网页双指、鼠标和键盘输入 |
| test.py | 日常入口，复用cv_work视觉实现 |
| control.py / dashboard.py | 控制计算预览 / 只读网页 |
| operator_cli.py | 独立通信诊断、显式无桨电机测试 |
| offboard_probe.py | 显式未解锁零速度模式验证 |
| scripts/offboard_restart_check.py | 显式未解锁进程退出与重新连接验证 |
| verify.py | PTY模拟与安全条件验证，不访问UART7 |
| scripts/ | 按需检查；部分旧脚本会生成诊断文件 |
| config/backups/ | 保留的启动配置恢复备份 |

只读诊断：`python3 operator_cli.py doctor`。软件检查：`/home/pi/venv/bin/python -B verify.py`。

电机测试：`python3 operator_cli.py motor-test --props-removed`，会实际驱动电机；仅在现场确认无桨、供电与停止手段后运行。相同输出不保证相同RPM。

## 文档

- [硬件与端口](docs/hardware.md)
- [软件与运行方式](docs/software.md)
- [摄像头调用](docs/camera_cam1.md)
- [当前验证结果](docs/validation_20261009.md)
- [下一阶段](docs/development_plan.md)
- [恢复与排障](docs/recovery.md)

旧原始日志和被新文档替代的报告已清理。清理前文档/日志备份：本地 `F:/codex_pro/outputs/drone_project_docs_logs_before_cleanup_20261009.tar.gz`。源码、模型与启动配置备份保留。
