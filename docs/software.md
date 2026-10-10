# 软件与运行方式

更新：2026-10-09。

使用已有Python 3.10环境 `/home/pi/venv`，已验证OpenCV 5、NumPy 2.2.6、pyserial 3.5、pymavlink 2.4.49。本次不安装或升级依赖。系统ROS 2 Humble与视觉虚拟环境分开，尚未用于控制飞控。

`python3 test.py`自动切换虚拟环境；`settings.py`保存UART7/115200、网页8080等配置。相机与检测参数由 `/home/pi/cv_work/config.py` 提供，当前两处帧率配置均为30，是配置上限，不等于实测FPS。

`link.py`在导入pymavlink前选择当前进程MAVLINK20=1，不改系统全局环境或飞控保存参数。工程入口先导入link；外部程序提前导入pymavlink时需核对方言版本。

Offboard按custom_mode的主模式字节6识别，兼容当前固件未解锁base_mode=17、已解锁145；其他模式沿用pymavlink解析。回执与实际心跳状态分别核对；超时不等于命令未执行，不能盲目重复动力命令。

串口线程异常后尝试重连。实机进程退出/重启验收见验证报告。默认网页和控制预览不发送运动指令；IMAGE_TO_BODY=None，未完成视觉方向标定，也不把像素坐标当成米制位置。

`verify.py`用PTY覆盖参数/ACK、超时、速度字段、心跳失联、预览抑制及合成AprilTag，不占用真实UART7。部分旧诊断脚本按需运行仍可能写logs，日常入口无常驻日志服务。
