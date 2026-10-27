# 相机内参与视觉距离 PID

内参来自独立标定程序会话 `20261012T114159-e85dca89`：640×480，28张，棋盘格方格20mm，总RMS 0.500796455 px。
`config/camera_intrinsics.yaml`供机载C++加载，`config/camera_intrinsics_source.json`记录完整求解结果。标定程序继续独立存在；不再需要运行它来提供内参。

用户确认 AprilTag 36h11 ID0 的黑色正方形边长120mm，不含外侧白边；相机朝机头正前方、画面保持原始方向。棋盘格方格20mm与AprilTag边长120mm是不同参数。

## 本地链路与坐标

相机 BGR → C++ AprilTag四角点 → 内参、5项畸变、`solvePnP(IPPE_SQUARE)` → ROS Target2D → `/dev/shm/drone_vision/status.json` → C++20Hz控制线程 → UART MAVLink → PX4。
视觉状态改为约30Hz。网页仅显示、启动/停止任务及提交设置，关闭网页不会中断任务。

像素误差沿用旧接口：`ex=(u-(W-1)/2)/(W/2)`，`ey=(v-(H-1)/2)/(H/2)`。右/下误差为正，对应机体向右偏航和向下速度。
前向距离是相机光轴方向的深度 `translation_m[2]`，不是斜距或机体中心到标记的欧氏距离；未加入相机安装位置平移补偿。
距离误差 `ed=当前前向距离-目标前向距离`，正值前进、负值后退。
机体前向速度按当前航向转换到LOCAL_NED：`vx=v_forward*cos(yaw)`，`vy=v_forward*sin(yaw)`。移动时忽略XY位置设定，停止前后移动时捕获当前XY保持，避免返回启动原点。

## 网页调参

打开8081网页 → 运动控制 → 视觉控制。支持偏航、升降、距离三组Kp/Ki/Kd，以及升降/前后开关、目标距离、标记边长、三轴速度幅值。

- 控制输出 `Kp*e + Ki*积分(e*dt) + Kd*(e-e_previous)/dt`；首次没有D项跳变。
- 原接口 `yaw_gain`、`vertical_gain`继续作为Kp；默认0.3、0.15。距离Kp默认0.3。所有Ki、Kd默认0。
- `target_distance_m`默认1.0，`tag_size_m`默认0.12。
- `yaw_speed_rad_s`、`vertical_speed_m_s`、`forward_speed_m_s`是用户选择的速度幅值，默认0，表示不截断PID输出；大于0时按该幅值输出饱和。没有固定速度上限。
- 停止状态“保存为默认值”将PID、速度、距离、码纸尺寸和轴开关写入开发板`config/vision_control.json`。网页重新打开和服务/开发板重启后读回同一文件，浏览器缓存只兼容尚未首次保存的旧配置。运行中“应用并保存为默认值”先发送`VISION_UPDATE`，机载确认后再持久化；应用或保存失败明确显示，不报告成功。C++新建视觉任务也读取此文件，任务显式参数覆盖保存的默认值。参数不会写入PX4内部PID。
- 保留之前的任务互斥和飞行状态检查；未新增死区、加速度斜坡、积分限幅、时长、位移、网页租约要求。仍按原行为在无目标时等待并保持；没有有效米制位姿时前后输出为0，像素偏航/升降继续。
- 没有持久化配置文件时，旧客户端未传`distance_enabled`仍默认关闭；存在文件时使用已保存的值。新网页首次默认勾选。

## 验证

`vision_pose_test`使用本次实际内参与畸变，验证不同距离/倾角的PnP和物理尺寸尺度。
`verify_vision_pose_graph.py`用独立ROS域的合成码纸验证位姿传到本地状态。
`verify_vision_pid.cpp`验证P/I/D和用户速度幅值。
`verify_vision_distance.py`用新建PTY模拟飞控验证前进、后退、航向旋转、MAVLink掩码、实时调参及目标丢失/恢复。此脚本不打开真实相机或飞控串口。
真实现场距离精度和飞行动态响应需另行实测。

默认参数持久化接口：`GET/POST /api/vision/control-config`，保存使用同目录临时文件、fsync和原子替换。`DRONE_VISION_CONTROL_CONFIG`可指定独立测试配置路径。
