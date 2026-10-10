# IMX219调用

更新：2026-10-09。

CAM1的MIPI相机通过已有全志V4L2/ISP实现采集，再交给OpenCV，不能简单替换为cv2.VideoCapture(0)。

日常运行根目录 `python3 test.py`，复用 `/home/pi/cv_work/camera.py` 与vision.py。终端打印网页地址；MJPEG显示图像，JSON显示检测和飞控状态。默认不把视觉结果用于飞行控制。

cv_work/config.py当前配置：/dev/video0、640×480、处理上限30 FPS、AprilTag 36h11 ID0。使用已有/home/pi/venv，不需要图形桌面。

历史实测采集640×480图像、网页解码成功，合成ID0检测通过；现场码纸识别需要实际画面确认。本次文档整理未重新采集，不能把历史成功当成当前运行状态。

重启后节点缺失先只读检查：

```bash
ls -l /dev/video0
v4l2-ctl --list-devices
```

`-l`显示权限，`--list-devices`列出设备。区分驱动/传感器枚举和应用读取故障，不盲目重装OpenCV或改设备树。旧scripts/imx219_*是按需诊断，可能生成文件，不是日常启动方式。

不要同时启动cv_work和drone_project，它们共享相机与8080。Ctrl+C退出；MIPI排线调整必须断电。
