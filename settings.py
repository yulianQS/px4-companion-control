"""工程默认参数。启动：python3 test.py；不需要命令行参数。"""
PYTHON_ENV = "/home/pi/venv"
VISION_DIRECTORY = "/home/pi/cv_work"
UART_DEVICE = "/dev/ttyS7"
UART_BAUD = 115200
SOURCE_SYSTEM = 42
SOURCE_COMPONENT = 191
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080
VISION_FPS = 10                  # 本工程上限，不修改 cv_work 的配置文件
HEARTBEAT_MAX_AGE = 3.0
VISION_MAX_AGE = 0.5
TEMPERATURE_LIMIT = 85.0
VELOCITY_LIMIT = 0.2             # m/s，未来启用闭环时的水平限速
VISION_GAIN = 0.15
# 图像归一化误差 → BODY 前/右速度的 2×2 矩阵。
# 相机安装方向确认前保持 None；不能凭图像像素猜飞行方向。
IMAGE_TO_BODY = None

