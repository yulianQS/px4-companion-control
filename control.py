"""视觉控制预览：只计算，不发送设定值。像素不冒充定位。"""
import math
import time
import settings


def preview(detection, px4, now=None):
    now = time.monotonic() if now is None else now
    result = {"mode": "preview", "transmitting": False,
              "body_velocity_m_s": [0.0, 0.0, 0.0], "reason": None}
    if not detection.get("camera_ready"):
        result["reason"] = "摄像头未就绪"
    elif now - detection.get("received_at", 0) > settings.VISION_MAX_AGE:
        result["reason"] = "视觉数据过期"
    elif not detection.get("target_found") or not detection.get("center"):
        result["reason"] = "未发现目标 ID 0"
    else:
        width, height = detection["width"], detection["height"]
        cx, cy = detection["center"]
        error = [(cx - width / 2) / (width / 2),
                 (cy - height / 2) / (height / 2)]
        result["image_error_normalized"] = error
        matrix = settings.IMAGE_TO_BODY
        if matrix is None:
            result["reason"] = "图像到机体的方向映射尚未标定"
        elif detection.get("cpu_temp_c", 0) is not None and detection["cpu_temp_c"] >= settings.TEMPERATURE_LIMIT:
            result["reason"] = "开发板温度过高"
        elif not px4.get("connected") or not px4.get("bidirectional"):
            result["reason"] = "飞控双向链路未通过"
        elif (now - px4.get("estimator_at", 0) > 2
              or px4.get("estimator_flags", 0) & 12 != 12
              or px4.get("estimator_flags", 0) & 128):
            result["reason"] = "缺少新鲜且有效的水平/垂直速度估计"
        else:
            velocity = [-settings.VISION_GAIN * sum(a * b for a, b in zip(row, error))
                        for row in matrix]
            if len(velocity) != 2 or not all(math.isfinite(v) for v in velocity):
                result["reason"] = "方向映射无效"
            else:
                norm = math.hypot(*velocity)
                scale = min(1, settings.VELOCITY_LIMIT / max(norm, 1e-9))
                result["body_velocity_m_s"] = [v * scale for v in velocity] + [0.0]
                result["reason"] = "预览计算正常；未发送飞行指令"
    return result

