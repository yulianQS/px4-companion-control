# 失联恢复与继续验收

**状态更新：本轮已经重新连接，UART7=/dev/ttyS7 且 COM25 双向收发通过，无需执行下方回滚步骤。**下方保留上轮失联时的恢复方案以备后续需要，不能把它当作当前故障状态。

2026-09-21 约 18:56（北京时间）配置 UART7 并重启。重启前 SSH 正常；之后 192.168.137.3 不响应 SSH/ICMP，共享子网的一次 TCP22 检查未发现 SSH 服务，COM25 没有启动/登录输出。

## 先恢复实际连接

1. 现场检查开发板电源、指示灯和电脑热点连接状态；先观察是否完成启动并重新获得 IP。没有改动热点或板端网络设置。
[Local credential omitted from publication.]
3. 无法联网时，用经过核对的 UART0 调试控制台或本地显示/键盘查看启动；COM25 目前只知道是用户给出的串口模块，不能断言接在 UART0。UART0 的启动波特率为 115200、8N1。
4. 若不能正常启动，关机后读取 SD 卡 boot 分区，或通过可用本地控制台回滚本次 UART7 设置。

## 保留当前配置后回滚

以下命令仅在重新获得开发板终端、确认目标文件后执行；恢复前检查备份与现状差异。恢复备份会撤销备份之后的其他设置，因此若有人后续修改过该文件，应仅手工移除 `overlays` 中的 `uart7` 项。

```bash
cd /home/pi/drone_project
sudo diff -u config/backups/orangepiEnv.txt.20260921T105625476535Z /boot/orangepiEnv.txt
sudo cp -a /boot/orangepiEnv.txt "config/backups/orangepiEnv.before-recovery.$(date -u +%Y%m%dT%H%M%SZ)"
# 仅当差异只有本次 overlays=uart7 时：
sudo cp -a config/backups/orangepiEnv.txt.20260921T105625476535Z /boot/orangepiEnv.txt
sudo reboot
```

若 SD 卡 boot 分区可直接访问，也可先另存 orangepiEnv.txt，再删除仅包含 `overlays=uart7` 的新增行；保留其余启动参数。

## 继续任务

本地尚未部署的 README、docs、models/README.md 和 scripts/run_checks.sh 需在确认远程不存在同名文件后复制到相应目录；不要覆盖已有文件。不要再次运行 enable_uart7.py，除非回滚后已有明确证据说明可以重试。

重新登录使 dialout 组生效；用 `id` 检查。部署剩余文件后运行：

```bash
cd /home/pi/drone_project
bash scripts/run_checks.sh
```

该脚本追加日志，UART 只接收，不发送飞控命令。相机多个视频节点时先查 `v4l2-ctl --list-formats-ext`，再指定 capture 节点运行 `camera_test.py --device /dev/videoX`。设备未枚举时不要制造测试图片。
