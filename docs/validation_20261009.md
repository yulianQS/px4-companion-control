# 当前验证结果

日期：2026-10-09。只记录实际执行的范围。

## 已通过

| 项目 | 实机证据 |
|---|---|
| 双向通信 | 参数回复、固件请求ACK=0；最近121.07秒只读60/60回复，最大延迟26.1ms |
| Offboard模式 | ACK=0，心跳custom_mode=393216；未解锁base_mode=17 |
| 水平速度接收 | BODY前0.2→[0.19562,0.04159,0]；右0.2→[-0.04155,0.19563,0]，当时估计航向约12° |
| 垂直/回零 | 向上0.1→[0,0,-0.1]；回零→[0,0,0]，读取新鲜trajectory_setpoint |
| 无桨普通解锁 | ACK=0、armed=True；约1秒后上锁ACK=0、armed=False；用户确认转动且全部停转 |
| 持续零速度 | 未解锁Offboard约20Hz持续60秒，六次固件请求ACK=0、无超时，该区间BAD_DATA增量0 |
| 中断/恢复 | offboard_control_signal_lost依次False/True/False；仅验证检测，不代表飞行保护动作 |
| 收尾 | 模式测试最后恢复初始未解锁LOITER，默认入口没有自动解锁 |

无桨确认不作为未来永久前提；相同电机输出不保证同RPM。

## 未解决与限制

- 历史间歇性损坏根因未知。电机未转动时20Hz发送8秒出现11次CRC、15次底层帧错误；5Hz也可复现，降频不等于修复。
- 同期USB无CRC，而UART7有错误，范围缩小到TELEM1、线缆/电气条件、开发板UART7硬件或驱动，尚未确定端点。COM25与当前UART7无关。
- 最新只读60/60测试无BAD_DATA，frame=440/break=34未增长。后续Offboard全程共4条BAD_DATA，内核frame最终444；没有持续区间单独内核起止值，不能称全链路零错误。
- 最新xy/z/vxy/vz valid、dead_reckoning=False，但heading_good_for_control=False、dist_bottom_valid=False，两个EKF cs_yaw_align=False；地面水平重置约每秒一次。已核对落地光流/地形逻辑，不直接认定飞行光流故障或航向导致重置。
- 原始光流/测距曾确认新鲜、range约0.69–0.74m；前置相机不是当前控制链路的定位前提。
- 最近RC/GCS均报告失联，未改对应检查或保护参数。
- 未装桨飞行、起降，未验证飞行失联动作、长期定位或真实速度跟踪。

## 软件与有限证据

PTY模拟、命令接受/拒绝/超时、速度编码、心跳失联、预览安全条件、合成AprilTag曾通过；软件模拟不能替代实机。

最近有限输出在本地F:/codex_pro/work：px4_velocity_receive_result.txt、px4_offboard_arm_result.txt、px4_continuous_readonly_result.txt、px4_offboard_sustained_result.txt、px4_usb_during_tx_parse_result.txt。工程不再保留逐轮长终端日志。

固件依据为同一git提交 `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c` 的HealthAndArmingChecks/checks/modeCheck.cpp、EKF/optical_flow_control.cpp和terrain_estimator.cpp。

## 本次下一阶段：控制进程退出与重新接管

已实机执行 `scripts/offboard_restart_check.py --unarmed-restart-check`：

1. 初始双向正常、未解锁LOITER；零速度预发送后正常进入Offboard。
2. 释放父进程UART，由本次创建的子进程重新打开，确认开始发送零速度。
3. 仅终止该子进程，模拟应用突然退出；新连接成功打开UART7并读到offboard_control_signal_lost=True。
4. 新进程恢复约20Hz零速度，读到标志False。
5. 恢复请求ACK=0，心跳确认未解锁LOITER；串口和测试子进程均已退出。

此测试不包含解锁或非零速度。重连阶段接收计数没有BAD_DATA，但未汇总子进程的全部原始字节，不声称整个过程零误码。未验证真实机载计算机断电、物理断线或飞行中的失联动作。

脚本默认运行或--help不操作硬件，只有显式测试参数才执行模式切换。其内部子进程入口不作为日常启动方式。用户运行前须退出test.py，确认飞控未解锁；异常与回执超时按真实结果报告，不能凭预期判成功。

有限输出：本地 `F:/codex_pro/work/px4_restart_check_result.txt`。脚本本身不创建日志文件。整理文档未改系统配置、依赖、飞控参数或日常启动行为。

本次结束内核frame=452、break=34；较此前已记录frame=444增加8。未在每次进程连接边界取样，不能分配到具体阶段，也不能宣称本轮进程验证消除了底层通信错误。

## 未解锁会话的目标超时与人工停止

已执行 `scripts/offboard_session_check.py --self-check`：离线检查目标到期、人工停止、心跳中断和意外解锁四种停止锁存条件，停止后拒绝刷新目标。

已实机执行 `scripts/offboard_session_check.py --unarmed-session-check`：

- 初始doctor双向通过、未解锁LOITER，固件读取ACK=0。
- 两轮均预发送约20Hz零速度，并通过ACK和心跳确认进入未解锁Offboard。
- 第一轮停止刷新目标，0.5秒有效期到期后发送线程结束，拒绝后续目标；共发送88条零速度。
- 第二轮调用人工停止入口，发送线程结束，拒绝后续目标；共发送78条零速度。
- 两轮恢复请求ACK=0，最终心跳均确认未解锁LOITER。
- 全部接收计数包含8条BAD_DATA，本轮未分类具体原因，不能全部称为CRC错误，也不代表通信问题已修复。

此脚本只有零速度验证入口，没有解锁或非零速度功能。人工停止为程序主动调用，尚未测试真实Ctrl+C、物理断线或飞行中的退出动作。心跳中断与意外解锁条件仅离线测试，没有在实机制造这些情况。未验证定位适航性，不能作为装桨或飞行许可。

默认入口和网页不变。脚本默认运行不操作硬件；不自动记录日志。有限输出位于本地 `F:/codex_pro/work/px4_session_check_result.txt`。

收尾doctor再次双向通过、固件读取ACK=0、未解锁LOITER；fuser确认UART7无占用。内核uart-ng当前累计fe=483、brk=34，较上次记录452增长31；本轮开始未取得同一计数器快照，不能把这31次全部归于本轮脚本。

## 真实Ctrl+C与输入停止验证、当前定位退化

通过Linux控制终端PTY写入Ctrl+C字符，由终端向实际会话子进程发送SIGINT。两轮均完成finally收尾，恢复请求ACK=0、心跳确认未解锁LOITER。修正中断提示：原先误报空白“验证未通过”并退出2，现明确提示终端中断并退出130；第二轮实际确认130。默认程序未改。

读取offboard_control_mode：Ctrl+C后两次时间戳均为5613640215，读取时分别约2.01秒和7.05秒陈旧，证明该输入停止更新。随后独立LOITER零速度输入测试确认新时间戳5674695352、年龄0.027秒；停止发送后时间戳5681806417在约1.00秒和6.06秒两次读取保持不变。采样不能测出精确毫秒级停止延迟。

恢复测试最初未通过：新输入虽已到达，offboard_control_signal_lost仍True。读取当前固件同一提交的offboardCheck.cpp确认，这个标志是Offboard可用性判断，并非纯通信超时：速度控制还要求local_velocity_invalid=False。当前实测local_velocity_invalid=True、local_position_invalid=True，解释了输入新鲜但Offboard不可用。历史False/True/False只能记录标志变化，不能单独用作物理链路断开/恢复证据。

当前sensor_optical_flow与distance_sensor样本分别约984秒和986秒陈旧，飞控光流端口ttyS3的mavlink实例1 rx=0.0B/s。vehicle_local_position的xy_valid=False、v_xy_valid=False、dead_reckoning=True；不能沿用之前有效状态。飞控电池报告2.385V、cell_count=4、warning=3、remaining=0，实际电池接入与供电尚待用户确认，不能据此直接认定电池真实电压或传感器损坏。本轮不进行动力或飞行测试。

第二轮重连检查收到7条Bad prefix；独立输入时序检查3条Bad prefix，均未记录CRC错误，不代表全部过程无CRC。全轮内核计数从fe=483、brk=34开始，中途fe=492、brk=34，存在实际帧错误增长。最终保持未解锁LOITER、UART7已释放。

下一项先恢复光流/测距的新鲜输入并确认估计有效，再验证完整Offboard可用性恢复；其他异常退出路径与真实飞行保护仍未完成。有限输出：本地F:/codex_pro/work/px4_interrupt_timing_retry_result.txt、px4_input_timing_result.txt、px4_flow_current_status_result.txt。

固件依据：https://github.com/Minderring/PX4-Autopilot/blob/4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c/src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp

最终内核计数fe=498、brk=34；本轮有起点fe=483，因此全轮增加15次帧错误。已同步Notion验证流程页面。

随后用户现场确认：电池没电。停止模式切换和动力测试，等待更换正常电池或充电后恢复供电。光流/测距停更是否由供电造成，需恢复后复核，尚未判定传感器故障。

## 充电恢复后的复测

用户充满电并恢复供电后，实机只读与未解锁零速度复测通过。

- 电池报告16.87V左右、remaining约99.5%、warning=0。
- 光流端口ttyS3接收约3552B/s；光流样本年龄4ms左右、quality=105–106；测距样本年龄20–24ms、距离0.73–0.74m。
- xy/z/vxy/vz有效，local_velocity_invalid=False、local_position_invalid=False，dead_reckoning=False。
- 未解锁Offboard接管成功。真实PTY终端Ctrl+C触发SIGINT，恢复请求ACK=0，心跳确认未解锁LOITER，退出码130。
- 退出后Offboard输入时间戳79616086在约2.04秒和7.09秒采样不变；重新零速度输入时间戳89708767、年龄约21ms，offboard_control_signal_lost=False。
- 停止发送后时间戳96799571在约1.02秒和6.07秒采样不变，offboard_control_signal_lost=True。输入更新与可用性恢复均得到证据；没有测量精确毫秒级停止延迟。
- 机载板内核计数本轮前后fe=498、brk=34，无增长；重连后的诊断接收未见BAD_DATA。未完整汇总被中断子进程的全部原始帧，不能宣称历史间歇性问题已修复。

充电/恢复供电后光流与测距恢复，先前陈旧输入不再是当前阻碍。本次飞控也经历重新启动，不能仅凭这一对照断言具体供电故障根因。

**仍待验证：**heading_good_for_control=False、dist_bottom_valid=False，水平reset计数约从49到103（约55秒）；这些地面行为与之前一致，尚未证明飞行时航向、测距和定位稳定性。RC/GCS失联标志仍True，接管和保护待核对。其他异常退出、长期通信可靠性及装桨飞行仍未完成。

最终未解锁LOITER，未发送解锁或非零速度，不启动电机。下一项可先验证主动异常抛出后的收尾与恢复，再核对接管/保护配置。有限输出在本地F:/codex_pro/work：px4_after_charge_status_result.txt、px4_after_charge_offboard_result.txt。

## 受控RuntimeError异常退出验证

已执行新增的显式未解锁测试入口：`scripts/offboard_session_check.py --unarmed-exception-check`。默认入口无变化。

- 双向连接、初始未解锁LOITER；零速度预发送后进入未解锁Offboard。
- ACTIVE阶段维持零速度约1秒后，主线程主动抛出指定RuntimeError。
- finally执行停止发送与模式恢复，恢复ACK=0、心跳确认未解锁LOITER；进程退出码2是此次故意注入异常的预期结果，测试外层核对异常标记和恢复证据后判定通过。
- 新连接重新打开UART7成功。offboard_control_mode时间戳304662607在约1.98秒和4.99秒采样保持不变；没有自动重放旧目标。offboard_control_signal_lost=True，本地位置/速度无效标志均False。
- 电池约16.79V、warning=0。收尾未解锁LOITER，UART7已释放。
- 本轮内核fe=498→504，增加6次，brk=34不变；重连诊断5条Bad prefix。不能把所有BAD_DATA归为CRC，也不能认定电气/驱动故障根因。历史通信问题仍未解决。

本项只证明ACTIVE主线程受控RuntimeError的finally路径，不覆盖全部异常类型、发送线程故障、连续中断、物理断线、系统掉电或飞行中的保护响应。下阶段优先核对遥控接管/失联保护参数与可用条件，只读读取；不会自动修改保护参数或启动动力。

有限输出：本地F:/codex_pro/work/px4_exception_result.txt。

## 保护配置只读检查与范围调整

用户当前决定暂缓断联/接管问题，后续再处理。本轮仅完成读取，无参数修改、模式切换或解锁；暂缓不代表这些保护已通过验证。

实测配置：
- COM_RC_IN_MODE=3：RC或摇杆选择首个输入并保持；并非禁用RC。当前input_rc从未发布、manual_control_setpoint.valid=False。
- COM_RC_OVERRIDE=1，仅启用自动模式的摇杆覆盖位；Offboard覆盖位未启用。
- COM_OF_LOSS_T=1秒、COM_OBL_RC_ACT=0（参数指定Position回退）。不把它解释为已验证的自动降落；实际结果还受其他保护与模式可用条件影响。
- NAV_RCL_ACT=3、COM_RC_LOSS_T=0.5秒、COM_RCL_EXCEPT=0；NAV_DLL_ACT=0、COM_DL_LOSS_T=10秒。
- COM_POSCTL_NAVL=0，COM_FAIL_ACT_T=5秒，COM_LOW_BAT_ACT=2。只记录参数含义，不将超时简单相加为已验证的响应时间。
- 当前RC与GCS失联标志True、pre_flight_checks_pass=False。收尾未解锁LOITER。
- 主体只读检查未见BAD_DATA，内核fe=504、brk=34保持不变。后续短补读没有另取内核快照，不扩大此结论范围。

参考：[同版本参数定义](https://github.com/Minderring/PX4-Autopilot/blob/4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c/src/modules/commander/commander_params.c)、[同版本保护逻辑](https://github.com/Minderring/PX4-Autopilot/blob/4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c/src/modules/commander/failsafe/failsafe.cpp)。

下一项回到通信可靠性和其他异常路径，继续未解锁、零速度范围；接管与断联策略暂缓。有限输出：本地F:/codex_pro/work/px4_protection_readonly_result.txt、px4_no_rc_followup_result.txt。


## Web v0.4运动任务接入与未解锁验证

已部署到 `/home/pi/drone_project`，运行服务PID42586，网址 http://192.168.137.3:8081/ 。新版提供真实平移、升降、相对转向、起飞和PX4 LAND；任务完成根据位置/偏航误差、速度和稳定样本判定，取消/超时保持当时有效局部位置。解锁后未开始起飞保留10秒台架收尾；明确起飞任务被接受后清除此计时。普通模式/解锁检查不绕过。

- 板端32项离线检查通过：12项基本控制与HTTP边界、15项任务计算、5项任务集成及pymavlink实际编码。frame=LOCAL_NED(1)、mask=1479，启用速度与yaw_rate；未连接UART执行这些测试。
- 修正ESTIMATOR_STATUS速度有效掩码为6，任务要求位置/速度掩码46。增加ODOMETRY reset_counter及时间回退检查，估计器重置不继续旧坐标任务。
- 实机HTTP：未解锁LOITER→OFFBOARD，ACK=0、新心跳确认；20Hz零速度流新鲜。未解锁的起飞/平移/转向/升降请求全部409、任务仍idle，没有发送ARM或非零速度。
- 已落地上锁时降落请求仅返回无需命令；关闭实机控制后恢复未解锁LOITER、租期与设定值流关闭，双向通信保持。结束采样15.763V、estimator_flags=302、ODOMETRY年龄0.15秒。
- 当前ODOMETRY重置计数仍持续改变；新任务会据此中止。历史航向、测距及UART间歇性帧错误未解决，不能称已具备稳定飞行条件。
- JS语法与DOM逻辑通过：四类请求、降落/取消、实际位置映射、真实进度与陈旧反馈清空、控制关闭后禁用。当前浏览器绑定超时，实际点击与视觉渲染未验证。
- 本轮未解锁、未启动电机、未实际起飞或降落。当前版本悬停、运动跟踪和降落完成验证仍待现场测试。比赛编排、视觉闭环、PID写入未接通。

使用说明见 `docs/web_prototype.md`。服务后台运行、无新增运行日志和开机自启；原test.py与cv_work接口保持。临时回退副本位于板端 `/tmp/drone_web_previous`（重启可能清除）。


## 2026-10-10：Web v0.5 双摇杆

用户选择沿用Offboard速度控制。已增加左杆升降/偏航、右杆前后/左右速度；前端约10Hz输入，开发板独立20Hz死区、限幅、实时机头坐标转换及唯一来源发送，仍复用UART7。速度上限水平/垂直0.2m/s、偏航30度/s；松杆对应轴归零，旧会话/乱序/超时不重放。

摇杆可接管并取消普通运动目标；起飞/降落期间不可接管；摇杆期间新自动任务拒绝，LAND优先撤销摇杆。退出后用有效定位保持退出位置，迟到退出不会破坏新自动任务。页面失焦/切页回中，板端输入0.5秒停更锁止。未修改PX4安全参数。

- 板端47项离线检查通过（原32+15项摇杆/协调/HTTP）。DOM逻辑检查通过双指独立捕获与松杆、首帧中立、序号、键盘回中、失焦退出、自动任务互斥和LAND可用。
- v0.5已部署，后台服务本次PID5166，未设置自启或新增日志；旧文件临时回退副本 `/tmp/drone_web_v04_backup`，重启可能清除。
- 真实HTTP未解锁LOITER→OFFBOARD→LOITER，ACK=0、新心跳确认。未解锁stick_start及输入接口均409，任务仍idle；原起飞/平移/转向/升降未解锁拒绝仍通过。结束控制租期与设定值流关闭。
- 结束采样电池16.02V、estimator_flags=302、ODOMETRY年龄0.10秒；这些是采样值，不表示当前固定状态。
- 网页v0.5、两摇杆HTML及joystick.js请求通过。浏览器自动绑定/新页读取超时，实际视觉渲染和双指操作未现场验证。
- 本轮未解锁、未启动电机、未发送非零速度。实际空中摇杆与飞行跟踪仍待验证；估计器重置、相对航向、测距与历史UART坏帧问题没有宣称修复。

结构及操作说明：`docs/joystick_control.md`；启动方式仍 `python3 web_control.py`，刷新 `http://192.168.137.3:8081/#motion` 加载新版。

## 2026-10-10：Web v0.6 简化地面摇杆调试

用户要求取消影响调试的额外限制。摇杆不再要求已解锁/确认空中，不因位置原点或ODOMETRY重置停用；仅以新鲜偏航完成机头到NED转换。可以接管并取消起飞任务，LAND期间仍由PX4完成降落。网页额外10秒自动上锁已取消，解锁保持手动，发送普通ARM前清除地面调试杆量。PX4参数及正常检查未改，自动位置任务反馈检查保持原状。

- 48项板端离线检查通过：12项基本控制、16项摇杆/来源协调/HTTP、20项运动任务与编码。JS语法与DOM交互通过，包括地面未解锁启用、起飞任务切换及LAND协调。
- 更新前后均确认未解锁；已部署v0.6，后台服务本次PID7865，无新增依赖、运行日志或自启；临时回退副本 `/tmp/drone_web_v05_backup`。
- 真实HTTP未解锁LOITER→OFFBOARD→LOITER，ACK=0；25帧中立输入全部接受，采到4个不同ODOMETRY重置计数仍保持摇杆active、来源joystick，目标始终[0,0,0,0]。旧会话输入409；未解锁自动运动仍409且任务idle。
- 最终控制关闭、设定值流停止、未解锁LOITER。采样电池15.856V、估计器标志302、ODOMETRY年龄0.09秒。本次服务BAD_DATA累计6条，未宣称UART问题修复。
- 本轮无ARM、无非零速度、无电机转动；实际空中操纵与运动跟踪尚未验证。浏览器现场鼠标/双指操作仍需现场检查，DOM结果不替代此项。
