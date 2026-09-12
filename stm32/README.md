# AVG_V1：Orange Pi UART V1 对接固件

2026-09-08 新增车头超声波停车：20 cm 触发、30 cm 恢复，上电及遇障后需要显式解除。烧录和验收见 [超声波停车说明](ULTRASONIC_STOP.md)，后续扩展见 [摄像头跟随与陀螺仪辅助循迹方案](CAMERA_FOLLOW_PLAN.md)。

2026-09-08 横移启动同步修复：四路 TIM1 PWM 在同一个更新事件提交；从静止或换向时，四轮先共同归零并等待一个 PWM 死区周期，再同时加载新占空比。编码器尚未检测到转动时加入独立的 M1～M4 静摩擦补偿，转速达到 5 RPM 后恢复普通 PID。标定参数位于 `Core/Inc/car_config.h` 的 `CAR_MOTOR_Mx_START_COMMAND`。

本工程运行于 STM32F407VET6，Orange Pi 使用 `USART2` 与固件通信。网页、FastAPI
网关或以后替换的 ROS 串口桥都必须发送本文定义的同一套二进制帧；STM32 不解析 JSON，
也不依赖 ROS。原有 `USART3` 仅保留为本地调试命令和可读文本日志，不得接入网页网关。

> 首次联调必须架空四个车轮。当前轮径、轮距、编码器参数和 PID 是可编译的起始值，
> 不是实车标定结果。

## 编译与测试

使用 Keil 打开 `MDK-ARM/AVG_V1.uvprojx`，选择 `AVG_V1` target，执行 Rebuild。工程使用
Arm Compiler 6。协议模块也可在 Windows 上独立回归：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -I Core/Inc `
  Core/Src/avg_protocol.c Tests/test_avg_protocol.c `
  -o Tests/test_avg_protocol.exe
./Tests/test_avg_protocol.exe

gcc -std=c11 -Wall -Wextra -Werror -I Core/Inc `
  Tests/test_motor_start_compensation.c `
  -o Tests/test_motor_start_compensation.exe
./Tests/test_motor_start_compensation.exe
```

香橙派安装 `pyserial` 后，可用 `Tools/opi_protocol_test.py` 产生与网关完全相同的帧并保存
串口验收日志：

```bash
python3 -m pip install pyserial
python3 Tools/opi_protocol_test.py --port /dev/ttyS5 monitor
python3 Tools/opi_protocol_test.py --port /dev/ttyS5 --wheels-up smoke
python3 Tools/opi_protocol_test.py --port /dev/ttyS5 --wheels-up watchdog
python3 Tools/opi_protocol_test.py --port /dev/ttyS5 crc-error
```

`--wheels-up` 是运动测试的强制确认，不加时脚本拒绝发送非零 DRIVE。实际设备名以 Orange
Pi 的设备树和 `ls -l /dev/serial/by-id/` 为准。

## 串口与硬件资源

Orange Pi 串口参数为 `115200, 8 data bits, no parity, 1 stop bit, no flow control`，电平为
3.3 V TTL。Orange Pi TX 接 PA3，Orange Pi RX 接 PA2，双方 GND 必须共地。

| 功能 | STM32 引脚 | 外设 |
| --- | --- | --- |
| Orange Pi TX → STM32 RX | PA3 | USART2_RX，DMA1 Stream5 Channel4，循环 DMA + IDLE |
| Orange Pi RX ← STM32 TX | PA2 | USART2_TX，中断式非阻塞发送 |
| 调试 TX/RX | PB10/PB11 | USART3，115200 8N1 |
| 四路 PWM | PE9/PE11/PE13/PE14 | TIM1 CH1～CH4，20 kHz |
| M1/M2 方向 | PC0/PC1、PC2/PC3 | GPIO |
| M3/M4 方向 | PE0/PE1、PE2/PE3 | GPIO |
| TB6612 STBY | PE4 | GPIO 开漏；低电平关闭驱动 |
| 四路编码器 | PA0/PA1、PA6/PA7、PB6/PB7、PC6/PC7 | TIM2/3/4/8 编码器模式 |
| 电池电压 | PA4 | ADC1 IN4 |
| MPU6050 | PB8/PB9、PB5 | I2C1 + RX DMA、数据就绪 EXTI |
| HC-SR04 ECHO/TRIG | PE5/PE6 | TIM9_CH1 输入捕获（1 MHz）/GPIO 输出，60 ms 周期 |

工程目前没有分配物理急停输入。因此 `Core/Inc/car_config.h` 中
`CAR_PHYSICAL_ESTOP_ENABLED` 默认为 0；产品上车前应接入常闭急停回路、在 CubeMX 配置输入，
并把 `CAR_PHYSICAL_ESTOP_RELEASED()` 映射到该输入。软件 ESTOP 已完整实现，但不能替代
断电型硬件急停。

HC-SR04 使用 5 V 供电并与 STM32 共地，TRIG 接 PE6。模块 ECHO 通常输出约 5 V，接入
PE5 前必须使用分压或电平转换降到 3.3 V，不能把 5 V ECHO 直接接到 MCU。TIM9 以
1 MHz 计数，输入捕获中断测量高电平宽度；测距任务最多等待 40 ms，不会阻塞电机控制。
USART3 文本遥测输出 `Distance: xx.x cm`，无回波时输出 `Distance: timeout`。

## UART V1 帧格式

所有整数与 IEEE-754 `float32` 都是小端。最大 payload 为 256 字节。

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 2 | 同步字 `A5 5A` |
| 2 | 1 | version，固定 `01` |
| 3 | 1 | message_type |
| 4 | 1 | flags；控制帧必须为 `00`，STATUS bit 0 为 BLE 配对请求 |
| 5 | 2 | payload_len，uint16 |
| 7 | 4 | transport_sequence，uint32 |
| 11 | N | payload |
| 11+N | 2 | CRC16，小端 |

CRC 为 CRC-16/CCITT-FALSE：`poly=0x1021`、`init=0xFFFF`、`refin=false`、
`refout=false`、`xorout=0`。校验范围从 version（偏移 2）到 payload 末尾，不包含同步字和
CRC。ASCII `123456789` 的结果必须为 `0x29B1`。

接收任务支持半帧、跨 DMA 回绕、粘包和任意噪声。长度、版本或 CRC 错误时不会刷新通信
看门狗，解析器会继续查找下一个 `A5 5A`。

## 消息定义

### DRIVE `0x01`，Orange Pi → STM32

payload 固定 8 字节：

```c
typedef struct __attribute__((packed)) {
    float linear_mps;   /* 前进为正，m/s */
    float angular_rps;  /* 左转为正，rad/s */
} AvgDrivePayload_t;
```

NaN、Inf、错误长度和非零 flags 被拒绝。有限数值会限幅到 `linear [-1, 1] m/s`、
`angular [-3, 3] rad/s`。差速解算为：

```text
left_mps  = linear_mps - wheel_track / 2 * angular_rps
right_mps = linear_mps + wheel_track / 2 * angular_rps
wheel_rpm = wheel_mps * 60 / (2π * wheel_radius)
```

左前/左后使用 left，右前/右后使用 right。超过最大轮速时四路按同比例缩放，保持曲率；
目标 RPM 再进入四路独立编码器 PID，绝不把 UART 数值直接写到 PWM。

### DRIVE_HOLONOMIC `0x04`，Orange Pi → STM32

payload 固定 12 字节，依次为 `float linear_mps`、`float lateral_mps`、
`float angular_rps`。正横移速度表示向右平移。该帧使用麦克纳姆四轮混控，最大轮速时
四路同比例缩放，并沿用 DRIVE 的 300 ms 通信看门狗、ESTOP 锁存、硬件故障拦截和
逐帧 ACK。原 `DRIVE 0x01` 的格式和行为保持不变。

### ESTOP `0x02`，Orange Pi → STM32

payload 必须为单字节 `01`。收到合法帧即锁存急停、拉低 STBY、清目标和待处理运动命令；
电机任务在最多一个 10 ms 控制周期内清四路 PWM 和 PID。普通 DRIVE 只能得到
`ESTOP_LATCHED`，不能解除急停。

### CLEAR_ESTOP `0x03`，Orange Pi → STM32

payload 必须为单字节 `A5`。只有物理急停已释放、没有编码器方向等硬件故障、通信没有
超时且输出保持为 0 时才允许清除。清除后仍保持 STBY 关闭，必须再收到一帧合法 DRIVE
才能运动。

### ACK `0x81`，STM32 → Orange Pi

payload 固定 2 字节：`acked_type`、`result`。ACK 帧的 transport_sequence 原样回显被确认
命令的序号。

| result | 名称 | 含义 |
| ---: | --- | --- |
| 0 | OK | 已接受 |
| 1 | INVALID_PAYLOAD | 长度、flags、magic 或 float 非法 |
| 2 | ESTOP_LATCHED | 急停仍锁存，DRIVE 未执行 |
| 3 | HARDWARE_FAULT | 物理/编码器故障，或清急停安全条件不满足 |
| 4 | UNSUPPORTED | 未支持的 message_type |

CRC 错误、错误 version 或不完整帧没有 ACK。

### STATUS `0x80`，STM32 → Orange Pi

STATUS payload 仍固定为 34 字节。帧头 `flags bit 0` 为
`AVG_STATUS_FLAG_BLE_PAIR_REQUEST`：PB0 外接常开按键长按 3 秒且车辆输出持续为零时，
该位维持约 1 秒，由香橙派网关转换成本机配对事件。按键松开后才能再次触发。
其他 STATUS flags 保留并必须为 0。配对请求不改变目标速度、STBY、ESTOP 或故障锁存。

按键接线：PB0 使用 MCU 内部上拉，常开按键的一端接 PB0，另一端接 GND。不要复用
开发板常见的 PA0、PE2、PE3 或 PE4 按键脚；这些脚在本工程已分配给编码器、M4 方向和
电机 STBY。香橙派收到请求后仅开放 180 秒蓝牙可发现/可配对窗口，之后自动关闭。

以 10 Hz 发送，payload 严格为 34 字节：

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | uint16 | battery_mv |
| 2 | uint16 | ultrasonic_mm；HC-SR04 毫米值，超时/离线/数据过期为 `0xFFFF` |
| 4/8/12/16 | int32 × 4 | 前左、前右、后左、后右累计编码器计数 |
| 20 | float32 | measured_linear_mps |
| 24 | float32 | measured_angular_rps |
| 28 | float32 | imu_yaw_rad |
| 32 | uint8 | estop，0/1 |
| 33 | uint8 | fault_code |

MPU6050 没有磁力计，yaw 是陀螺积分值，会随时间漂移。

## 故障码

| fault_code | 定义 |
| ---: | --- |
| `0x00` | 无故障 |
| `0x01` | 超过 300 ms 未收到合法 USART2 DRIVE 或 USART3 调试运动心跳，通信看门狗停车 |
| `0x02` | 物理急停输入有效 |
| `0x11`～`0x14` | 前左、前右、后左、后右编码器反馈方向故障 |
| `0x20` | USART2 RX/TX 错误或 RX 流缓冲溢出；下一帧合法 DRIVE 清除显示 |

软件 ESTOP 由 STATUS 的 `estop` 字段表示，本身不伪装成硬件故障。通信超时可由下一帧
合法 DRIVE 自动恢复；ESTOP 必须显式 CLEAR。编码器方向故障需先排除接线/配置问题，再从
USART3 调试口发送 `K` 清除。

## FreeRTOS 任务与并发边界

| 任务 | 优先级 | 周期/触发 | 职责 |
| --- | --- | --- | --- |
| safetyTask | High | 100 Hz | 物理急停与独立 300 ms 通信看门狗 |
| uartRxTask | AboveNormal | DMA/IDLE 数据到达 | 增量解帧、CRC、命令状态和 ACK |
| motorControlTask | AboveNormal | 100 Hz，绝对节拍 | 编码器测速、差速目标、四路 PID、PWM/方向/STBY |
| protocolTxTask | Normal | TX 队列 | 独占 USART2 中断式非阻塞发送 |
| uartCommandTask | Normal | 轮询 USART3 | 本地调试命令和文本遥测 |
| imuTask | BelowNormal | MPU6050 DRDY/超时 | 静止校准、I2C DMA、姿态快照 |
| ultrasonicTask | BelowNormal | 60 ms；TIM9 捕获/40 ms 超时 | 触发 HC-SR04 并发布测距快照 |
| telemetryTask | Low | 10 Hz，绝对节拍 | 采集快照并打包 STATUS，不直接控制电机 |

DMA/USART ISR 只把新字节写入 FreeRTOS stream buffer 或设置完成标志，不做解帧。共享状态用
临界区保护；正常 PWM 和方向控制只在 motorControlTask 中完成。急停路径唯一的异步硬件
动作是把 STBY 拉低，确保即使电机任务尚未到下一个周期，H 桥也已经禁止输出。

## 集中标定参数

所有车辆参数位于 `Core/Inc/car_config.h`：

- `CAR_WHEEL_RADIUS_M`：默认 0.0325 m，必须按轮胎负载半径实测；
- `CAR_WHEEL_TRACK_M`：默认 0.180 m，必须按左右轮接地点中心距实测；
- `CAR_ENCODER_MOTOR_PPR`、`CAR_ENCODER_QUADRATURE_FACTOR`、
  `CAR_MOTOR_GEAR_RATIO`：默认得到每轮 1560 count/rev；
- `CAR_MAX_WHEEL_RPM`、`CAR_PWM_MAX_COMMAND`；
- `CAR_PID_KP/KI/KD/KFF` 与积分限幅；
- 四个电机和四个编码器的正方向符号；
- 控制、遥测和通信看门狗周期。

## 架空验收顺序

1. 上电不发命令，确认 PWM=0、STBY 关闭；约 300 ms 后 STATUS 为 fault `0x01`。
2. 发 DRIVE `v=0.25, w=0`，确认 ACK OK，四轮都向前低速旋转；STATUS 四路计数和线速度为正。
3. 发 DRIVE `v=0, w=1`，确认左轮向后、右轮向前，小车语义为原地左转。
4. 发一帧坏 CRC，确认没有 ACK、目标不改变、通信看门狗时间不刷新。
5. 停止发送 DRIVE 超过 300 ms，确认 STBY 关闭、PWM/目标为 0、fault 为 `0x01`。
6. 发 ESTOP，随后发普通 DRIVE，确认 ACK 为 `ESTOP_LATCHED` 且不运动。
7. 在物理急停释放且无硬件故障时，先发一帧 DRIVE 证明链路正常，再发 CLEAR_ESTOP；确认仍
   静止，直到下一帧 DRIVE 才恢复。
8. 在 PE5 接入经降压的 ECHO 后，用不同距离的平面目标检查 USART3 的厘米文本和 STATUS
   的 `ultrasonic_mm`；断开 ECHO 时确认文本为 timeout、STATUS 为 `0xFFFF`。
9. 连续观察 STATUS 频率约 10 Hz，并核对电压、编码器、IMU yaw、estop 和 fault。

实车落地前还必须完成每轮方向、每轮一圈计数、轮径、轮距与 PID 标定。编译和协议单元测试
不能替代上述硬件验收。
