# AVG_V1 基础运动控制说明

> 2026-08-30 更新：Orange Pi 正式控制链已接入 USART2 二进制 UART V1 协议、300 ms
> 通信看门狗、锁存急停、ACK 和 10 Hz STATUS。本文保留的 USART3 单字节命令仅用于架空
> 调试；正式协议、故障码和联调步骤以根目录 `README.md` 为准。

本工程面向 STM32F407VETx（LQFP100）、四路 TIM1 PWM 和双 TB6612
（或兼容双 H 桥）。当前固件采用四轮独立的编码器闭环速度控制：每 10 ms 读取四路
编码器、换算轮端 RPM、执行四路 PID，并把 PID 输出写入四路 PWM。上电后 PWM 保持为
0、STBY 保持关闭，不会自动启动车轮。首次烧录和参数标定必须先把车轮架空。

## FreeRTOS 任务结构

- `uartCommandTask`：Normal 优先级、2048 字节栈，只负责 USART3 命令解析、ACK/NACK
  和遥测发送，不直接操作电机硬件。
- `motorControlTask`：AboveNormal 优先级、2048 字节栈，每 10 ms 读取编码器、计算 RPM、
  运行四路 PID，并独占调用 PWM、方向引脚和 STBY 驱动。
- `imuTask`：BelowNormal 优先级、2048 字节栈，初始化并静止校准 MPU6050，随后由 PB5
  数据就绪中断触发 I2C1 DMA，以 100 Hz 更新加速度、角速度和姿态快照。IMU 断线时只重试
  传感器，不会阻塞或关闭电机任务。
- `ultrasonicTask`：BelowNormal 优先级、1024 字节栈，每 60 ms 触发 HC-SR04，通过
  PE5/TIM9_CH1 输入捕获测量回波；40 ms 无回波即超时，不阻塞电机任务。
- `motorCommandQueue`：长度 16，元素为动作模式和四轮目标 RPM。串口任务通过 CMSIS-RTOS2
  消息队列把前进、后退、左右转、停车和清故障请求交给电机任务。
- `uartRxTask`：AboveNormal 优先级，消费 USART2 RX 循环 DMA + IDLE 写入的 stream
  buffer，完成增量解帧、CRC 和 ACK。
- `safetyTask`：High 优先级，100 Hz 独立检查物理急停和 300 ms DRIVE 看门狗。
- `protocolTxTask`：Normal 优先级，独占 USART2 中断式非阻塞发送。
- `telemetryTask`：Low 优先级，10 Hz 打包并排队发送固定 34 字节 STATUS。

队列让电机状态只有一个任务负责写入，避免串口收发与 10 ms 控制周期并发修改目标。
若瞬间发送过多命令导致队列已满，串口返回 `NACK:<命令> MOTOR_QUEUE_BUSY`；该命令
不会执行，发送端应稍后重发。硬件初始化仍在调度器启动前完成，并保持 PWM=0、STBY=OFF。

## STM32F407VET6 引脚配置

| 模块信号 | STM32F407VET6 | 外设/模式 |
| --- | --- | --- |
| `PWMA/PWMB/PWMC/PWMD` | `PE9/PE11/PE13/PE14` | `TIM1_CH1~CH4`, AF1, 20 kHz |
| `AIN1/AIN2` | `PC0/PC1` | 推挽输出 |
| `BIN1/BIN2` | `PC2/PC3` | 推挽输出 |
| `CIN1/CIN2` | `PE0/PE1` | 推挽输出 |
| `DIN1/DIN2` | `PE2/PE3` | 推挽输出 |
| `STBY` | `PE4` | 开漏输出，无内部上下拉 |
| 编码器1～4 | `PA0/PA1`, `PA6/PA7`, `PB6/PB7`, `PC6/PC7` | `TIM2/3/4/8`编码器模式 |
| `ADC` | `PA4` | `ADC1_IN4` |
| 调试串口 | `PB10/PB11` | `USART3_TX/RX`, 115200 8N1 |
| Orange Pi 串口 | `PA2/PA3` | `USART2_TX/RX`, 115200 8N1；RX DMA + IDLE |
| MPU6050 `SCL/SDA` | `PB8/PB9` | `I2C1`, 100 kHz；模块地址 `0x68`（AD0 接地） |
| MPU6050 `INT` | `PB5` | 上升沿 EXTI5，数据就绪中断 |
| HC-SR04 `ECHO/TRIG` | `PE5/PE6` | `TIM9_CH1` 输入捕获（1 MHz）/推挽输出 |

MPU6050 模块使用 3.3 V 逻辑电平并与主板共地。若模块本身没有 I2C 上拉电阻，需要在
SCL、SDA 各加约 4.7 kOhm 上拉到 3.3 V。上电后的约 2 秒校准期间必须保持车辆静止，
否则陀螺仪零偏会被错误学习。

HC-SR04 使用 5 V 供电并与主板共地；TRIG 可直接接 PE6，ECHO 必须先通过分压或电平转换
降到 3.3 V 再接 PE5。USART3 遥测每 250 ms 输出一次 `Distance: xx.x cm`，无有效回波
时输出 `Distance: timeout`；Orange Pi STATUS 同时携带毫米值。

## 串口控制

调试串口为 USART3（PB10 TX、PB11 RX），参数 `115200 8N1`。发送单字节命令：

| 命令 | 动作 |
| --- | --- |
| `F` | 前进 |
| `B` | 后退 |
| `L` | 原地左转 |
| `R` | 原地右转 |
| `S` 或 `X` | 短刹车 |
| `C` | 滑行停止 |
| `!` 或 `0` | 电机驱动进入 standby |
| `1`～`9` | 设置目标转速 20～180 RPM（每档 20 RPM） |
| `Q` / `E` | 麦克纳姆轮左移 / 右移；默认关闭，见下文 |
| `K` | 清除编码器方向故障；修正符号后使用 |
| `?` | 返回完整串口命令帮助 |

默认目标为 80 RPM。`F` 会给四个车轮相同的 `+80 RPM` 目标，四路 PID 会分别补偿
电机和负载差异；`B` 使用相同的负目标。每个 USART3 运动命令都作为一次 300 ms 看门狗
心跳，因此架空调试时必须以至少 5 Hz 重复发送 `F/B/L/R/Q/E`。停止发送后 PWM 和 STBY
会自动关闭。正式运行仍应由 Orange Pi 周期发送 UART V1 DRIVE。

每个有效命令都会通过 USART3 TX 返回确认，例如：

```text
ACK:F STATE:FORWARD TARGET_RPM:80 FAULT:0
ACK:L STATE:TURN_LEFT TARGET_RPM:80 FAULT:0
ACK:S STATE:BRAKE TARGET_RPM:80 FAULT:0
```

回车、换行、空格和制表符会被忽略，因此串口助手启用“发送新行”也可以正常使用。
其他未知字节会返回 `NACK:0xXX SEND:? FOR HELP`，不会改变当前运动状态。

串口每 250 ms 输出一次：

```text
ENC_D:每10ms增量 ENC_T:累计计数 RPM:四轮实际转速 TARGET:四轮目标转速 PWM:四路PID输出 ESIGN:运行时编码器符号 ECAL:符号锁定状态 FAULT:故障轮号 VIN:电压
IMU:状态 WHO:器件ID SAMPLE:样本数 AGE:样本年龄 ERR:错误数 ACC_mg:X,Y,Z GYRO_mdps:X,Y,Z ANGLE_cdeg:Roll,Pitch,Yaw
```

`IMU:CAL` 表示正在静止校准，`IMU:READY` 表示实时数据有效，`IMU:ERROR` 表示传感器
未响应或 DMA 连续失败；固件会每秒自动重试。数值使用定点单位避免在嵌入式串口格式化
中启用浮点 `printf`：`1000 mg = 1 g`、`1000 mdps = 1 dps`、`100 cdeg = 1°`。
MPU6050 没有磁力计，因此 Roll/Pitch 由互补滤波校正，Yaw 为角速度积分并会随时间漂移。

首次持续运动时，每一路会依据目标方向和原始 A/B 计数方向独立确认运行时编码器符号。
`ECAL:1,1,1,1` 表示四路均已确认，`ESIGN` 显示实际采用的 `+1/-1`。只有完成确认后
才启用方向故障保护，避免未经实车标定的默认符号导致四轮启动约半秒后误停。
`FAULT:0` 表示正常；`FAULT:1`～`4` 表示已确认极性后，对应车轮的反馈方向持续与目标
相反。发生真正的方向故障时固件仍会立即把 PWM 清零并拉低 STBY。

## 编码器与 PID 参数

编码器模式为 `TIM_ENCODERMODE_TI12`，A/B 两相四倍频计数。RPM 换算公式为：

```text
RPM = 编码器增量 × 60000 / (每轮一圈计数 × 实际采样毫秒数)
```

配置集中在 `Core/Inc/ax_encoder.h`：

- `AX_ENCODER_COUNTS_PER_WHEEL_REV`：轮子转一整圈时定时器的计数增量。当前默认 `1560`
  仅对应 `13 PPR × 4倍频 × 30:1减速比`；编码器或减速比不同时必须改为实测值。
- `AX_ENCODER_M1_FORWARD_SIGN`～`M4`：上电后的初始默认值。固件会在首次持续运动时按
  每一路原始计数方向自动确认运行时符号；遥测中的 `ESIGN` 可用于把实测结果回填为以后
  的默认值。

PID 初始参数集中在 `Core/Src/freertos.c` 的 `CAR_PID_KP/KI/KD/KFF`。当前值是安全的
起调值，不是所有电机的最终标定值。四轮架空确认方向与 RPM 正确后，再按实际电机逐步
调整；PID 输出限制为 `±AX_MOTOR_MAX_COMMAND`，且超速校正只会把 PWM 降到零，不会自行
反转 H 桥。

## 轮位与方向标定

运动层默认轮位如下：

```text
        车头
  M1             M2
  前左           前右

  M3             M4
  后左           后右
```

实车架空最终标定结果为：M1左前=`+1`、M2右前=`+1`、M3左后=`+1`、M4右后=`+1`。
四个已安装电机均采用正极性。若以后更换电机或对调电机线，只修改对应的
`AX_MOTOR_Mx_FORWARD_SIGN`，不要同时修改运动学公式。

首次上车前应把车轮架空，并按以下顺序检查：

1. 车轮架空后上电，确认四轮始终不转；串口应显示 `STATE:DISABLED`、`FAULT:0`。
2. 手动把每个轮子朝车辆前进方向转一圈，记录 `ENC_D/RPM` 的正负号，并核对一圈计数。
   修改每轮编码器符号和 `AX_ENCODER_COUNTS_PER_WHEEL_REV` 后重新编译。
3. 建议先发送 `4` 和 `F`，以 80 RPM 架空运行，等待遥测显示 `ECAL:1,1,1,1`；确认
   四轮都朝车辆前进方向、四路 RPM 均为正且 `FAULT:0`。若某路 `ECAL` 长期为 0，检查
   对应编码器供电、A/B 接线和定时器引脚。
4. 分别验证 `B`、`L`、`R`，每一步之间发送 `S`；目标和实际 RPM 的正负号应一致。
5. 保持 `4` 和 `F`，确认四轮目标都是 `80 RPM`，四路实际 RPM 稳定接近目标。
6. 落地低速测试直行，根据遥测调整 PID；轮径差、打滑或底盘装配误差仍需机械校正。
7. 仅在确认安装的是麦克纳姆轮且轮位正确后，把 `Core/Inc/ax_motor.h` 中
   `AX_MOTOR_ENABLE_STRAFE` 改为 `1`，重新编译后再测试 `Q`、`E`。普通轮底盘保持为 `0`。

## 主要接口

- `AX_MOTOR_SetWheelSpeeds()`：按前左、前右、后左、后右设置逻辑轮速。
- `AX_MOTOR_SetDifferential()`：设置左右两侧轮速。
- `AX_MOTOR_SetMotion()`：麦克纳姆轮前进/横移/旋转混控，并按比例归一化限幅。
- `AX_MOTOR_Forward/Backward/TurnLeft/TurnRight()`：基本动作。
- `AX_MOTOR_BrakeAll()`、`AX_MOTOR_CoastAll()`、`AX_MOTOR_Disable()`：三种停车方式。
- `CAR_MotorControlTask()`：FreeRTOS 电机任务，消费命令队列并执行 10 ms 四轮 RPM
  计算、方向诊断和独立 PID 闭环。

非零指令直接反向时，驱动会先把 PWM 强制为零，等待一个 20 kHz PWM 周期，再改变
方向引脚。HardFault、总线错误、FreeRTOS 断言、栈溢出和堆分配失败时也会关闭 TIM1
主输出并拉低 STBY。

## 开源实现参考

实现方式参考了以下 GitHub 项目的公开设计模式，但本工程代码按当前硬件重新实现：

- [SparkFun TB6612FNG Arduino Library](https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library/blob/master/src/SparkFun_TB6612.cpp)：带符号速度、前进/后退、差速转向和 standby 的接口组织。
- [Wemos Motor Shield Firmware](https://github.com/wemos/Motor_Shield_Firmware/blob/f8a3f28c2c8c2b7e2cb0f8f8a285caed34a76126/Src/tb6612.c)：STM32 上对短刹车、滑行停止和 standby 的区分。
- [WayV5 四电机 STM32 驱动](https://github.com/WayV5/RK3588-STM32-ROS2-SLAM-Robot/blob/aeb6303041bc14dd3b8c710d96979c0353058c47/stm32/prj/Core/Src/motor.c)：四轮角色表、限幅以及反向前先过零的设计。
- [XTARK 同类 STM32F4 四驱工程归档](https://github.com/heiyebaitian/esp32_smart_car/tree/5c692b93adb2d1da6ec64a07e2914e5f460c9cab)：同类 TIM1 20 kHz 四电机硬件和基本差速动作语义。
- [Toshiba TB6612FNG 数据手册](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG)：最终停车模式和 STBY 行为以器件真值表为准。

## 构建与硬件注意事项

使用 `MDK-ARM/AVG_V1.uvprojx` 的 `AVG_V1` target 进行 Rebuild，工程已配置 Arm
Compiler 6。构建成功只能证明软件控制链完整；每轮一圈计数、方向、轮位和 PID 参数仍须
按上述流程实车标定。没有实物标定，任何软件都不能保证真实 RPM 精度和落地直行效果。

当前系统时钟按 8 MHz HSE 配置。若板上晶振不是 8 MHz，应先修正 CubeMX 时钟树，
否则 UART、PWM 和系统节拍都会不正确。PE4/STBY 配置为开漏输出：写低进入 standby，
写高时释放引脚，由驱动模块原理图中的 10 kOhm 电阻上拉到 5 V。软件急停不能覆盖 MCU
掉电、调试器冻结或引脚断线等所有故障。

驱动模块原理图把 TB6612 的逻辑 `VCC` 接到 5 V，而器件数据手册规定控制输入高门限
最小为 `0.7 * VCC`，即 3.5 V。STM32的3.3 V方向/PWM信号不满足保证值。可靠硬件必须
把TB6612逻辑VCC改为3.3 V，或在8路方向和4路PWM之间加入74AHCT/74HCT单向电平转换；
普通5 V供电的74HC和自动双向TXS系列不适合替代。仅修改软件不能消除这一电气风险。
