# AGV

四轮麦克纳姆小车的完整源码，包含 Android 控制 App、香橙派通信网关与 BLE 备用链路、STM32F407 控制固件。控制命令经过网关的单客户端仲裁与看门狗，再由 STM32 执行急停、通信超时及超声波避障等硬件安全逻辑。

## 目录

| 目录 | 内容 |
| --- | --- |
| `android/` | Kotlin + Jetpack Compose 控制 App（版本 1.6.6） |
| `orange_pi/gateway/` | FastAPI WebSocket ↔ UART 网关、摄像头跟随与部署脚本 |
| `orange_pi/ble/` | BlueZ BLE 备用控制桥 |
| `stm32/` | STM32F407VET6 的 Keil / STM32CubeMX 工程、协议与电机控制 |

## 构建

Android：使用 Android Studio 打开 `android/`，由本机生成 `local.properties`，然后运行 `gradlew.bat testDebugUnitTest assembleDebug`（Windows）或 `./gradlew testDebugUnitTest assembleDebug`。

香橙派网关：在 `orange_pi/gateway/` 创建 Python 虚拟环境并安装 `requirements-runtime.txt`；运行测试需安装 `requirements-dev.txt`。部署前应阅读该目录的 README 与 systemd 配置。

STM32：使用 Keil 打开 `stm32/MDK-ARM/AVG_V1.uvprojx`，选择 `AVG_V1` 目标并执行 Rebuild。固件所需的 HAL、CMSIS 和 FreeRTOS 源码已随工程保留；Keil Device Pack 由本机安装。烧录前请核对实际接线与轮位。

## 当前轮位

| 物理驱动 | 车轮位置 | 正转极性 |
| --- | --- | --- |
| C | 左前 | + |
| A | 右前 | − |
| B | 左后 | − |
| D | 右后 | + |

实际车体首次测试应架空四轮，使用 App 单轮校准确认映射、转向和编码器，再进行地面运动验收。App 的校准界面与 STM32 固件必须使用支持 `WHEEL_TEST` 的版本。

仓库仅包含源码与构建所需配置；本机 SDK 路径、虚拟环境、构建结果、历史固件包和设备私有令牌不提交。
