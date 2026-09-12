# ROVER ONE BLE 备用控制桥

STM32 可在 STATUS 帧的 `flags bit 0` 发出配对请求。网关只向
`127.0.0.1:45991` 发送版本化 UDP 事件，BLE 服务收到后运行
`pair-3min.sh`，开放 180 秒可发现/可配对窗口。重复状态帧会被网关限流，
窗口期间的重复请求也不会创建第二个配对进程；该通路不读写 UART 控制帧，
也不修改 ESTOP、超时停车或控制租约。

BLE 只替代“手机到香橙派”这一段。桥接服务不访问 UART，也不生成 STM32 帧；它以普通单一控制客户端连接现有 `ws://127.0.0.1:8000/ws`，把重组后的原始 JSON 原样转发。因此现有网关 200 ms 看门狗、严格字段校验、单控制端仲裁、UART CRC/ACK，以及 STM32 300 ms 独立看门狗都保持不变。

## 安装

Ubuntu 24.04 上需要可用的 BlueZ 适配器与 Python 3.12：

```bash
sudo apt install bluetooth bluez python3-venv
sudo install -d -m 0755 /opt/rover-one-ble
sudo cp bridge.py frame_codec.py requirements.txt /opt/rover-one-ble/
sudo python3 -m venv /opt/rover-one-ble/.venv
sudo /opt/rover-one-ble/.venv/bin/pip install -r /opt/rover-one-ble/requirements.txt
sudo install -m 0644 rover-ble-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rover-ble-bridge.service
```

安装脚本会从 `/etc/rover-one/gateway.env` 读取第一个 `ROVER_ALLOWED_ORIGINS`，写入 `/etc/rover-one/ble.env`。如果之后修改网关 Origin，需同步更新该文件再重启 BLE 服务；桥接不会放宽网关白名单。

在 `bluetoothctl` 中启用代理并只与控制手机配对。命令特征要求 BlueZ 加密写入，Android App 也要求设备已经处于系统“已配对”状态才会连接。

无屏首次配对可执行：

```bash
sudo /opt/rover-one-ble/pair-3min.sh
```

脚本在前台注册 `NoInputNoOutput` 配对代理，开放 180 秒后自动关闭可发现和可配对状态。手机在窗口内从系统蓝牙设置选择 `ROVER-ONE-BLE`。

## 安全行为

- App 每 750 ms 更新一次 BLE 控制租约；1.5 s 无租约时桥接主动关闭 WebSocket。
- WebSocket 关闭会触发现有网关归零；即使香橙派失效，STM32 仍必须在 300 ms 无有效 UART 控制帧后独立关闭 PWM/STBY。
- BLE 与 Wi-Fi 共用现有网关的单控制端互斥，不能同时取得控制权。
- 物理 ESTOP 仍直接切断电机使能；BLE `clear_estop` 仍需 STM32 ACK，不能绕过物理急停或硬件故障。

GATT UUID：服务 `7f510001-1b15-4ab5-9d6b-5b45cbb3a101`，命令写入 `...0002...`，状态通知 `...0003...`。ATT 内容采用 5 字节小端分片头（版本、消息 ID、分片序号、总数）加 UTF-8 JSON；JSON 本身与现有 WebSocket 协议完全一致。
