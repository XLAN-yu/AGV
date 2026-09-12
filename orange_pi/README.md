# 香橙派端

`gateway/` 是 WebSocket 到 STM32 UART 的主网关，负责协议校验、控制权仲裁和看门狗；`ble/` 是连接到同一网关的 BLE 备用传输。两者共享 STM32 的安全链路。

部署步骤、环境变量与服务文件分别见 [gateway/README.md](gateway/README.md) 和 [ble/README.md](ble/README.md)。设备上的令牌、网络参数和 systemd 私有环境文件应在目标设备单独配置，不保存在仓库中。
