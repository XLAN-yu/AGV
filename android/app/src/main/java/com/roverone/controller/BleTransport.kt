package com.roverone.controller

import android.annotation.SuppressLint
import android.bluetooth.*
import android.content.Context
import android.os.Build
import java.util.UUID
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

/** Android is the BLE central/GATT client; the Orange Pi bridge is the bonded peripheral. */
@SuppressLint("MissingPermission")
class BleTransport(
    context: Context,
    private val address: String,
    private val onText: (String) -> Unit,
    private val onEnded: (String) -> Unit,
) : TextTransport {
    private val manager = context.getSystemService(BluetoothManager::class.java)
    private val assembler = BleFrameAssembler()
    private val writeSlot = LatestWriteSlot<ByteArray>()
    private val lease = Executors.newSingleThreadScheduledExecutor()
    @Volatile private var lastApplicationWriteNanos = 0L
    @Volatile private var gatt: BluetoothGatt? = null
    @Volatile private var command: BluetoothGattCharacteristic? = null
    @Volatile private var open = false
    @Volatile private var mtu = 23

    fun connect(context: Context) {
        val adapter = manager.adapter ?: return onEnded("此手机不支持蓝牙")
        if (!adapter.isEnabled) return onEnded("请先开启蓝牙")
        val device = try { adapter.getRemoteDevice(address) }
        catch (_: IllegalArgumentException) { return onEnded("BLE 地址无效") }
        if (device.bondState != BluetoothDevice.BOND_BONDED) {
            device.createBond()
            return onEnded("请先完成系统蓝牙配对，再连接小车")
        }
        gatt = device.connectGatt(context, false, callback, BluetoothDevice.TRANSPORT_LE)
    }

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS && newState == BluetoothProfile.STATE_CONNECTED) {
                g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
                if (!g.requestMtu(247)) g.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                open = false; onEnded("BLE 已断开（${gattStatus(status)}）")
            }
        }
        override fun onMtuChanged(g: BluetoothGatt, value: Int, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) mtu = value
            g.discoverServices()
        }
        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            val service = g.getService(SERVICE_UUID)
            val tx = service?.getCharacteristic(COMMAND_UUID)
            val rx = service?.getCharacteristic(STATUS_UUID)
            if (status != BluetoothGatt.GATT_SUCCESS || tx == null || rx == null) {
                g.disconnect(); return onEnded("未找到 REMOTE AGV BLE 安全服务")
            }
            command = tx
            g.setCharacteristicNotification(rx, true)
            val descriptor = rx.getDescriptor(CCCD_UUID)
                ?: return onEnded("BLE 状态通知不可用")
            val enabled = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            if (Build.VERSION.SDK_INT >= 33) g.writeDescriptor(descriptor, enabled)
            else {
                @Suppress("DEPRECATION")
                descriptor.value = enabled
                @Suppress("DEPRECATION")
                g.writeDescriptor(descriptor)
            }
        }
        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            if (d.uuid == CCCD_UUID && status == BluetoothGatt.GATT_SUCCESS) {
                open = true
                writeFrame(BLE_CLAIM)
                lease.scheduleAtFixedRate({
                    if (open && System.nanoTime() - lastApplicationWriteNanos >= TimeUnit.MILLISECONDS.toNanos(500))
                        writeFrame(BLE_CLAIM)
                }, 750, 750, TimeUnit.MILLISECONDS)
            }
            else if (d.uuid == CCCD_UUID) onEnded("BLE 状态通知启用失败")
        }
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            receive(c, value)
        }
        @Deprecated("API 33 callback")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            @Suppress("DEPRECATION") receive(c, c.value)
        }
        private fun receive(c: BluetoothGattCharacteristic, bytes: ByteArray) {
            if (c.uuid != STATUS_UUID) return
            try { assembler.accept(bytes)?.let(onText) }
            catch (e: Exception) { cancel(); onEnded("BLE 状态分片无效：${e.message}") }
        }
        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                writeSlot.reset()
                cancel()
                onEnded("BLE 指令写入失败 ($status)")
                return
            }
            val next = writeSlot.complete() ?: return
            if (!issueWrite(g, c, next)) {
                writeSlot.reset()
                cancel()
                onEnded("BLE 指令写入启动失败")
            }
        }
    }

    override fun send(text: String): Boolean {
        val sent = writeFrame(text)
        if (sent) lastApplicationWriteNanos = System.nanoTime()
        return sent
    }

    private fun writeFrame(text: String): Boolean {
        val g = gatt ?: return false
        val c = command ?: return false
        val frames = try { BleFrameCodec.encode(text, mtu) } catch (_: Exception) { return false }
        if (!open || frames.size != 1) return false
        // Keep one write in flight and at most one waiting value. A new joystick
        // sample overwrites the waiting sample, so radio latency cannot build a
        // stale motion queue and does not cause an artificial disconnect.
        val immediate = writeSlot.submit(frames[0]) ?: return true
        if (issueWrite(g, c, immediate)) return true
        writeSlot.reset()
        return false
    }

    private fun issueWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, frame: ByteArray): Boolean {
        c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        val status = if (Build.VERSION.SDK_INT >= 33) g.writeCharacteristic(c, frame, c.writeType)
        else {
            @Suppress("DEPRECATION")
            c.value = frame
            @Suppress("DEPRECATION")
            if (g.writeCharacteristic(c)) BluetoothStatusCodes.SUCCESS else BluetoothStatusCodes.ERROR_UNKNOWN
        }
        return status == BluetoothStatusCodes.SUCCESS
    }
    override fun close() { open = false; writeSlot.reset(); lease.shutdownNow(); gatt?.disconnect() }
    override fun cancel() { open = false; writeSlot.reset(); lease.shutdownNow(); gatt?.close(); gatt = null; command = null }

    companion object {
        private fun gattStatus(status: Int): String = when (status) {
            BluetoothGatt.GATT_SUCCESS -> "远端已断开"
            8 -> "链路超时 status=8"
            19 -> "远端主动断开 status=19"
            22 -> "本机终止连接 status=22"
            133 -> "蓝牙协议栈异常 status=133"
            else -> "status=$status"
        }
        val SERVICE_UUID: UUID = UUID.fromString("7f510001-1b15-4ab5-9d6b-5b45cbb3a101")
        val COMMAND_UUID: UUID = UUID.fromString("7f510002-1b15-4ab5-9d6b-5b45cbb3a101")
        val STATUS_UUID: UUID = UUID.fromString("7f510003-1b15-4ab5-9d6b-5b45cbb3a101")
        private const val BLE_CLAIM = "{\"_ble\":\"claim\"}"
        private val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
    }
}
