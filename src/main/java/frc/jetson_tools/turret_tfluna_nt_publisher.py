#!/usr/bin/env python3
"""讀取砲台正前方的 TF-Luna（I2C 接在 Jetson Orin Nano），發布到 NetworkTables 的 TurretLidar 表。

接線（Jetson 40-pin 排針，3.3V 邏輯，TF-Luna 的 I/O 也是 3.3V，不用轉準位）：
  TF-Luna 1 (+5V)  → 排針 pin 2 或 4（5V）
  TF-Luna 2 (SDA)  → 排針 pin 3（I2C SDA）
  TF-Luna 3 (SCL)  → 排針 pin 5（I2C SCL）
  TF-Luna 4 (GND)  → 排針 pin 6（GND）
  TF-Luna 5 (CFG)  → GND（接地才是 I2C 模式；空接是 UART 模式，I2C 會找不到）
  TF-Luna 6 (MUX)  → 不接
  Orin Nano 開發板 pin 3/5 是 /dev/i2c-7；pin 27/28 是 /dev/i2c-1。

先確認看得到（應該在 0x10 看到 10）：
  sudo apt install i2c-tools
  sudo i2cdetect -y -r 7
權限：sudo usermod -aG i2c zheng（重新登入後生效），否則要 sudo 才能讀。
套件：/home/zheng/yolo-env/bin/pip install smbus2

NT 表 TurretLidar（必須和 robot/subsystems/turretlidar/TurretLidarIONT.java 一致）：
  distance     double  距離（公尺，TF-Luna 原始值，沒加 offset）
  strength     double  訊號強度（< 100 不可靠，65535 = 過曝）
  temperature  double  晶片溫度（°C）
  heartbeat    int     每讀到一次正確資料 +1；機器人 0.25 秒沒變就當斷線
  i2cErrors    int     連續讀取失敗次數（0 = 正常）

用法：
  只看數值：python turret_tfluna_nt_publisher.py --print-only
  連主控制器：python turret_tfluna_nt_publisher.py --team 7130
  連筆電模擬：python turret_tfluna_nt_publisher.py --server <筆電IP>
"""
import argparse
import time

REG_DIST_LOW = 0x00
READ_LEN = 6  # 距離、強度、溫度各 2 bytes（little-endian）


def configure_nt_server(inst, team, server):
    """和 flysky_nt_publisher.py 相同：先試 10.TE.AM.2，再試 robot.local。"""
    if server:
        inst.setServer(server)
        return server
    servers = [f"10.{team // 100}.{team % 100}.2", "robot.local"]
    inst.setServer([(s, 5810) for s in servers])
    return " / ".join(servers)


class TFLuna:
    def __init__(self, bus, address):
        from smbus2 import SMBus, i2c_msg  # pip install smbus2

        self._msg = i2c_msg
        self.bus = SMBus(bus)
        self.address = address

    def read(self):
        """回傳 (距離 m, 強度, 溫度 °C)。I2C 失敗會丟 OSError。

        先寫暫存器位址、再連續讀 6 bytes（repeated start），和 SystemCore 上
        DualTFLunaRangeSensor 用 I2C.read() 的方式一樣。
        """
        write = self._msg.write(self.address, [REG_DIST_LOW])
        read = self._msg.read(self.address, READ_LEN)
        self.bus.i2c_rdwr(write, read)
        b = list(read)
        dist_cm = b[0] | (b[1] << 8)
        strength = b[2] | (b[3] << 8)
        temp = (b[4] | (b[5] << 8)) / 100.0
        return dist_cm / 100.0, float(strength), temp

    def close(self):
        self.bus.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bus", type=int, default=7, help="I2C bus 編號（Orin Nano 排針 pin 3/5 = 7）")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x10, help="TF-Luna 位址（預設 0x10）")
    parser.add_argument("--rate", type=float, default=100.0, help="讀取頻率 Hz（TF-Luna 預設 100 Hz 輸出）")
    parser.add_argument("--team", type=int, default=7130)
    parser.add_argument("--server", default=None, help="直接指定 NT 伺服器 IP（優先於 --team）")
    parser.add_argument("--print-only", action="store_true", help="只印出數值，不連 NetworkTables")
    args = parser.parse_args()

    pubs = None
    inst = None
    if not args.print_only:
        import ntcore

        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("jetson-turret-tfluna")
        print(f"NetworkTables 連線目標：{configure_nt_server(inst, args.team, args.server)}")
        table = inst.getTable("TurretLidar")
        # 預設 NT 每 100 ms 才送一次，改成 10 ms（同 vision / flysky publisher）
        opts = ntcore.PubSubOptions(periodic=0.01, sendAll=True)
        pubs = {
            "distance": table.getDoubleTopic("distance").publish(opts),
            "strength": table.getDoubleTopic("strength").publish(opts),
            "temperature": table.getDoubleTopic("temperature").publish(opts),
            "heartbeat": table.getIntegerTopic("heartbeat").publish(opts),
            "i2cErrors": table.getIntegerTopic("i2cErrors").publish(opts),
        }

    lidar = None
    period = 1.0 / args.rate
    heartbeat = 0
    errors = 0
    last_print = 0.0
    last_connected = None
    print(f"讀取 /dev/i2c-{args.bus} 位址 0x{args.address:02x}，Ctrl+C 結束")
    try:
        while True:
            start = time.monotonic()
            try:
                if lidar is None:
                    lidar = TFLuna(args.bus, args.address)
                dist, strength, temp = lidar.read()
                heartbeat += 1
                if errors:
                    print(f"\n✅ I2C 恢復（之前連續失敗 {errors} 次）")
                errors = 0
                if pubs:
                    pubs["distance"].set(dist)
                    pubs["strength"].set(strength)
                    pubs["temperature"].set(temp)
                    pubs["heartbeat"].set(heartbeat)
                if start - last_print > 0.2:
                    last_print = start
                    print(f"距離 {dist:5.2f} m  強度 {strength:6.0f}  溫度 {temp:5.1f}°C", end="\r")
            except OSError as e:
                errors += 1
                if errors == 1 or errors % 100 == 0:
                    print(f"\n⚠️ I2C 讀取失敗（{errors} 次）：{e}。檢查接線、CFG 腳接地、i2cdetect 看得到 0x10")
                if lidar is not None and errors % 100 == 0:
                    lidar.close()  # 重開 bus，接頭鬆掉再插回來時可以恢復
                    lidar = None
                if errors >= 100:
                    time.sleep(0.1)  # 一直失敗就放慢，不要狂刷
            if pubs:
                pubs["i2cErrors"].set(errors)

            if inst:
                inst.flush()
                connected = inst.isConnected()
                if connected != last_connected:
                    last_connected = connected
                    print("\n✅ NetworkTables 已連線" if connected else "\n⚠️ NetworkTables 未連線")

            time.sleep(max(0.0, period - (time.monotonic() - start)))
    except KeyboardInterrupt:
        print("\n結束")
    finally:
        if lidar is not None:
            lidar.close()


if __name__ == "__main__":
    main()
