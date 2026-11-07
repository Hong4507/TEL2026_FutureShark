#!/usr/bin/env python3
"""讀取 FlySky 接收機的 i-BUS 訊號，發布到 NetworkTables 的 FlySky 表。

接線（USB 轉 TTL 模組）：
  接收機 i-BUS 的訊號腳 → USB-TTL 的 RX
  接收機 GND           → USB-TTL 的 GND
  接收機 5V            → USB-TTL 的 5V（或其他 5V 電源，要共地）
  USB-TTL 的 TX 不用接

NT 表 FlySky：
  channels   double[]  各通道脈寬（μs，約 1000 ~ 2000），index 0 = CH1
                       ※ 不要先正規化！機器人端 FlySkyController 會自己做 (us-1500)/500
  raw        int[]     同上，整數版（除錯用）
  valid      boolean   最近 0.1 秒內有收到正確封包（除錯用，機器人端不讀）
  heartbeat  int       每收到一個正確封包就 +1，主控制器用它判斷資料是否過期（逾時 0.1 秒）

這個格式必須和 robot/controller/FlySkyController.java 一致（kTableName / kChannelsTopic /
kHeartbeatTopic）。

用法：
  只看數值（不連 NT，用來對照搖桿與通道）：python flysky_nt_publisher.py --print-only
  連主控制器：                              python flysky_nt_publisher.py --team 7130
  連筆電模擬：                              python flysky_nt_publisher.py --server <筆電IP>
"""
import argparse
import time

IBUS_FRAME_LEN = 32
IBUS_HEADER = (0x20, 0x40)
STALE_SEC = 0.1


class IBusParser:
    """把序列埠收到的位元組切成 i-BUS 封包並驗證 checksum。"""

    def __init__(self, num_channels=14):
        self.num_channels = num_channels
        self.buf = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        """餵入新的位元組，回傳這次解出來的所有封包（每個封包是通道值的 list）。"""
        self.buf.extend(data)
        frames = []
        while len(self.buf) >= IBUS_FRAME_LEN:
            if self.buf[0] != IBUS_HEADER[0] or self.buf[1] != IBUS_HEADER[1]:
                del self.buf[0]  # 不同步，往後找標頭
                continue
            frame = self.buf[:IBUS_FRAME_LEN]
            checksum = 0xFFFF - (sum(frame[:30]) & 0xFFFF)
            received = frame[30] | (frame[31] << 8)
            if checksum != received:
                self.bad_frames += 1
                del self.buf[0]
                continue
            channels = [
                (frame[2 + 2 * i] | (frame[3 + 2 * i] << 8)) & 0x0FFF
                for i in range(self.num_channels)
            ]
            frames.append(channels)
            del self.buf[:IBUS_FRAME_LEN]
        return frames


def configure_nt_server(inst, team, server):
    """設定 NT 伺服器位址。

    指定 --server 時只連那個 IP。否則依序嘗試 10.TE.AM.2 與 robot.local。
    舊版 pyntcore 的 setServerTeam 只會找 roboRIO 的主機名稱，找不到 SystemCore，所以這裡自己列。
    回傳一段描述文字，用來顯示。
    """
    if server:
        inst.setServer(server)
        return server
    servers = [f"10.{team // 100}.{team % 100}.2", "robot.local"]
    inst.setServer([(s, 5810) for s in servers])
    return " / ".join(servers)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="USB-TTL 的序列埠")
    parser.add_argument("--channels", type=int, default=10, help="要發布的通道數（i-BUS 最多 14）")
    parser.add_argument("--team", type=int, default=7130)
    parser.add_argument("--server", default=None, help="直接指定 NT 伺服器 IP（優先於 --team）")
    parser.add_argument("--print-only", action="store_true", help="只印出數值，不連 NetworkTables")
    args = parser.parse_args()
    if not 6 <= args.channels <= 14:
        parser.error("--channels 必須在 6 ~ 14 之間（機器人端會讀 CH1 ~ CH6）")

    import serial  # pip install pyserial

    ser = serial.Serial(args.port, 115200, timeout=0.01)
    ibus = IBusParser()

    pubs = None
    inst = None
    if not args.print_only:
        import ntcore

        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("jetson-flysky")
        target = configure_nt_server(inst, args.team, args.server)
        print(f"NetworkTables 連線目標：{target}")
        table = inst.getTable("FlySky")
        # 預設 NT 每 100 ms 才送一次，遙控器要更即時，改成 10 ms
        opts = ntcore.PubSubOptions(periodic=0.01)
        pubs = {
            "channels": table.getDoubleArrayTopic("channels").publish(opts),
            "raw": table.getIntegerArrayTopic("raw").publish(opts),
            "valid": table.getBooleanTopic("valid").publish(opts),
            "heartbeat": table.getIntegerTopic("heartbeat").publish(opts),
        }
        pubs["valid"].set(False)

    print(f"讀取 {args.port}，Ctrl+C 結束")
    heartbeat = 0
    last_frame_time = 0.0
    last_print = 0.0
    last_valid = None
    last_connected = None
    try:
        while True:
            data = ser.read(64)
            frames = ibus.feed(data) if data else []
            now = time.monotonic()

            if frames:
                raw = frames[-1][: args.channels]
                last_frame_time = now
                heartbeat += 1
                if pubs:
                    pubs["raw"].set(raw)
                    pubs["channels"].set([float(v) for v in raw])  # μs，機器人端再正規化
                    pubs["heartbeat"].set(heartbeat)
                if now - last_print > 0.2:
                    last_print = now
                    shown = " ".join(f"CH{i + 1}:{v:4d}" for i, v in enumerate(raw))
                    print(shown, end="\r")

            valid = (now - last_frame_time) < STALE_SEC
            if valid != last_valid:
                last_valid = valid
                if pubs:
                    pubs["valid"].set(valid)
                if not valid:
                    print("\n⚠️ 沒有收到 i-BUS 訊號（檢查接線、接收機是否有電、遙控器是否開機）")

            if inst:
                inst.flush()
                connected = inst.isConnected()
                if connected != last_connected:
                    last_connected = connected
                    if connected:
                        conns = ", ".join(f"{c.remote_ip}:{c.remote_port}" for c in inst.getConnections())
                        print(f"\n✅ NetworkTables 已連線：{conns}")
                    else:
                        print("\n⚠️ NetworkTables 未連線（檢查 Jetson 與 SystemCore 的網路、機器人程式是否在跑）")
    except KeyboardInterrupt:
        print("\n結束")
    finally:
        if pubs:
            pubs["valid"].set(False)
            inst.flush()
        ser.close()


if __name__ == "__main__":
    main()
