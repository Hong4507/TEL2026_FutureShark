#!/usr/bin/env python3
"""假的九宮格偵測結果，格式和 vision_nt_publisher.py 一樣。用來在沒有模型時測 NT 連線與機器人端程式。

假設攝影機正對九宮格，距離 --distance 公尺，九宮格中欄在攝影機右邊 --offset 公尺。
--sweep 會讓 offset 左右來回擺動，可以在 AdvantageScope 看 Vision/ColumnErrorsMeters 有沒有跟著變。

用法：
  python vision_fake_publisher.py --team 7130 --offset 0.3 --lit 2
  python vision_fake_publisher.py --server 127.0.0.1 --sweep 0.5     # 筆電模擬（RobotContainer 要改用 VisionIONT）
"""
import argparse
import math
import time

import ntcore

from nt_test import configure_nt_server

COLUMN_SPACING_M = 0.90
ROW_HEIGHTS_M = [1.2, 1.8, 2.4]
BONUS_HEIGHT_M = 2.9
CAMERA_HEIGHT_M = 0.45


def build_targets(offset, distance, lit_column):
    flat = []
    for c in range(3):
        x = offset + (c - 1) * COLUMN_SPACING_M  # c=0 最左欄（畫面左邊）
        tx = math.atan2(x, distance)
        for z in ROW_HEIGHTS_M:
            ty = math.atan2(z - CAMERA_HEIGHT_M, distance)
            flat += [0.0, tx, ty, 0.9, 2 * math.atan2(0.2, distance), -1.0]
        ty = math.atan2(BONUS_HEIGHT_M - CAMERA_HEIGHT_M, distance)
        lit = 0.6 if c == lit_column else 0.01
        flat += [1.0, tx, ty, 0.9, 2 * math.atan2(0.1, distance), lit]
    return flat


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--team", type=int, default=7130)
    parser.add_argument("--server", default=None)
    parser.add_argument("--offset", type=float, default=0.0, help="中欄在攝影機右邊幾公尺")
    parser.add_argument("--distance", type=float, default=2.5)
    parser.add_argument("--lit", type=int, default=0, help="亮燈的欄：0 左、1 中、2 右，-1 不亮")
    parser.add_argument("--sweep", type=float, default=0.0, help="offset 來回擺動的幅度（公尺）")
    parser.add_argument("--rate", type=float, default=30.0)
    args = parser.parse_args()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("jetson-vision-fake")
    print(f"NT 連線目標：{configure_nt_server(inst, args.team, args.server)}，Ctrl+C 結束")
    table = inst.getTable("Vision")
    # NT4 預設每 0.1 s 才合併送一次 → 機器人最多晚 100 ms 才看到；改 10 ms + 每張 flush
    opts = ntcore.PubSubOptions(periodic=0.01, sendAll=True)
    targets = table.getDoubleArrayTopic("targets").publish(opts)
    heartbeat = table.getIntegerTopic("heartbeat").publish(opts)
    latency = table.getDoubleTopic("latencyMs").publish(opts)
    fps = table.getDoubleTopic("fps").publish(opts)
    table.getStringTopic("model").publish().set("fake")

    count = 0
    start = time.monotonic()
    try:
        while True:
            t = time.monotonic() - start
            offset = args.offset + args.sweep * math.sin(t * 0.5)
            targets.set(build_targets(offset, args.distance, args.lit))
            latency.set(5.0)
            fps.set(args.rate)
            count += 1
            heartbeat.set(count)
            inst.flush()
            if count % int(args.rate) == 0:
                print(f"offset={offset:+.2f} m  connected={inst.isConnected()}")
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
