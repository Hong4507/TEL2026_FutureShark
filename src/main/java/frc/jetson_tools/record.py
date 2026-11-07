#!/usr/bin/env python3
"""錄製訓練用影片（不需要螢幕，SSH 下就能用）。

用法：
  python record.py --name near_left               # 錄到按 Ctrl+C 為止
  python record.py --name far_center --seconds 60  # 錄 60 秒

影片存在 recordings/，檔名含名稱與時間，例如 recordings/near_left_20261108_143015.mp4。
請用「距離_角度_情況」命名（例如 3m_left30_backlight），之後分訓練／驗證集時比較好整理。
解析度請設成比賽時要用的設定。
"""
import argparse
import os
import time
from datetime import datetime

import cv2

from camera_check import open_camera


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True, help="這段影片的描述，例如 3m_left30_normal")
    parser.add_argument("--device", default="0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--seconds", type=float, default=0, help="錄影秒數，0 代表錄到 Ctrl+C")
    parser.add_argument("--outdir", default="recordings")
    args = parser.parse_args()

    cap = open_camera(args.device, args.width, args.height, args.fps)
    if not cap.isOpened():
        print("❌ 打不開攝影機")
        return
    ok, frame = cap.read()
    if not ok:
        print("❌ 讀不到畫面")
        return
    h, w = frame.shape[:2]

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(args.outdir, f"{args.name}_{stamp}.mp4")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))

    print(f"開始錄影 {w}x{h} → {path}（Ctrl+C 停止）")
    start = time.time()
    count = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("⚠️ 讀取失敗，停止錄影")
                break
            writer.write(frame)
            count += 1
            elapsed = time.time() - start
            if count % args.fps == 0:
                print(f"  已錄 {elapsed:.0f} 秒，{count} 幀", end="\r")
            if args.seconds and elapsed >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        writer.release()
        cap.release()
    print(f"\n完成：{path}，共 {count} 幀，{time.time() - start:.1f} 秒")


if __name__ == "__main__":
    main()
