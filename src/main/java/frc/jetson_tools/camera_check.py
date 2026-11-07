#!/usr/bin/env python3
"""攝影機檢查：確認攝影機能開、實際解析度與幀率，並存一張截圖 snapshot.jpg。

用法：
  python camera_check.py                      # 預設 /dev/video0、1280x720、30fps
  python camera_check.py --device 1 --width 640 --height 480

建議先執行 `v4l2-ctl --list-formats-ext -d /dev/video0` 看攝影機支援哪些解析度與幀率。
"""
import argparse
import time

import cv2


def open_camera(device, width, height, fps):
    src = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
    # MJPG 通常才跑得到高解析度 + 高幀率
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="0", help="攝影機編號或路徑，例如 0 或 /dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frames", type=int, default=120, help="用來量測幀率的幀數")
    args = parser.parse_args()

    cap = open_camera(args.device, args.width, args.height, args.fps)
    if not cap.isOpened():
        print("❌ 打不開攝影機，用 `v4l2-ctl --list-devices` 確認編號")
        return

    ok, frame = cap.read()
    if not ok:
        print("❌ 攝影機有開但讀不到畫面")
        return

    h, w = frame.shape[:2]
    print(f"實際解析度：{w}x{h}（要求 {args.width}x{args.height}）")

    start = time.time()
    got = 0
    for _ in range(args.frames):
        ok, frame = cap.read()
        if ok:
            got += 1
    elapsed = time.time() - start
    print(f"實測幀率：{got / elapsed:.1f} FPS（{got}/{args.frames} 幀成功）")

    cv2.imwrite("snapshot.jpg", frame)
    print("已存截圖 snapshot.jpg，可用 scp 拿回筆電確認畫面")
    cap.release()


if __name__ == "__main__":
    main()
