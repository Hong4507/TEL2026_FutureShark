#!/usr/bin/env python3
"""從 record.py 錄的影片抽出要標註的畫面，自動跳過幾乎一樣的畫面。

輸出檔名是「影片名_f幀號.jpg」，train.py 會用影片名分訓練／驗證集（同一段影片不會同時出現在兩邊）。

用法：
  python extract_frames.py recordings/*.mp4 --out frames --every 10
  python extract_frames.py recordings/ --out frames --every 5 --diff 6
"""
import argparse
import glob
import os

import cv2
import numpy as np


def video_files(inputs):
    files = []
    for path in inputs:
        if os.path.isdir(path):
            for ext in ("mp4", "avi", "mkv", "mov"):
                files += glob.glob(os.path.join(path, f"*.{ext}"))
        else:
            files += glob.glob(path)
    return sorted(files)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", help="影片檔或資料夾")
    parser.add_argument("--out", default="frames")
    parser.add_argument("--every", type=int, default=10, help="每幾幀檢查一次")
    parser.add_argument("--diff", type=float, default=4.0, help="和上一張存下來的畫面平均差異小於這個值就跳過")
    parser.add_argument("--max-per-video", type=int, default=150)
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    total = 0
    for path in video_files(args.inputs):
        name = os.path.splitext(os.path.basename(path))[0]
        cap = cv2.VideoCapture(path)
        last = None
        saved = 0
        idx = -1
        while saved < args.max_per_video:
            ok, frame = cap.read()
            if not ok:
                break
            idx += 1
            if idx % args.every:
                continue
            small = cv2.cvtColor(cv2.resize(frame, (160, 90)), cv2.COLOR_BGR2GRAY).astype(np.float32)
            if last is not None and np.mean(np.abs(small - last)) < args.diff:
                continue
            last = small
            cv2.imwrite(os.path.join(args.out, f"{name}_f{idx:05d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            saved += 1
        cap.release()
        total += saved
        print(f"{name}: {saved} 張")
    print(f"共 {total} 張，存在 {args.out}")


if __name__ == "__main__":
    main()
