#!/usr/bin/env python3
"""相機校正（棋盤格），全部可在 SSH 下完成，不需要螢幕。

步驟一：拍照（拿棋盤格在攝影機前慢慢移動，程式偵測到棋盤格就自動存一張）
  python calibrate.py capture --count 25
  每張之間會間隔 1.5 秒，請換不同位置、距離、傾斜角度；畫面四個角落都要拍到。

步驟二：計算內參
  python calibrate.py solve
  結果存成 camera_calib.yaml（相機矩陣 K、畸變係數、重投影誤差）。
  重投影誤差（RMS）小於 0.5 px 很好，小於 1 px 可用；太大就刪掉模糊的照片重算。

預設棋盤格：內角點 9x6（也就是 10x7 個方格），方格邊長 25 mm。
⚠️ 列印後務必用尺量方格實際大小，用 --square 填入實際值（mm）。
⚠️ 拍照與比賽時的解析度必須相同，校正結果只適用於該解析度。
"""
import argparse
import glob
import os
import time

import cv2
import numpy as np

CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def find_corners(gray, cols, rows):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if found:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
    return found, corners


def capture(args):
    from camera_check import open_camera

    cap = open_camera(args.device, args.width, args.height, 30)
    if not cap.isOpened():
        print("❌ 打不開攝影機")
        return
    os.makedirs(args.dir, exist_ok=True)
    saved = len(glob.glob(os.path.join(args.dir, "*.png")))
    last = 0.0
    print(f"開始拍攝，目標 {args.count} 張（Ctrl+C 提前結束）")
    try:
        while saved < args.count:
            ok, frame = cap.read()
            if not ok:
                continue
            if time.time() - last < args.interval:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, _ = find_corners(gray, args.cols, args.rows)
            if found:
                path = os.path.join(args.dir, f"calib_{saved:02d}.png")
                cv2.imwrite(path, frame)
                saved += 1
                last = time.time()
                print(f"  ✅ 已存 {path}（{saved}/{args.count}）→ 請換個位置或角度")
    except KeyboardInterrupt:
        pass
    cap.release()
    print(f"拍攝結束，共 {saved} 張。接著執行：python calibrate.py solve")


def solve(args):
    paths = sorted(glob.glob(os.path.join(args.dir, "*.png")) + glob.glob(os.path.join(args.dir, "*.jpg")))
    if not paths:
        print(f"❌ {args.dir}/ 裡沒有照片")
        return

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * (args.square / 1000.0)

    obj_points, img_points, used = [], [], []
    size = None
    for p in paths:
        img = cv2.imread(p)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if size is None:
            size = gray.shape[::-1]
        elif gray.shape[::-1] != size:
            print(f"  ⚠️ {p} 解析度不同，略過")
            continue
        found, corners = find_corners(gray, args.cols, args.rows)
        if found:
            obj_points.append(objp)
            img_points.append(corners)
            used.append(p)
        else:
            print(f"  ⚠️ {p} 找不到棋盤格，略過")

    if len(used) < 10:
        print(f"❌ 只有 {len(used)} 張可用，至少需要 10 張（建議 20 張以上）")
        return

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

    print("\n各張照片的重投影誤差（px），特別大的照片可以刪掉重算：")
    for p, op, ip, rv, tv in zip(used, obj_points, img_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        diff = ip.reshape(-1, 2) - proj.reshape(-1, 2)
        err = float(np.sqrt((diff ** 2).sum(axis=1).mean()))
        print(f"  {os.path.basename(p)}: {err:.3f}")

    fx, fy = K[0, 0], K[1, 1]
    hfov = 2 * np.degrees(np.arctan(size[0] / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(size[1] / (2 * fy)))

    fs = cv2.FileStorage(args.output, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", size[0])
    fs.write("image_height", size[1])
    fs.write("camera_matrix", K)
    fs.write("dist_coeffs", dist)
    fs.write("rms", float(rms))
    fs.release()

    print(f"\n使用 {len(used)} 張，解析度 {size[0]}x{size[1]}")
    print(f"整體重投影誤差 RMS = {rms:.3f} px")
    print(f"fx={fx:.1f} fy={fy:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
    print(f"視角：水平 {hfov:.1f}°，垂直 {vfov:.1f}°")
    print(f"結果已存到 {args.output}")


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("capture", "solve"):
        sp = sub.add_parser(name)
        sp.add_argument("--cols", type=int, default=9, help="每列內角點數")
        sp.add_argument("--rows", type=int, default=6, help="每行內角點數")
        sp.add_argument("--square", type=float, default=25.0, help="方格實際邊長（mm）")
        sp.add_argument("--dir", default="calib_images")
        if name == "capture":
            sp.add_argument("--device", default="0")
            sp.add_argument("--width", type=int, default=1280)
            sp.add_argument("--height", type=int, default=720)
            sp.add_argument("--count", type=int, default=25)
            sp.add_argument("--interval", type=float, default=1.5)
        else:
            sp.add_argument("--output", default="camera_calib.yaml")
    args = parser.parse_args()
    capture(args) if args.cmd == "capture" else solve(args)


if __name__ == "__main__":
    main()
