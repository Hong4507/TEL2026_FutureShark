#!/usr/bin/env python3
"""攝影機檢查：確認攝影機能開、實際解析度與幀率，並存一張截圖 snapshot.jpg。

用法：
  python camera_check.py                      # 預設 /dev/video0、1280x720、30fps
  python camera_check.py --device 1 --width 640 --height 480
  python camera_check.py --exposure 150 --wb 4500   # 試固定曝光／白平衡，看 snapshot.jpg 亮度顏色
  python camera_check.py --device http://10.71.30.12:5802/   # Limelight 等網路攝影機的 MJPEG 串流

建議先執行 `v4l2-ctl --list-formats-ext -d /dev/video0` 看攝影機支援哪些解析度與幀率。
"""
import argparse
import time

import cv2


def is_stream(device):
    """網路攝影機（例如 Limelight 的 MJPEG 串流 http://10.71.30.12:5802/）。"""
    return str(device).startswith(("http://", "https://", "rtsp://"))


def open_camera(device, width, height, fps):
    if is_stream(device):
        # 網路串流：解析度、幀率、曝光都在攝影機那端（Limelight 網頁）設定，這裡只負責收
        cap = cv2.VideoCapture(device, cv2.CAP_FFMPEG)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)
        return cap
    src = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
    # MJPG 通常才跑得到高解析度 + 高幀率
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    # USB 斷線時 read() 不要卡太久（OpenCV 預設最多等 10 秒），讓程式能快點發現並重連
    if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 500)
    return cap


# V4L2 的 exposure_auto：1 = 手動，3 = 自動（光圈優先）。舊版 OpenCV 用 0.25 / 0.75 表示。
_V4L2_EXPOSURE_MANUAL = (1, 0.25)


def lock_camera(cap, exposure="lock", wb="lock", settle_s=1.5):
    """便宜 webcam 預設自動曝光、自動白平衡：燈光一變亮度和顏色就跳，YOLO 和亮燈判斷都會受影響。

    exposure / wb：
      "auto"  保持自動
      "lock"  先讓自動調 settle_s 秒，再鎖住當下的值（每次開機依現場燈光決定一次）
      數字    直接用這個值（exposure 單位通常是 100 µs；wb 是色溫 K）
    攝影機不支援的設定只印警告，不會讓程式失敗。回傳實際設定的摘要字串。
    """
    try:
        backend = cap.getBackendName()
    except Exception:
        backend = "V4L2"
    if backend != "V4L2":
        return f"網路串流（{backend}）：曝光／白平衡請在攝影機網頁設定"
    notes = []

    if cap.set(cv2.CAP_PROP_AUTOFOCUS, 0):
        notes.append("自動對焦關閉")

    if exposure == "lock" or wb == "lock":
        end = time.monotonic() + settle_s
        while time.monotonic() < end:
            cap.read()

    if exposure != "auto":
        value = cap.get(cv2.CAP_PROP_EXPOSURE) if exposure == "lock" else float(exposure)
        manual = any(cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, m) for m in _V4L2_EXPOSURE_MANUAL)
        if manual and value > 0 and cap.set(cv2.CAP_PROP_EXPOSURE, value):
            notes.append(f"曝光鎖定 {cap.get(cv2.CAP_PROP_EXPOSURE):g}")
        else:
            notes.append(f"⚠️ 曝光鎖定失敗（讀到 {value:g}），維持自動")
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    if wb != "auto":
        value = cap.get(cv2.CAP_PROP_WB_TEMPERATURE) if wb == "lock" else float(wb)
        if cap.set(cv2.CAP_PROP_AUTO_WB, 0) and value > 0 and cap.set(cv2.CAP_PROP_WB_TEMPERATURE, value):
            notes.append(f"白平衡鎖定 {cap.get(cv2.CAP_PROP_WB_TEMPERATURE):g} K")
        else:
            notes.append(f"⚠️ 白平衡鎖定失敗（讀到 {value:g}），維持自動")
            cap.set(cv2.CAP_PROP_AUTO_WB, 1)

    return "、".join(notes) if notes else "沒有可調的設定"


def add_lock_args(parser):
    parser.add_argument("--exposure", default="lock", help="auto／lock（開機時鎖住自動調好的值）／數字（通常 100 µs 為單位）")
    parser.add_argument("--wb", default="lock", help="白平衡：auto／lock／色溫數字（K）")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="0", help="攝影機編號、路徑或串流網址，例如 0、/dev/video0、http://10.71.30.12:5802/")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frames", type=int, default=120, help="用來量測幀率的幀數")
    add_lock_args(parser)
    args = parser.parse_args()

    cap = open_camera(args.device, args.width, args.height, args.fps)
    if not cap.isOpened():
        print("❌ 打不開攝影機，用 `v4l2-ctl --list-devices` 確認編號")
        return
    print(f"攝影機設定：{lock_camera(cap, args.exposure, args.wb)}")

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
