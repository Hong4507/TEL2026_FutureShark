#!/usr/bin/env python3
"""九宮格視覺：攝影機 → YOLO → 換算角度 → 發布到 NetworkTables 的 Vision 表。

NT 表 Vision（格式必須和 robot/subsystems/vision/VisionConstants.java 一致）：
  targets    double[]  每個偵測 6 個數字：[類別, tx, ty, 信心度, 角寬度, 亮燈分數]
                       tx 往畫面右邊為正、ty 往上為正（rad，已去畸變）
                       亮燈分數：加分洞外圈紅色像素比例 0~1；大洞固定 -1
  heartbeat  int       每處理完一張畫面 +1（機器人用它判斷是否斷線）
  latencyMs  double    從拿到畫面到發布的時間
  fps        double    實際處理幀率

亮燈判斷不靠 YOLO：YOLO 只找出加分洞（bonus），再看洞外一圈有多少紅色像素。
這樣不需要亮燈的訓練資料，門檻在機器人端 /Tuning/Vision/LitThreshold 調整。

用法：
  python vision_nt_publisher.py --model best.engine --calib camera_calib.yaml --team 7130
  python vision_nt_publisher.py --model best.engine --calib camera_calib.yaml --print-only   # 不連 NT
  python vision_nt_publisher.py ... --save-dir captures --save-every 2   # 每 2 秒存一張原始畫面（收訓練資料）

即時畫面（畫好框）：瀏覽器開 http://<Jetson IP>:1181/ ，--stream-port 0 關閉。
"""
import argparse
import math
import os
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from camera_check import open_camera

CLASS_HOLE = 0
CLASS_BONUS = 1
STRIDE = 6


class LatestFrame:
    """背景執行緒一直讀攝影機，只保留最新一張，避免緩衝區造成延遲。"""

    def __init__(self, cap):
        self.cap = cap
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = 0.0
        self.seq = 0
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame, self.stamp, self.seq = frame, time.monotonic(), self.seq + 1

    def get(self, last_seq):
        """等到有比 last_seq 新的畫面，回傳 (frame, stamp, seq)。"""
        while True:
            with self.lock:
                if self.frame is not None and self.seq != last_seq:
                    return self.frame, self.stamp, self.seq
            time.sleep(0.001)


class Undistorter:
    """像素座標 → 角度。有校正檔就去畸變；沒有就用 --hfov 估一個針孔模型。"""

    def __init__(self, calib_path, width, height, hfov_deg):
        if calib_path:
            fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
            K = fs.getNode("camera_matrix").mat()
            dist = fs.getNode("dist_coeffs").mat()
            cw = int(fs.getNode("image_width").real())
            ch = int(fs.getNode("image_height").real())
            fs.release()
            if (cw, ch) != (width, height):
                print(f"⚠️ 校正解析度 {cw}x{ch} 和攝影機 {width}x{height} 不同，依比例縮放（建議重新校正）")
                K = K.copy()
                K[0, :] *= width / cw
                K[1, :] *= height / ch
            self.K, self.dist = K, dist
            print(f"校正檔：fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
        else:
            f = width / 2 / math.tan(math.radians(hfov_deg) / 2)
            self.K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], np.float64)
            self.dist = np.zeros(5)
            print(f"⚠️ 沒有校正檔，用水平視角 {hfov_deg}° 估計（f={f:.1f} px），角度會有誤差")

    def normalized(self, pts):
        """(N,2) 像素 → (N,2) 去畸變後的正規化座標 (x/z, y/z)。"""
        pts = np.asarray(pts, np.float64).reshape(-1, 1, 2)
        return cv2.undistortPoints(pts, self.K, self.dist).reshape(-1, 2)


def lit_score(frame, box):
    """加分洞外圈（燈條位置附近）的紅色像素比例。"""
    x0, y0, x1, y1 = box
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ax, ay = (x1 - x0) / 2, (y1 - y0) / 2
    # 燈條在洞外 5 cm，中心半徑約為洞半徑的 1.6 倍；取 1.25~2.0 倍的環
    rx, ry = ax * 2.0, ay * 2.0
    h, w = frame.shape[:2]
    X0, Y0 = int(max(cx - rx, 0)), int(max(cy - ry, 0))
    X1, Y1 = int(min(cx + rx + 1, w)), int(min(cy + ry + 1, h))
    if X1 - X0 < 4 or Y1 - Y0 < 4:
        return 0.0
    roi = frame[Y0:Y1, X0:X1]
    ring = np.zeros(roi.shape[:2], np.uint8)
    center = (int(cx - X0), int(cy - Y0))
    cv2.ellipse(ring, center, (int(rx), int(ry)), 0, 0, 360, 255, -1)
    cv2.ellipse(ring, center, (int(ax * 1.25), int(ay * 1.25)), 0, 0, 360, 0, -1)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red = (cv2.inRange(hsv, (0, 120, 120), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 120), (180, 255, 255)))
    total = cv2.countNonZero(ring)
    return cv2.countNonZero(red & ring) / total if total else 0.0


class MjpegServer:
    """很簡單的 MJPEG 串流，給操作手或除錯看框。只有在有人看的時候才編碼。"""

    def __init__(self, port):
        self.jpeg = None
        self.clients = 0
        self.cond = threading.Condition()
        server = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                with server.cond:
                    server.clients += 1
                try:
                    while True:
                        with server.cond:
                            server.cond.wait(timeout=1.0)
                            data = server.jpeg
                        if data is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with server.cond:
                        server.clients -= 1

            def log_message(self, *args):
                pass

        httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        print(f"即時畫面：http://<Jetson IP>:{port}/")

    def wants_frame(self):
        return self.clients > 0

    def publish(self, img):
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self.cond:
                self.jpeg = buf.tobytes()
                self.cond.notify_all()


def draw(frame, dets, fps):
    img = frame.copy()
    for cls, box, conf, lit in dets:
        x0, y0, x1, y1 = map(int, box)
        color = (0, 255, 0) if cls == CLASS_HOLE else (0, 200, 255)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
        label = f"{conf:.2f}" + (f" lit={lit:.2f}" if cls == CLASS_BONUS else "")
        cv2.putText(img, label, (x0, max(y0 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    cv2.putText(img, f"{fps:.1f} fps", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.line(img, (img.shape[1] // 2, 0), (img.shape[1] // 2, img.shape[0]), (255, 255, 255), 1)
    return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="best.engine（Jetson 上轉好的 TensorRT）或 best.pt")
    parser.add_argument("--calib", default=None, help="calibrate.py 產生的 camera_calib.yaml")
    parser.add_argument("--hfov", type=float, default=70.0, help="沒有校正檔時用的水平視角（度）")
    parser.add_argument("--device", default="0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--imgsz", type=int, default=640, help="要和匯出 engine 時一樣")
    parser.add_argument("--conf", type=float, default=0.25, help="Jetson 端的低門檻；機器人端另外有 MinConfidence")
    parser.add_argument("--team", type=int, default=7130)
    parser.add_argument("--server", default=None)
    parser.add_argument("--print-only", action="store_true", help="不連 NT，只印結果")
    parser.add_argument("--stream-port", type=int, default=1181)
    parser.add_argument("--save-dir", default=None, help="定時存原始畫面的資料夾（收訓練資料用）")
    parser.add_argument("--save-every", type=float, default=2.0, help="幾秒存一張")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model, task="detect")
    cap = open_camera(args.device, args.width, args.height, args.fps)
    if not cap.isOpened():
        print("❌ 打不開攝影機")
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"攝影機：{width}x{height}")
    undistort = Undistorter(args.calib, width, height, args.hfov)
    camera = LatestFrame(cap)
    stream = MjpegServer(args.stream_port) if args.stream_port else None

    pubs = None
    if not args.print_only:
        import ntcore
        from nt_test import configure_nt_server

        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("jetson-vision")
        print(f"NT 連線目標：{configure_nt_server(inst, args.team, args.server)}")
        table = inst.getTable("Vision")
        # NT4 預設每 0.1 s 才合併送一次 → 機器人最多晚 100 ms 才看到；改 10 ms + 每張 flush
        opts = ntcore.PubSubOptions(periodic=0.01, sendAll=True)
        pubs = {
            "targets": table.getDoubleArrayTopic("targets").publish(opts),
            "heartbeat": table.getIntegerTopic("heartbeat").publish(opts),
            "latencyMs": table.getDoubleTopic("latencyMs").publish(opts),
            "fps": table.getDoubleTopic("fps").publish(opts),
            "model": table.getStringTopic("model").publish(),
        }
        pubs["model"].set(os.path.basename(args.model))

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    last_save = 0.0
    heartbeat = 0
    seq = -1
    fps = 0.0
    last_t = time.monotonic()
    last_print = 0.0
    warmup = True

    try:
        while True:
            frame, stamp, seq = camera.get(seq)
            result = model.predict(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
            if warmup:
                warmup = False
                print("模型就緒")
                continue

            boxes = result.boxes
            xyxy = boxes.xyxy.cpu().numpy() if len(boxes) else np.zeros((0, 4))
            classes = boxes.cls.cpu().numpy().astype(int) if len(boxes) else np.zeros(0, int)
            confs = boxes.conf.cpu().numpy() if len(boxes) else np.zeros(0)

            flat, dets = [], []
            if len(xyxy):
                cxy = np.stack([(xyxy[:, 0] + xyxy[:, 2]) / 2, (xyxy[:, 1] + xyxy[:, 3]) / 2], axis=1)
                lft = np.stack([xyxy[:, 0], cxy[:, 1]], axis=1)
                rgt = np.stack([xyxy[:, 2], cxy[:, 1]], axis=1)
                nc, nl, nr = undistort.normalized(cxy), undistort.normalized(lft), undistort.normalized(rgt)
                for i in range(len(xyxy)):
                    cls = int(classes[i])
                    tx = math.atan(nc[i, 0])
                    ty = math.atan(-nc[i, 1] / math.sqrt(1 + nc[i, 0] ** 2))
                    ang_w = math.atan(nr[i, 0]) - math.atan(nl[i, 0])
                    lit = lit_score(frame, xyxy[i]) if cls == CLASS_BONUS else -1.0
                    flat += [float(cls), tx, ty, float(confs[i]), ang_w, lit]
                    dets.append((cls, xyxy[i], float(confs[i]), lit))

            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(now - last_t, 1e-3)
            last_t = now
            latency_ms = (now - stamp) * 1000
            heartbeat += 1
            if pubs:
                pubs["targets"].set(flat)
                pubs["latencyMs"].set(latency_ms)
                pubs["fps"].set(fps)
                pubs["heartbeat"].set(heartbeat)  # 最後才更新 heartbeat，機器人看到新值時其他欄位已經是新的
                inst.flush()  # 馬上送出，不等 periodic

            if stream and stream.wants_frame():
                stream.publish(draw(frame, dets, fps))
            if args.save_dir and now - last_save >= args.save_every:
                last_save = now
                name = datetime.now().strftime("cap_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
                cv2.imwrite(os.path.join(args.save_dir, name), frame)
            if args.print_only and now - last_print >= 0.5:
                last_print = now
                n_hole = sum(1 for d in dets if d[0] == CLASS_HOLE)
                lits = [f"{d[3]:.2f}" for d in dets if d[0] == CLASS_BONUS]
                print(f"{fps:5.1f} fps  延遲 {latency_ms:5.1f} ms  大洞 {n_hole}  加分洞亮燈分數 {lits}")
    except KeyboardInterrupt:
        pass
    finally:
        camera.running = False
        cap.release()


if __name__ == "__main__":
    main()
