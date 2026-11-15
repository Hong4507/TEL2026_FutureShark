#!/usr/bin/env python3
"""九宮格視覺：攝影機 → YOLO → 換算角度 → 發布到 NetworkTables 的 Vision 表。

NT 表 Vision（格式必須和 robot/subsystems/vision/VisionConstants.java 一致）：
  targets    double[]  每個偵測 6 個數字：[類別, tx, ty, 信心度, 角寬度, 亮燈分數]
                       tx 往畫面右邊為正、ty 往上為正（rad，已去畸變）
                       亮燈分數：加分洞外圈紅色像素比例 0~1；大洞固定 -1
  heartbeat  int       每處理完一張畫面 +1（機器人用它判斷是否斷線）
  latencyMs  double    從拿到畫面到發布的時間
  fps        double    實際處理幀率

亮燈判斷不靠 YOLO：YOLO 只找出加分洞（bonus），再看洞外一圈有多少紅色像素（黑白攝影機改看亮像素）。
這樣不需要亮燈的訓練資料，門檻在機器人端 /Tuning/Vision/LitThreshold 調整。

用法：
  python vision_nt_publisher.py --model best.engine --calib camera_calib.yaml --team 7130
  python vision_nt_publisher.py --model best.engine --calib camera_calib.yaml --print-only   # 不連 NT
  python vision_nt_publisher.py ... --record                 # 存圖模式：每秒 2.5 張原始畫面 → ~/captures/YYYYMMDD/HHMMSS_mmm.jpg
  python vision_nt_publisher.py ... --record --record-fps 3  # 也可以不加 --record，改在 NT /Vision/Record 設 true

存圖模式（Robot-Brief §8.10，收 YOLO 訓練資料）：
  開關     --record 或 NT /Vision/Record（boolean，預設 false），任一個開著就存
  輸出     /Vision/Recording（boolean）、/Vision/RecordCount（本次開機存了幾張）、/Vision/RecordFreeGB、
           /Vision/RecordStatus（給 Elastic 的文字）、/Vision/RecordStopReason
  保護     剩餘空間 < --record-min-free-gb（5 GB）自動停止；另一個執行緒寫檔，佇列滿就丟幀，不拖慢偵測

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

from camera_check import add_lock_args, is_stream, lock_camera, open_camera

CLASS_HOLE = 0
CLASS_BONUS = 1
STRIDE = 6


class LatestFrame:
    """背景執行緒一直讀攝影機，只保留最新一張，避免緩衝區造成延遲。

    USB 斷線時（OV9281 實測約每 10 秒一次，約 0.5~1 秒後重新出現）在這裡直接重開攝影機，
    不必整個程式重啟、重新載入模型（那要 5~10 秒）。
    """

    def __init__(self, cap, reopen=None, timeout_s=10.0, reconnect_after_s=0.3):
        self.cap = cap
        self.reopen = reopen
        self.timeout_s = timeout_s
        self.reconnect_after_s = reconnect_after_s
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = 0.0
        self.seq = 0
        self.reconnects = 0
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_ok = time.monotonic()
        while self.running:
            ok, frame = self.cap.read() if self.cap is not None else (False, None)
            now = time.monotonic()
            if ok:
                last_ok = now
                with self.lock:
                    self.frame, self.stamp, self.seq = frame, now, self.seq + 1
                continue
            if self.reopen is None or now - last_ok < self.reconnect_after_s:
                time.sleep(0.01)
                continue
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            cap = self.reopen()
            if cap is None:
                time.sleep(0.2)  # 裝置還沒重新出現
                continue
            self.cap = cap
            self.reconnects += 1
            print(f"⚠️ 攝影機斷線 {now - last_ok:.1f} 秒後重新連上（第 {self.reconnects} 次）")
            last_ok = time.monotonic()

    def release(self):
        self.running = False
        if self.cap is not None:
            self.cap.release()

    def get(self, last_seq):
        """等到有比 last_seq 新的畫面，回傳 (frame, stamp, seq)。

        超過 timeout_s 一直重連不上就丟例外讓程式結束，交給 systemd 的 Restart=always；
        否則程式會卡在這裡、服務看起來還活著。
        """
        start = time.monotonic()
        while True:
            with self.lock:
                if self.frame is not None and self.seq != last_seq:
                    return self.frame, self.stamp, self.seq
            limit = self.timeout_s if self.frame is not None else self.timeout_s + 3  # 第一張多等一點
            if time.monotonic() - start > limit:
                raise RuntimeError(f"❌ 攝影機 {limit:.0f} 秒沒有新畫面、重連也失敗，結束讓 systemd 重啟")
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


# 黑白攝影機（OV9281）看不到紅色：畫面三個色彩通道幾乎一樣時，改數「很亮」的像素。
# 九宮格板子是黑的，洞外一圈亮起來就是燈條；門檻一樣在機器人端 /Tuning/Vision/LitThreshold 調。
MONO_CHANNEL_DIFF = 3.0
MONO_LIT_MIN = 200


def is_mono(img):
    s = img[::4, ::4].astype(np.int16)
    return np.abs(s[..., 0] - s[..., 1]).mean() < MONO_CHANNEL_DIFF and np.abs(s[..., 1] - s[..., 2]).mean() < MONO_CHANNEL_DIFF


def lit_score(frame, box):
    """加分洞外圈（燈條位置附近）的紅色像素比例；黑白畫面改用亮像素比例。"""
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
    if is_mono(roi):
        red = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), MONO_LIT_MIN, 255)
    else:
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


class Recorder:
    """存圖模式：主迴圈只把原始畫面丟進佇列（不複製、不編碼），另一個執行緒負責 JPEG 編碼和寫檔。

    佇列滿（磁碟太慢）就直接丟掉這一張，偵測迴圈永遠不會等存圖。
    """

    def __init__(self, root, fps, quality=90, min_free_gb=5.0, queue_size=4):
        import queue

        self.root = os.path.expanduser(root)
        self.period = 1.0 / fps if fps > 0 else float("inf")
        self.quality = quality
        self.min_free_gb = min_free_gb
        self.queue = queue.Queue(maxsize=queue_size)
        self.count = 0
        self.dropped = 0
        self.errors = 0
        self.stop_reason = ""
        self.free_gb = self._free_gb()
        self.last_offer = 0.0
        self.last_free_check = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def _free_gb(self):
        path = self.root
        while not os.path.exists(path):
            parent = os.path.dirname(path)
            if parent == path:
                break
            path = parent
        st = os.statvfs(path)
        return st.f_bavail * st.f_frsize / 1e9

    def offer(self, frame, now, enabled):
        """主迴圈每張畫面呼叫一次；到時間才丟進佇列。回傳 (是否錄影中, 狀態文字)。"""
        import queue

        if now - self.last_free_check >= 2.0:
            self.last_free_check = now
            try:
                self.free_gb = self._free_gb()
            except OSError:
                pass
        if not enabled:
            return False
        if self.free_gb < self.min_free_gb:
            if not self.stop_reason:
                self.stop_reason = f"剩餘空間 {self.free_gb:.1f} GB < {self.min_free_gb:.0f} GB"
                print(f"⚠️ 停止存圖：{self.stop_reason}")
            return False
        self.stop_reason = ""
        if now - self.last_offer < self.period:
            return True
        self.last_offer = now
        try:
            # LatestFrame 每張都是新的 ndarray、不會被改寫，所以不用 copy
            self.queue.put_nowait((frame, datetime.now()))
        except queue.Full:
            self.dropped += 1
        return True

    def _run(self):
        while True:
            frame, stamp = self.queue.get()
            folder = os.path.join(self.root, stamp.strftime("%Y%m%d"))
            path = os.path.join(folder, stamp.strftime("%H%M%S_") + f"{stamp.microsecond // 1000:03d}.jpg")
            try:
                os.makedirs(folder, exist_ok=True)
                if cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality]):
                    self.count += 1
                else:
                    self.errors += 1
            except OSError as e:
                self.errors += 1
                if self.errors <= 3:
                    print(f"⚠️ 存圖失敗：{e}")

    def status_text(self, recording):
        if self.stop_reason:
            return f"停止：空間不足 {self.free_gb:.0f}GB"
        if recording:
            return f"● 錄影 {self.count} 張"
        return f"未錄影（{self.count} 張）"


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
    parser.add_argument("--device", default="0", help="USB 攝影機路徑，或網路串流網址（Limelight：http://10.71.30.12:5802/）")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--imgsz", type=int, default=640, help="要和匯出 engine 時一樣")
    parser.add_argument("--conf", type=float, default=0.25, help="Jetson 端的低門檻；機器人端另外有 MinConfidence")
    parser.add_argument("--team", type=int, default=7130)
    parser.add_argument("--server", default=None)
    parser.add_argument("--print-only", action="store_true", help="不連 NT，只印結果")
    parser.add_argument("--stream-port", type=int, default=1181)
    parser.add_argument("--save-dir", default=None, help="（舊）定時存原始畫面的資料夾；建議改用 --record")
    parser.add_argument("--save-every", type=float, default=2.0, help="（舊）幾秒存一張")
    parser.add_argument("--record", action="store_true", help="一開始就開存圖模式（NT /Vision/Record 也能開）")
    parser.add_argument("--record-fps", type=float, default=2.5, help="存圖模式每秒幾張")
    parser.add_argument("--record-dir", default="~/captures", help="存圖根目錄（底下自動分 YYYYMMDD 資料夾）")
    parser.add_argument("--record-quality", type=int, default=90, help="JPEG 品質")
    parser.add_argument("--record-min-free-gb", type=float, default=5.0, help="剩餘空間低於這個值就停止存圖")
    add_lock_args(parser)
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model, task="detect")
    cap = open_camera(args.device, args.width, args.height, args.fps)
    if not cap.isOpened():
        print("❌ 打不開攝影機")
        raise SystemExit(1)  # 非 0 結束碼，systemd 才會記成失敗並重啟
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"攝影機：{width}x{height}")
    print(f"攝影機設定：{lock_camera(cap, args.exposure, args.wb)}")  # 要在 LatestFrame 開始讀之前

    # 重連時直接套用剛鎖好的數值，不再等 1.5 秒自動調整
    fixed_exposure = "auto" if args.exposure == "auto" else str(cap.get(cv2.CAP_PROP_EXPOSURE))
    fixed_wb = "auto" if args.wb == "auto" else str(cap.get(cv2.CAP_PROP_WB_TEMPERATURE))

    def reopen():
        if not is_stream(args.device) and not os.path.exists(args.device) and not str(args.device).isdigit():
            return None
        c = open_camera(args.device, args.width, args.height, args.fps)
        if not c.isOpened():
            c.release()
            return None
        c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        lock_camera(c, fixed_exposure, fixed_wb, settle_s=0)
        return c
    undistort = Undistorter(args.calib, width, height, args.hfov)
    camera = LatestFrame(cap, reopen)
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
            "recording": table.getBooleanTopic("Recording").publish(),
            "recordCount": table.getIntegerTopic("RecordCount").publish(),
            "recordFreeGB": table.getDoubleTopic("RecordFreeGB").publish(),
            "recordStatus": table.getStringTopic("RecordStatus").publish(),
            "recordStopReason": table.getStringTopic("RecordStopReason").publish(),
        }
        pubs["model"].set(os.path.basename(args.model))
        # /Vision/Record：Elastic 或 OutlineViewer 設 true 就開始存圖。publish 預設值讓它出現在 NT 上可以直接改
        record_topic = table.getBooleanTopic("Record")
        record_pub = record_topic.publish(ntcore.PubSubOptions(keepDuplicates=True))
        record_topic.setRetained(True)
        record_pub.setDefault(False)
        record_sub = record_topic.subscribe(False)

    recorder = Recorder(args.record_dir, args.record_fps, args.record_quality, args.record_min_free_gb)
    print(f"存圖：{'開' if args.record else '關（NT /Vision/Record 可開）'}，{args.record_fps} 張/秒 → "
          f"{recorder.root}，剩餘 {recorder.free_gb:.1f} GB")
    last_record_pub = 0.0

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

            record_on = args.record or (pubs is not None and record_sub.get())
            recording = recorder.offer(frame, now, record_on)
            if pubs and now - last_record_pub >= 0.5:
                last_record_pub = now
                pubs["recording"].set(recording)
                pubs["recordCount"].set(recorder.count)
                pubs["recordFreeGB"].set(round(recorder.free_gb, 2))
                pubs["recordStatus"].set(recorder.status_text(recording))
                pubs["recordStopReason"].set(recorder.stop_reason)

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
                rec = f"  存圖 {recorder.count} 張（丟 {recorder.dropped}）" if record_on else ""
                print(f"{fps:5.1f} fps  延遲 {latency_ms:5.1f} ms  大洞 {n_hole}  加分洞亮燈分數 {lits}{rec}")
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()


if __name__ == "__main__":
    main()
