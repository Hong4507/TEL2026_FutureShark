#!/usr/bin/env python3
"""用官方九宮格設計圖產生合成訓練資料（自動附 YOLO 標註）。在 PC 上跑即可，不需要攝影機。

原理：把設計圖當成一塊 3 m 寬的平面，用隨機的攝影機位置、角度、視角投影到畫面上，
再加上隨機背景、洞內填充（網子／背景／暗色）、紅色 LED 燈圈、遮擋、模糊、雜訊、亮度變化。
洞的位置由程式從設計圖自動找出來，所以標註是精確的。

類別（要和 robot/subsystems/vision/VisionConstants.java 一致）：
  0 hole   直徑 40 cm 的大洞
  1 bonus  直徑 20 cm 的加分洞（亮不亮燈都是 bonus；亮燈由 vision_nt_publisher.py 另外判斷）

用法：
  python synth_dataset.py --out datasets/synth --count 4000
  python synth_dataset.py --out datasets/synth --count 4000 --bg-dir backgrounds/   # 加入真實背景照片
  python synth_dataset.py --preview 12 --out preview/                               # 只畫幾張框起來的圖檢查

--bg-dir 建議放場地附近、不含九宮格的照片或影片截圖（extract_frames.py 抽出來的也可以），
沒有的話會用程式產生的背景。
"""
import argparse
import glob
import math
import os
import random

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DESIGN = os.path.join(HERE, "assets", "nine_grid_design.jpg")

CLASS_NAMES = ["hole", "bonus"]
HOLE_DIAMETER_M = 0.40
BONUS_DIAMETER_M = 0.20
LOW_ROW_HEIGHT_M = 1.20  # 最下排大洞中心離地高度（FieldConstants.kLowTargetHeight）
LED_RING_RADIUS_M = 0.10 + 0.05 + 0.007  # 洞半徑 + 5 cm 間隔 + 燈條半寬（規則 p.14）
LED_RING_WIDTH_M = 0.014


# ---------------------------------------------------------------- 設計圖

def find_holes(design):
    """找出設計圖上的白色圓洞，回傳 [(x, y, r, class_id)]（設計圖像素）。"""
    hsv = cv2.cvtColor(design, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 200), (180, 60, 255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    circles = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 300:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        if area / (math.pi * r * r) < 0.9:  # 排除 AI 字樣等非圓形
            continue
        circles.append((x, y, r))
    big = max(r for _, _, r in circles)
    holes = []
    for x, y, r in circles:
        if abs(r - big) / big < 0.1:
            holes.append((x, y, r, 0))
        elif abs(r - big / 2) / (big / 2) < 0.15:
            holes.append((x, y, r, 1))
    n_hole = sum(1 for h in holes if h[3] == 0)
    n_bonus = sum(1 for h in holes if h[3] == 1)
    if (n_hole, n_bonus) != (9, 3):
        raise RuntimeError(f"設計圖上找到 {n_hole} 個大洞、{n_bonus} 個加分洞，預期 9 和 3")
    return holes


class Board:
    def __init__(self, design_path):
        self.image = cv2.imread(design_path)
        if self.image is None:
            raise FileNotFoundError(design_path)
        self.holes = find_holes(self.image)
        big_r = np.mean([h[2] for h in self.holes if h[3] == 0])
        self.m_per_px = HOLE_DIAMETER_M / (2 * big_r)
        h, w = self.image.shape[:2]
        self.h, self.w = h, w
        low_row_y = max(h_[1] for h_ in self.holes if h_[3] == 0)
        # 板子底邊離地高度，讓最下排中心在 1.2 m
        self.bottom_m = LOW_ROW_HEIGHT_M - (h - low_row_y) * self.m_per_px

    def to_board_m(self, px, py):
        """設計圖像素 → 板面座標（公尺，u 往右、v 離地高度）。"""
        return (px - self.w / 2) * self.m_per_px, (self.h - py) * self.m_per_px + self.bottom_m


# ---------------------------------------------------------------- 3D 投影

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Camera:
    """世界座標：X 沿牆往右、Y 往上、Z 朝牆。板子底邊在 Z = distance。"""

    def __init__(self, width, height, rng):
        self.width, self.height = width, height
        hfov = math.radians(rng.uniform(55, 95))
        self.f = width / 2 / math.tan(hfov / 2)
        self.distance = rng.uniform(1.8, 4.5)        # 擋板到網框約 2.5 m，前後留範圍
        self.pos = np.array([rng.uniform(-1.6, 1.6), rng.uniform(0.25, 0.9), 0.0])
        self.tilt = math.radians(rng.uniform(80, 90))  # 網框安裝角度約 85°
        # 大多數時候正對牆，偶爾斜一點；對準點在板子附近
        aim_x = rng.uniform(-1.2, 1.2)
        aim_y = rng.uniform(1.2, 2.6)
        yaw = math.atan2(aim_x - self.pos[0], self.distance) + math.radians(rng.gauss(0, 4))
        pitch = math.atan2(aim_y - self.pos[1], self.distance)
        roll = math.radians(rng.gauss(0, 1.5))
        # 世界 → 相機：先轉 yaw，再 pitch，再 roll
        self.R = rot_z(roll) @ rot_x(pitch) @ rot_y(-yaw)

    def board_point(self, u, v):
        """板面座標（u 往右、v 離地高度）→ 世界座標，板子往後仰 (90° - tilt)。"""
        return np.array([u, v * math.sin(self.tilt), self.distance + v * math.cos(self.tilt)])

    def project(self, pts):
        """世界座標 (N,3) → 像素 (N,2) 與深度 (N,)。"""
        pc = (self.R @ (pts - self.pos).T).T
        z = pc[:, 2]
        x = self.f * pc[:, 0] / np.maximum(z, 1e-6) + self.width / 2
        y = -self.f * pc[:, 1] / np.maximum(z, 1e-6) + self.height / 2
        return np.stack([x, y], axis=1), z


# ---------------------------------------------------------------- 外觀

def random_background(width, height, rng, bg_files):
    if bg_files and rng.random() < 0.8:
        img = cv2.imread(rng.choice(bg_files))
        if img is not None:
            scale = max(width / img.shape[1], height / img.shape[0]) * rng.uniform(1.0, 1.6)
            img = cv2.resize(img, None, fx=scale, fy=scale)
            x0 = rng.randint(0, img.shape[1] - width)
            y0 = rng.randint(0, img.shape[0] - height)
            img = img[y0:y0 + height, x0:x0 + width]
            if rng.random() < 0.5:
                img = cv2.flip(img, 1)
            return img
    # 程式產生：漸層 + 雜訊 + 隨機方塊與線條（模擬場館、桁架、燈）
    base = np.array([rng.randint(10, 120) for _ in range(3)], np.float32)
    grad = np.linspace(rng.uniform(0.4, 1.0), rng.uniform(0.4, 1.0), height, dtype=np.float32)[:, None, None]
    img = np.clip(base * grad * np.ones((height, width, 3), np.float32), 0, 255).astype(np.uint8)
    for _ in range(rng.randint(3, 15)):
        color = [rng.randint(0, 255) for _ in range(3)]
        x, y = rng.randint(0, width), rng.randint(0, height)
        if rng.random() < 0.5:
            cv2.rectangle(img, (x, y), (x + rng.randint(10, 400), y + rng.randint(10, 300)), color, -1)
        else:
            cv2.line(img, (x, y), (rng.randint(0, width), rng.randint(0, height)), color, rng.randint(2, 25))
    for _ in range(rng.randint(0, 4)):  # 白色亮點（燈），容易被誤認成洞
        cv2.circle(img, (rng.randint(0, width), rng.randint(0, height)), rng.randint(5, 60), (255, 255, 255), -1)
    noise = np.random.normal(0, rng.uniform(2, 12), img.shape).astype(np.int16)
    return np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def net_texture(size, rng):
    tex = np.full((size, size, 3), rng.randint(5, 70), np.uint8)
    step = rng.randint(6, 18)
    color = (rng.randint(120, 255),) * 3
    for i in range(0, size, step):
        cv2.line(tex, (i, 0), (i, size), color, 1)
        cv2.line(tex, (0, i), (size, i), color, 1)
    m = cv2.getRotationMatrix2D((size / 2, size / 2), rng.uniform(0, 90), 1.0)
    return cv2.warpAffine(tex, m, (size, size), borderMode=cv2.BORDER_REFLECT)


def board_texture(board, rng, bg_files):
    """板面貼圖：設計圖，洞內換成網子／背景／暗色／原本的白色。"""
    tex = board.image.copy()
    for x, y, r, _ in board.holes:
        choice = rng.random()
        size = int(2 * r) + 4
        if choice < 0.35:
            fill = net_texture(size, rng)
        elif choice < 0.65:
            fill = random_background(size, size, rng, bg_files)
        elif choice < 0.85:
            fill = np.full((size, size, 3), rng.randint(0, 90), np.uint8)
        else:
            continue  # 保留白色
        mask = np.zeros((size, size), np.uint8)
        cv2.circle(mask, (size // 2, size // 2), int(r), 255, -1)
        x0, y0 = int(x) - size // 2, int(y) - size // 2
        roi = tex[y0:y0 + size, x0:x0 + size]
        roi[mask > 0] = fill[mask > 0]
    return tex


def led_layer(board, rng):
    """隨機一個加分洞的紅色 LED 燈圈（跟板子用同一個投影）。"""
    layer = np.zeros_like(board.image)
    bonus = [h for h in board.holes if h[3] == 1]
    x, y, _, _ = rng.choice(bonus)
    r = LED_RING_RADIUS_M / board.m_per_px
    t = max(1, int(LED_RING_WIDTH_M / board.m_per_px))
    cv2.circle(layer, (int(x), int(y)), int(r), (40, 40, 255), t)
    return layer


def photometric(img, rng):
    img = img.astype(np.float32)
    img = img * rng.uniform(0.5, 1.4) + rng.uniform(-30, 30)
    img *= np.array([rng.uniform(0.85, 1.15) for _ in range(3)], np.float32)  # 色溫
    img = np.clip(img, 0, 255).astype(np.uint8)
    if rng.random() < 0.3:
        gamma = rng.uniform(0.6, 1.6)
        img = (255 * (img / 255.0) ** gamma).astype(np.uint8)
    if rng.random() < 0.4:
        k = rng.choice([3, 5, 7])
        img = cv2.GaussianBlur(img, (k, k), 0)
    if rng.random() < 0.25:  # 動態模糊（機器人移動）
        k = rng.randint(5, 15)
        kernel = np.zeros((k, k), np.float32)
        kernel[k // 2, :] = 1.0 / k
        img = cv2.filter2D(img, -1, kernel)
    noise = np.random.normal(0, rng.uniform(1, 8), img.shape).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:
        q = rng.randint(40, 90)
        img = cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])[1], cv2.IMREAD_COLOR)
    return img


# ---------------------------------------------------------------- 產生一張

def render(board, width, height, rng, bg_files):
    cam = Camera(width, height, rng)
    corners_px = np.array([[0, 0], [board.w, 0], [board.w, board.h], [0, board.h]], np.float32)
    corners_w = np.array([cam.board_point(*board.to_board_m(x, y)) for x, y in corners_px])
    proj, depth = cam.project(corners_w)
    if np.any(depth < 0.3):
        return None, None
    H = cv2.getPerspectiveTransform(corners_px, proj.astype(np.float32))

    img = random_background(width, height, rng, bg_files)
    tex = board_texture(board, rng, bg_files)
    warped = cv2.warpPerspective(tex, H, (width, height))
    mask = cv2.warpPerspective(np.full(tex.shape[:2], 255, np.uint8), H, (width, height))
    img[mask > 0] = warped[mask > 0]

    if rng.random() < 0.4:
        led = cv2.warpPerspective(led_layer(board, rng), H, (width, height))
        glow = cv2.GaussianBlur(led, (0, 0), rng.uniform(2, 6))
        img = cv2.add(img, cv2.add(led, (glow * rng.uniform(1.0, 2.5)).clip(0, 255).astype(np.uint8)))

    labels = []
    angles = np.linspace(0, 2 * math.pi, 32, endpoint=False)
    for x, y, r, cls in board.holes:
        pts = np.stack([x + r * np.cos(angles), y + r * np.sin(angles)], axis=1)
        world = np.array([cam.board_point(*board.to_board_m(px, py)) for px, py in pts])
        p, _ = cam.project(world)
        x0, y0 = p.min(axis=0)
        x1, y1 = p.max(axis=0)
        full = (x1 - x0) * (y1 - y0)
        cx0, cy0, cx1, cy1 = max(x0, 0), max(y0, 0), min(x1, width), min(y1, height)
        if cx1 <= cx0 or cy1 <= cy0 or full <= 0:
            continue
        if (cx1 - cx0) * (cy1 - cy0) / full < 0.6 or min(cx1 - cx0, cy1 - cy0) < 6:
            continue  # 大部分在畫面外或太小
        labels.append([cls, cx0, cy0, cx1, cy1])

    # 遮擋：隨機色塊（其他機器人、人、桁架）。被遮住一半以上的洞就不標
    for _ in range(rng.choice([0, 0, 1, 1, 2, 3])):
        w_, h_ = rng.randint(width // 12, width // 3), rng.randint(height // 8, height // 2)
        ox, oy = rng.randint(-w_ // 2, width), rng.randint(height // 3, height)
        cv2.rectangle(img, (ox, oy), (ox + w_, oy + h_), [rng.randint(0, 255) for _ in range(3)], -1)
        kept = []
        for lab in labels:
            ix = max(0, min(lab[3], ox + w_) - max(lab[1], ox))
            iy = max(0, min(lab[4], oy + h_) - max(lab[2], oy))
            if ix * iy / ((lab[3] - lab[1]) * (lab[4] - lab[2])) < 0.5:
                kept.append(lab)
        labels = kept

    return photometric(img, rng), labels


def to_yolo(labels, width, height):
    lines = []
    for cls, x0, y0, x1, y1 in labels:
        lines.append(f"{cls} {(x0 + x1) / 2 / width:.6f} {(y0 + y1) / 2 / height:.6f} "
                     f"{(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}")
    return "\n".join(lines) + ("\n" if lines else "")


def write_data_yaml(out_dir):
    with open(os.path.join(out_dir, "data.yaml"), "w", encoding="utf-8") as f:
        f.write(f"path: {os.path.abspath(out_dir)}\ntrain: images/train\nval: images/val\n")
        f.write("names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASS_NAMES)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--design", default=DEFAULT_DESIGN)
    parser.add_argument("--out", default="datasets/synth")
    parser.add_argument("--count", type=int, default=4000)
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--width", type=int, default=1280, help="請和比賽用的攝影機解析度相同")
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--bg-dir", default=None, help="背景照片資料夾（可選）")
    parser.add_argument("--negatives", type=float, default=0.05, help="完全沒有九宮格的背景圖比例")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--preview", type=int, default=0, help="只輸出 N 張畫好框的預覽圖")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)
    board = Board(args.design)
    bg_files = []
    if args.bg_dir:
        for ext in ("jpg", "jpeg", "png"):
            bg_files += glob.glob(os.path.join(args.bg_dir, "**", f"*.{ext}"), recursive=True)
    print(f"設計圖：{len(board.holes)} 個洞，{board.m_per_px * 100:.3f} cm/px，背景圖 {len(bg_files)} 張")

    if args.preview:
        os.makedirs(args.out, exist_ok=True)
        n = 0
        while n < args.preview:
            img, labels = render(board, args.width, args.height, rng, bg_files)
            if img is None:
                continue
            for cls, x0, y0, x1, y1 in labels:
                color = (0, 255, 0) if cls == 0 else (0, 200, 255)
                cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), color, 2)
            cv2.imwrite(os.path.join(args.out, f"preview_{n:02d}.jpg"), img)
            n += 1
        print(f"預覽圖存在 {args.out}")
        return

    for split in ("train", "val"):
        os.makedirs(os.path.join(args.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(args.out, "labels", split), exist_ok=True)
    n = 0
    while n < args.count:
        if rng.random() < args.negatives:
            img, labels = random_background(args.width, args.height, rng, bg_files), []
            img = photometric(img, rng)
        else:
            img, labels = render(board, args.width, args.height, rng, bg_files)
            if img is None:
                continue
        split = "val" if rng.random() < args.val_ratio else "train"
        name = f"synth_{n:05d}"
        cv2.imwrite(os.path.join(args.out, "images", split, name + ".jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        with open(os.path.join(args.out, "labels", split, name + ".txt"), "w") as f:
            f.write(to_yolo(labels, args.width, args.height))
        n += 1
        if n % 500 == 0:
            print(f"{n}/{args.count}")
    write_data_yaml(args.out)
    print(f"完成：{args.out}（data.yaml 已產生）")


if __name__ == "__main__":
    main()
