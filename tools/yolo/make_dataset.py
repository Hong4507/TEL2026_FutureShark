#!/usr/bin/env python3
"""把標好的照片整理成 YOLO 資料集，**依錄影時段**切 train／val。

為什麼不隨機切：存圖模式每秒 2.5 張，相鄰畫面幾乎一樣，隨機切會讓 val 裡的圖在 train 都有「雙胞胎」，
val 分數虛高。這裡把「同一天、相鄰兩張間隔 < --gap 秒」的照片算同一個時段，
每個時段再切成 --block 秒一塊，**每個時段各挑約 20% 的塊**當 val（整塊只會在 train 或 val）。
這樣近／中／遠／高曝光每一段在 train 和 val 都有，不會整段被拿去當 val、train 看不到。

檔名格式：captures/YYYYMMDD/HHMMSS_mmm.jpg（vision_nt_publisher.py --record）。
其他檔名（舊的 cap_YYYYMMDD_HHMMSS_mmm.jpg）也看得懂；完全看不懂的整個資料夾算一個時段。

用法：
  python make_dataset.py --images captures --out datasets/real1115
  python make_dataset.py --images captures --out datasets/real1115 --val-ratio 0.2 --gap 30 --block 20
輸出：<out>/images/{train,val}、<out>/labels/{train,val}（symlink，--copy 改複製）、<out>/data.yaml、<out>/split.txt
"""
import argparse
import os
import re
import shutil
from datetime import datetime

CLASS_NAMES = ["hole", "bonus"]
IMAGE_EXTS = (".jpg", ".jpeg", ".png")


def timestamp(path):
    """回傳 datetime；看不懂檔名就 None。"""
    name = os.path.splitext(os.path.basename(path))[0]
    day = os.path.basename(os.path.dirname(path))
    m = re.fullmatch(r"(\d{6})_(\d{3})", name)
    if m and re.fullmatch(r"\d{8}", day):
        return datetime.strptime(day + m.group(1) + m.group(2), "%Y%m%d%H%M%S%f").replace(
            microsecond=int(m.group(2)) * 1000)
    m = re.search(r"(\d{8})_(\d{6})_?(\d{3})?", name)
    if m:
        t = datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
        return t.replace(microsecond=int(m.group(3) or 0) * 1000)
    return None


def sessions(images, gap_s):
    """依時間把照片分成時段，回傳 [[path, ...], ...]（時間順序）。"""
    dated = sorted((t, p) for p in images if (t := timestamp(p)) is not None)
    out, last = [], None
    for t, p in dated:
        if last is None or (t - last).total_seconds() > gap_s:
            out.append([])
        out[-1].append(p)
        last = t
    undated = {}
    for p in images:
        if timestamp(p) is None:
            undated.setdefault(os.path.dirname(p), []).append(p)
    return out + [sorted(v) for v in undated.values()]


def blocks(group, block_s):
    """把一個時段切成每 block_s 秒一塊。"""
    out, start = [], None
    for p in group:
        t = timestamp(p)
        if t is None:  # 看不懂檔名的時段不切
            return [group]
        if start is None or (t - start).total_seconds() >= block_s:
            out.append([])
            start = t
        out[-1].append(p)
    return out


def pick_val(groups, ratio):
    """挑哪些時段當 val：平均分散在整天（早上、中午、下午的光線都有），直到張數 ≥ ratio。"""
    total = sum(len(g) for g in groups)
    if len(groups) < 2 or ratio <= 0:
        return set()
    target = total * ratio
    n = len(groups)
    order = []
    step = max(1, round(1 / ratio))
    for start in range(step - 1, -1, -1):  # 先挑 step-1, 2*step-1, ...，不夠再往前補
        order += list(range(start, n, step))
    val, count = set(), 0
    for i in order:
        if count >= target or len(val) >= n - 1:  # train 至少留一個時段
            break
        val.add(i)
        count += len(groups[i])
    return val


def place(src, dst, copy):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    if os.path.lexists(dst):
        os.remove(dst)
    if copy:
        shutil.copy2(src, dst)
    else:
        os.symlink(os.path.abspath(src), dst)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True, help="照片＋同名 .txt 的資料夾（會往下找）")
    parser.add_argument("--out", required=True)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--gap", type=float, default=60.0, help="相鄰兩張間隔超過幾秒就算新的時段")
    parser.add_argument("--block", type=float, default=20.0, help="每個時段切成幾秒一塊，每段各挑約 val-ratio 的塊當 val")
    parser.add_argument("--copy", action="store_true", help="複製檔案（預設 symlink）")
    args = parser.parse_args()

    images = []
    no_label = 0
    for dirpath, _, files in os.walk(args.images):
        for f in files:
            if f.lower().endswith(IMAGE_EXTS):
                p = os.path.join(dirpath, f)
                if os.path.exists(os.path.splitext(p)[0] + ".txt"):
                    images.append(p)
                else:
                    no_label += 1
    if no_label:
        print(f"⚠️ {no_label} 張沒有 .txt（還沒跑 autolabel.py？），略過")
    if not images:
        raise SystemExit("❌ 沒有可用的圖片")

    sess = sessions(images, args.gap)
    groups, val_idx = [], set()
    for g in sess:
        bs = blocks(g, args.block)
        val_idx |= {len(groups) + i for i in pick_val(bs, args.val_ratio)}
        groups += bs
    if os.path.isdir(args.out):
        for sub in ("images", "labels"):
            shutil.rmtree(os.path.join(args.out, sub), ignore_errors=True)

    counts = {"train": [0, 0, 0], "val": [0, 0, 0]}  # 張數、框數、負樣本
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "split.txt"), "w") as log:
        for i, g in enumerate(groups):
            split = "val" if i in val_idx else "train"
            t0, t1 = timestamp(g[0]), timestamp(g[-1])
            span = f"{t0:%m/%d %H:%M:%S}–{t1:%H:%M:%S}" if t0 else os.path.dirname(g[0])
            log.write(f"{split}\t{len(g)}\t{span}\n")
            for p in g:
                day = os.path.basename(os.path.dirname(p))
                stem = f"{day}_{os.path.splitext(os.path.basename(p))[0]}"
                ext = os.path.splitext(p)[1].lower()
                txt = os.path.splitext(p)[0] + ".txt"
                place(p, os.path.join(args.out, "images", split, stem + ext), args.copy)
                place(txt, os.path.join(args.out, "labels", split, stem + ".txt"), args.copy)
                with open(txt) as f:
                    n = sum(1 for line in f if line.strip())
                c = counts[split]
                c[0] += 1
                c[1] += n
                c[2] += n == 0

    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write(f"path: {os.path.abspath(args.out)}\ntrain: images/train\nval: images/val\n")
        f.write("names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASS_NAMES)))

    print(f"時段 {len(sess)} 個，切成 {len(groups)} 塊（val {len(val_idx)} 塊），明細見 {args.out}/split.txt")
    for g in sess:
        t0 = timestamp(g[0])
        print(f"  時段 {t0:%H:%M:%S} {len(g):5d} 張" if t0 else f"  {os.path.dirname(g[0])} {len(g):5d} 張")
    for split, (n, b, neg) in counts.items():
        print(f"  {split:5s} {n:5d} 張  {b:6d} 框  負樣本 {neg}")
    if not val_idx:
        print("⚠️ 只有 1 個時段，val 是空的。訓練前至少要分兩段錄（中間停 1 分鐘以上），否則 YOLO 會報錯")
    print(f"→ {os.path.abspath(os.path.join(args.out, 'data.yaml'))}")


if __name__ == "__main__":
    main()
