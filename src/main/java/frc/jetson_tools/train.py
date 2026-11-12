#!/usr/bin/env python3
"""訓練九宮格 YOLO11 模型（在有 GPU 的電腦上跑：4060Ti Ubuntu 優先）。

會把「合成資料」和「真實標註資料」合在一起：
- 合成資料：synth_dataset.py 的輸出資料夾（含 images/train、images/val）
- 真實資料：一個資料夾，裡面是圖片和同名 .txt（autolabel.py + 人工修正後的結果）
  依影片名（extract_frames.py 的「影片名_f幀號」）或擷取時間分組切驗證集，同一段影片不會兩邊都有。
  真實資料比合成資料珍貴，訓練時預設重複 3 次（--real-repeat）。
- 有真實資料時，驗證集只用真實資料（分數才有參考價值）。

用法：
  python train.py --synth datasets/synth --name v0                         # 只有合成資料
  python train.py --synth datasets/synth --real labeled/ --name v1 --weights runs/v0/weights/best.pt
完成後把 runs/<name>/weights/best.pt 複製到 Jetson，在 Jetson 上轉 TensorRT：
  yolo export model=best.pt format=engine half=True imgsz=640
"""
import argparse
import glob
import os
import random
import re
import shutil

CLASS_NAMES = ["hole", "bonus"]
IMAGE_EXTS = ("jpg", "jpeg", "png")


def images_in(folder):
    return sorted(sum((glob.glob(os.path.join(folder, f"*.{e}")) for e in IMAGE_EXTS), []))


def group_key(path):
    name = os.path.splitext(os.path.basename(path))[0]
    m = re.match(r"(.+)_f\d+$", name)
    if m:
        return m.group(1)
    m = re.match(r"(cap_\d{8}_\d{4})", name)  # vision_nt_publisher --save-dir：同一分鐘一組
    if m:
        return m.group(1)
    return name


def prepare_real(real_dir, work_dir, val_ratio, seed):
    """把真實資料整理成 YOLO 需要的 images/ 與 labels/ 結構，回傳 (train 圖片, val 圖片)。"""
    img_out = os.path.join(work_dir, "real", "images")
    lbl_out = os.path.join(work_dir, "real", "labels")
    os.makedirs(img_out, exist_ok=True)
    os.makedirs(lbl_out, exist_ok=True)
    groups = {}
    missing = 0
    for path in images_in(real_dir):
        txt = os.path.splitext(path)[0] + ".txt"
        if not os.path.exists(txt):
            missing += 1
            continue
        dst = os.path.join(img_out, os.path.basename(path))
        shutil.copy2(path, dst)
        shutil.copy2(txt, os.path.join(lbl_out, os.path.basename(txt)))
        groups.setdefault(group_key(path), []).append(dst)
    if missing:
        print(f"⚠️ {missing} 張真實圖片沒有標註檔，略過")
    keys = sorted(groups)
    random.Random(seed).shuffle(keys)
    n_val = max(1, round(len(keys) * val_ratio)) if len(keys) > 1 else 0
    val = [p for k in keys[:n_val] for p in groups[k]]
    train = [p for k in keys[n_val:] for p in groups[k]]
    print(f"真實資料：{len(keys)} 組，train {len(train)} 張、val {len(val)} 張（驗證組：{keys[:n_val]}）")
    return train, val


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--synth", default=None, help="synth_dataset.py 的輸出資料夾")
    parser.add_argument("--real", default=None, help="真實標註資料夾（圖片 + 同名 .txt）")
    parser.add_argument("--real-repeat", type=int, default=3)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--weights", default="yolo11n.pt", help="起始權重：yolo11n.pt，或上一版的 best.pt 接著練")
    parser.add_argument("--name", default="v0")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16, help="4060Ti 可試 32；1060 用 8")
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8, help="Windows 若卡住改 0")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if not args.synth and not args.real:
        parser.error("至少要給 --synth 或 --real")

    work = os.path.abspath(os.path.join("runs", args.name + "_data"))
    os.makedirs(work, exist_ok=True)
    train, val = [], []
    if args.synth:
        train += images_in(os.path.join(args.synth, "images", "train"))
        synth_val = images_in(os.path.join(args.synth, "images", "val"))
        print(f"合成資料：train {len(train)} 張、val {len(synth_val)} 張")
    if args.real:
        real_train, real_val = prepare_real(args.real, work, args.val_ratio, args.seed)
        train += real_train * args.real_repeat
        val = real_val
    if not val:
        val = synth_val if args.synth else []
        print("⚠️ 沒有真實驗證資料，用合成驗證集（分數只能參考）")

    for name, items in (("train.txt", train), ("val.txt", val)):
        with open(os.path.join(work, name), "w", encoding="utf-8") as f:
            f.write("\n".join(os.path.abspath(p) for p in items) + "\n")
    data_yaml = os.path.join(work, "data.yaml")
    with open(data_yaml, "w", encoding="utf-8") as f:
        f.write(f"path: {work}\ntrain: train.txt\nval: val.txt\n")
        f.write("names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASS_NAMES)))

    from ultralytics import YOLO

    model = YOLO(args.weights)
    model.train(
        data=data_yaml,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=os.path.abspath("runs"),
        name=args.name,
        exist_ok=True,
        seed=args.seed,
        patience=25,
        hsv_h=0.01,      # 色相別動太多（加分洞的紅色燈圈要保持紅色）
        degrees=3.0,
        perspective=0.0005,
        fliplr=0.5,
        close_mosaic=10,
    )
    best = os.path.join("runs", args.name, "weights", "best.pt")
    print(f"\n完成：{best}")
    print("複製到 Jetson 後轉 TensorRT：yolo export model=best.pt format=engine half=True imgsz=" + str(args.imgsz))


if __name__ == "__main__":
    main()
