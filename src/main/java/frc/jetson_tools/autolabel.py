#!/usr/bin/env python3
"""用現有模型（例如合成資料練的 v0）先自動標註，人只要修正，節省標註時間。

輸出 YOLO 格式：每張圖一個同名 .txt（class cx cy w h，0~1），以及 classes.txt。
X-AnyLabeling、CVAT（YOLO 1.1）、Roboflow 都能直接匯入修改。

用法：
  python autolabel.py --model runs/v0/weights/best.pt --images frames --conf 0.4
  （預設把 .txt 寫在圖片旁邊；--labels 指定別的資料夾）
"""
import argparse
import glob
import os

CLASS_NAMES = ["hole", "bonus"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", default=None, help="標註輸出資料夾，預設和圖片同一個")
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--overwrite", action="store_true", help="覆蓋已經存在的標註（預設跳過，避免蓋掉人工修正過的）")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model)
    out_dir = args.labels or args.images
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "classes.txt"), "w") as f:
        f.write("\n".join(CLASS_NAMES) + "\n")

    files = sorted(sum((glob.glob(os.path.join(args.images, f"*.{e}")) for e in ("jpg", "jpeg", "png")), []))
    done = skipped = 0
    for path in files:
        txt = os.path.join(out_dir, os.path.splitext(os.path.basename(path))[0] + ".txt")
        if os.path.exists(txt) and not args.overwrite:
            skipped += 1
            continue
        r = model.predict(path, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        with open(txt, "w") as f:
            for cls, xywhn in zip(r.boxes.cls.tolist(), r.boxes.xywhn.tolist()):
                f.write(f"{int(cls)} " + " ".join(f"{v:.6f}" for v in xywhn) + "\n")
        done += 1
    print(f"自動標註 {done} 張，跳過已有標註 {skipped} 張 → {out_dir}")


if __name__ == "__main__":
    main()
