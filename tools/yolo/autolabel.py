#!/usr/bin/env python3
"""用舊模型自動標註存圖模式的照片 → YOLO txt（class cx cy w h，0~1）。

- 每張圖都會有同名 .txt；**標不到東西的圖寫空檔**（YOLO 當負樣本），一律保留，人工補標時再看。
- 空檔的圖另外列在 <images>/autolabel_empty.txt，人工修正時優先看這些。
- 已經有 .txt 的圖預設跳過（避免蓋掉人工修正過的），--overwrite 才重標。
- 只能用 .pt / .onnx（best.engine 只能在建它的那台 Jetson 上用）。

用法：
  python autolabel.py --model models/jetson_models_best.pt --images captures
  python autolabel.py --model runs/v1/weights/best.pt --images captures --conf 0.25 --device 0
"""
import argparse
import os
import sys

CLASS_NAMES = ["hole", "bonus"]
IMAGE_EXTS = (".jpg", ".jpeg", ".png")


def find_images(root):
    out = []
    for dirpath, _, files in os.walk(root):
        out += [os.path.join(dirpath, f) for f in files if f.lower().endswith(IMAGE_EXTS)]
    return sorted(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True, help="舊模型 .pt（或 .onnx）")
    parser.add_argument("--images", required=True, help="照片資料夾（會往下找子資料夾，例如 captures/20261115/）")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default=None, help="0 = 第一張 GPU；不給就自動")
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if args.model.endswith(".engine"):
        sys.exit("❌ best.engine 只能在建它的 Jetson 上用。請用 .pt（pull_captures.sh 會試著從 Jetson 拉）。")

    from ultralytics import YOLO

    model = YOLO(args.model, task="detect")
    names = getattr(model, "names", None)
    if names and len(names) >= 2 and [names[i] for i in range(2)] != CLASS_NAMES:
        print(f"⚠️ 模型類別 {names} 和 {CLASS_NAMES} 不同，請確認類別編號一致")

    images = find_images(args.images)
    todo = [p for p in images if args.overwrite or not os.path.exists(os.path.splitext(p)[0] + ".txt")]
    print(f"共 {len(images)} 張，要標 {len(todo)} 張（已有標註 {len(images) - len(todo)} 張跳過）")

    boxes = new_empty = 0
    for i in range(0, len(todo), args.batch):
        chunk = todo[i:i + args.batch]
        results = model.predict(chunk, imgsz=args.imgsz, conf=args.conf, device=args.device, verbose=False)
        for path, r in zip(chunk, results):
            lines = [f"{int(c)} " + " ".join(f"{v:.6f}" for v in xywhn) + "\n"
                     for c, xywhn in zip(r.boxes.cls.tolist(), r.boxes.xywhn.tolist())]
            with open(os.path.splitext(path)[0] + ".txt", "w") as f:
                f.writelines(lines)
            boxes += len(lines)
            new_empty += not lines
        print(f"  {min(i + args.batch, len(todo))}/{len(todo)}", end="\r")

    # 每個有圖的資料夾放一份 classes.txt（X-AnyLabeling／labelImg 讀 YOLO 格式要用）
    for folder in sorted({os.path.dirname(p) for p in images}):
        with open(os.path.join(folder, "classes.txt"), "w") as f:
            f.write("\n".join(CLASS_NAMES) + "\n")
    # 空標註清單：重新掃全部（包含之前跑過、人工改過的），人工補標時優先看
    empty = []
    for p in images:
        with open(os.path.splitext(p)[0] + ".txt") as f:
            if not f.read().strip():
                empty.append(p)
    with open(os.path.join(args.images, "autolabel_empty.txt"), "w") as f:
        f.write("\n".join(empty) + ("\n" if empty else ""))
    print(f"\n這次標了 {len(todo)} 張、{boxes} 個框（{new_empty} 張沒標到）；"
          f"全部 {len(images)} 張裡空標註 {len(empty)} 張 → autolabel_empty.txt（保留當負樣本，人工補標時先看）")


if __name__ == "__main__":
    main()
