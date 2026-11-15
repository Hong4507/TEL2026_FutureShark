#!/usr/bin/env bash
# 一行開始訓練（4060Ti Ubuntu）。
#
# 用法：
#   ./train.sh datasets/real1115/data.yaml                       # 從 yolo11n.pt 開始
#   ./train.sh datasets/real1115/data.yaml models/old_best.pt    # 接續舊權重微調（建議，和現有模型同大小）
#   EPOCHS=1 DEVICE=cpu ./train.sh datasets/test/data.yaml       # 測流程
# 環境變數：EPOCHS(100) PATIENCE(20) IMGSZ(640) BATCH(16；4060Ti 16GB 可 32) DEVICE(0) NAME(real_YYYYMMDD_HHMM)
# 輸出：runs/<NAME>/weights/best.pt → 接著跑 ./export_and_deploy.sh runs/<NAME>/weights/best.pt
set -euo pipefail

DATA="${1:?用法：./train.sh <data.yaml> [起始權重 .pt]}"
WEIGHTS="${2:-yolo11n.pt}"
NAME="${NAME:-real_$(date +%Y%m%d_%H%M)}"

yolo detect train \
  data="$DATA" model="$WEIGHTS" \
  imgsz="${IMGSZ:-640}" epochs="${EPOCHS:-100}" patience="${PATIENCE:-20}" \
  batch="${BATCH:-16}" device="${DEVICE:-0}" workers="${WORKERS:-8}" \
  project="$(pwd)/runs" name="$NAME" exist_ok=True \
  hsv_v=0.6 hsv_s=0.7 hsv_h=0.01 \
  degrees=3.0 perspective=0.0005 fliplr=0.5 close_mosaic=10

echo
echo "完成：runs/$NAME/weights/best.pt"
echo "部署：./export_and_deploy.sh runs/$NAME/weights/best.pt"
