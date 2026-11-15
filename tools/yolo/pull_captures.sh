#!/usr/bin/env bash
# 從 Jetson 拉存圖模式的照片到訓練機（4060Ti Ubuntu）。可以重複跑：rsync 只傳新的檔案。
#
# 用法：
#   ./pull_captures.sh                 # 拉到 ./captures/
#   ./pull_captures.sh ~/data/captures # 指定目的地
# 環境變數：JETSON（預設 zheng@zheng.local）
#
# 順便把 Jetson 上的舊模型 .pt（如果有）拉到 ./models/，給 autolabel.py 用
# （best.engine 只能在建它的那台 Jetson 上用，訓練機不能拿來標註）。
set -euo pipefail

JETSON="${JETSON:-zheng@zheng.local}"
DEST="${1:-./captures}"
mkdir -p "$DEST" ./models

echo "== 拉照片 $JETSON:~/captures/ → $DEST/"
rsync -av --partial --info=progress2 "$JETSON:captures/" "$DEST/"

echo "== 找舊模型 .pt"
# 依序找：models/、jetson_tools/、家目錄 runs/ 底下的 best.pt
PTS=$(ssh "$JETSON" 'ls -1 ~/tel2026/models/*.pt ~/tel2026/jetson_tools/*.pt ~/tel2026/jetson_tools/runs/*/weights/best.pt ~/runs/*/weights/best.pt 2>/dev/null' || true)
if [ -n "$PTS" ]; then
  echo "$PTS"
  for f in $PTS; do
    rsync -a "$JETSON:$f" "./models/jetson_$(basename "$(dirname "$f")")_$(basename "$f")"
  done
  ls -l ./models
else
  echo "⚠️ Jetson 上找不到 .pt。看看訓練機的 runs/*/weights/best.pt（train.py 的輸出），找不到就用 yolo11n.pt 從頭標。"
fi

N=$(find "$DEST" -name '*.jpg' | wc -l)
echo "== 完成：$DEST 共 $N 張"
du -sh "$DEST"
ssh "$JETSON" 'df -h ~ | tail -1' || true
