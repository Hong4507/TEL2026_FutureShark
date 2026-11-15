#!/usr/bin/env bash
# best.pt → ONNX（訓練機）→ 傳到 Jetson → 在 Jetson 上用 TensorRT 轉 best.engine → 換模型、重啟 vision 服務。
#
# 用法：
#   ./export_and_deploy.sh runs/real_xxx/weights/best.pt
#   DRY_RUN=1 ./export_and_deploy.sh best.pt    # 只匯出 ONNX，不連 Jetson（測流程）
# 環境變數：JETSON（zheng@zheng.local）IMGSZ（640，要和 vision_nt_publisher --imgsz 一樣）
#
# 安全：新 engine 先在 Jetson 上試跑一張圖，成功才換；舊的留成 best.engine.bak-<時間>。
# 換回舊模型：ssh 進 Jetson，cp ~/tel2026/models/best.engine.bak-<時間> ~/tel2026/models/best.engine，再重啟 vision。
set -euo pipefail

PT="${1:?用法：./export_and_deploy.sh <best.pt>}"
JETSON="${JETSON:-zheng@zheng.local}"
IMGSZ="${IMGSZ:-640}"
STAMP="$(date +%Y%m%d_%H%M%S)"
REMOTE_DIR="tel2026/models"

echo "== 1/4 匯出 ONNX（訓練機）"
yolo export model="$PT" format=onnx imgsz="$IMGSZ" opset=17 simplify=True dynamic=False
ONNX="${PT%.pt}.onnx"
ls -l "$ONNX"

if [ "${DRY_RUN:-0}" = "1" ]; then
  echo "DRY_RUN=1：到這裡為止（$ONNX）"
  exit 0
fi

echo "== 2/4 傳到 Jetson $JETSON:~/$REMOTE_DIR/"
ssh "$JETSON" "mkdir -p ~/$REMOTE_DIR"
rsync -av "$ONNX" "$JETSON:$REMOTE_DIR/new_$STAMP.onnx"
rsync -av "$PT" "$JETSON:$REMOTE_DIR/new_$STAMP.pt"   # 留一份 .pt，下次 autolabel 用

echo "== 3/4 在 Jetson 上轉 TensorRT（約 5–10 分鐘）＋試跑"
ssh -t "$JETSON" bash -s -- "$STAMP" "$IMGSZ" <<'REMOTE'
set -euo pipefail
STAMP="$1"; IMGSZ="$2"
cd ~/tel2026/models
TRTEXEC=/usr/src/tensorrt/bin/trtexec
if [ -x "$TRTEXEC" ]; then
  "$TRTEXEC" --onnx="new_$STAMP.onnx" --saveEngine="new_$STAMP.engine" --fp16 > "new_$STAMP.trtexec.log" 2>&1 \
    || { tail -20 "new_$STAMP.trtexec.log"; exit 1; }
else
  echo "找不到 trtexec，改用 ultralytics 從 .pt 轉"
  ~/yolo-env/bin/yolo export model="new_$STAMP.pt" format=engine half=True imgsz="$IMGSZ"
fi
# 用和 vision 服務同一套環境試跑一張空白圖，確定載入得了
~/yolo-env/bin/python - "new_$STAMP.engine" "$IMGSZ" <<'PY'
import sys, numpy as np
from ultralytics import YOLO
m = YOLO(sys.argv[1], task="detect")
m.predict(np.zeros((480, 640, 3), np.uint8), imgsz=int(sys.argv[2]), verbose=False)
print("試跑 OK")
PY
[ -f best.engine ] && cp best.engine "best.engine.bak-$STAMP"
cp "new_$STAMP.engine" best.engine
cp "new_$STAMP.pt" best.pt
echo "已換成新模型（舊的：best.engine.bak-$STAMP）"
REMOTE

echo "== 4/4 重啟 vision 服務（會要 Jetson 的 sudo 密碼）"
ssh -t "$JETSON" "sudo systemctl restart vision && sleep 8 && systemctl --no-pager status vision | head -15"
echo "完成。看 Elastic 的 Vision fps／Jetson視覺燈，或瀏覽器開 http://zheng.local:1181/"
