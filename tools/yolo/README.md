# YOLO 流程（11/15 收資料 → 11/16–20 遠端訓練、部署）

訓練機：4060Ti Ubuntu。Jetson：`zheng@zheng.local`（模型在 `~/tel2026/models/`，服務 `vision`）。

```
Jetson 存圖（vision_nt_publisher.py --record 或 Elastic「賽前」的 Jetson 存圖開關）
  ↓ ./pull_captures.sh                       照片 → ./captures/YYYYMMDD/HHMMSS_mmm.jpg，順便找舊 .pt → ./models/
  ↓ python thin.py --images captures --apply      抽稀：每 1.2 秒留一張、每時段最多 400 張（其餘搬到 captures_thinned/）
  ↓ python autolabel.py --model models/<舊>.pt --images captures
  ↓ 人工修正（X-AnyLabeling，見下）
  ↓ python make_dataset.py --images captures --out datasets/real1115
  ↓ ./train.sh datasets/real1115/data.yaml models/<舊>.pt     ← 一行開始訓練
  ↓ ./export_and_deploy.sh runs/<NAME>/weights/best.pt        ← ONNX → Jetson TensorRT → 重啟 vision
```

## 0. 第一次在訓練機上安裝

```bash
python3 -m venv ~/yolo && source ~/yolo/bin/activate
pip install ultralytics onnx onnxslim        # 會一起裝 CUDA 版 torch
yolo checks                                   # 看得到 4060Ti 就好
```

## 1. Jetson 存圖

- 開：Elastic「賽前」分頁的 **Jetson 存圖** 開關（NT `/Vision/Record`），或服務加 `--record`。
- 狀態：Elastic「狀態」分頁右下 **Jetson錄影**（`● 錄影 N 張`／`未錄影`／`停止：空間不足`）。
  NT：`/Vision/Recording`、`/Vision/RecordCount`、`/Vision/RecordFreeGB`、`/Vision/RecordStopReason`。
- 每秒 2.5 張（`--record-fps`）、JPEG 90、原始畫面（沒畫框）。剩餘空間 < 5 GB 自動停止。
- 存圖在另一個執行緒，佇列滿就丟幀，偵測 fps 不受影響（主迴圈每張只多約 0.1 ms）。
- **整天開著**：NT 開關在 Jetson 重開機後會回到關。要整天開就在 Jetson 上：
  ```bash
  sudo systemctl edit --full vision     # ExecStart 那行最後加上 --record
  sudo systemctl restart vision
  ```
  比賽前記得拿掉。

收資料重點（Robot-Brief §8.10）：距離 2–4 m、沿牆每一欄、機器人歪 ±15°、洞內有球／沒球、有人站在附近。
**分成幾段錄（中間停 1 分鐘以上）**，`make_dataset.py` 才能依時段切 val。Limelight 曝光先在 `http://10.71.30.12:5801` 調低並固定。

## 2. 拉照片＋舊模型

```bash
./pull_captures.sh            # JETSON=zheng@zheng.local 可改
```
⚠️ `best.engine` 是 TensorRT 檔，只能在建它的那台 Jetson 上用，**不能拿來在訓練機上自動標註**。
要用 `.pt`：script 會到 Jetson 的 `~/tel2026/models/`、`~/tel2026/jetson_tools/runs/*/weights/` 找；
找不到就在訓練機上找 `train.py` 的輸出 `runs/v0/weights/best.pt`（合成資料版）。都沒有就跳過自動標註、全部人工標。

## 2.5 抽稀

```bash
python thin.py --images captures                 # 先預覽每個時段會留幾張
python thin.py --images captures --apply         # 沒留的連 .txt 搬到 captures_thinned/（不刪）
```
每秒 2.5 張，相鄰幾乎一樣；全部人工修正太浪費，也會讓某一段（例如額外錄的近距離）佔太多。
預設 `--min-dt 1.2`（約留 1/3）、`--max-per-session 400`。之後想多加資料，從 `captures_thinned/` 搬回來即可。

## 3. 自動標註

```bash
python autolabel.py --model models/<舊>.pt --images captures          # conf 預設 0.25
```
- 每張圖都會有同名 `.txt`；標不到的寫空檔（負樣本，保留）。清單：`captures/autolabel_empty.txt`。
- 已經有 `.txt` 的不會被蓋掉（人工改過的安全），`--overwrite` 才重標。

## 4. 人工修正：X-AnyLabeling

選它的原因：pip 一行裝好、直接讀寫 YOLO txt、內建快捷鍵很快。（labelImg 已停止維護，新版 Python 會當掉；Label Studio 要開伺服器、匯入匯出比較麻煩。）

```bash
python3 -m venv ~/xal && ~/xal/bin/pip install "x-anylabeling-cvhub[cpu]"   # 需要 Python ≥ 3.11（Ubuntu 24.04 內建 3.12）
~/xal/bin/xanylabeling
```
Ubuntu 22.04（Python 3.10）：先 `sudo apt install python3.11 python3.11-venv`，第一行改用 `python3.11 -m venv ~/xal`。
如果開不起來缺 Qt 套件：`sudo apt install libxcb-cursor0 libxkbcommon-x11-0`。

使用：
1. File → Open Dir → 選 `captures/20261115`。
2. 上方選單 **Import → YOLO（Hbb）**，classes 選 `captures/20261115/classes.txt`（`hole`、`bonus`，順序不能變）。
3. 修正框（`W` 畫框、`D`／`A` 下一張／上一張、`Del` 刪框）。先看 `autolabel_empty.txt` 裡的圖。
4. **Export → YOLO（Hbb）** 存回同一個資料夾（覆蓋 `.txt`）。

## 5. 切資料集、訓練

```bash
python make_dataset.py --images captures --out datasets/real1115      # 每個時段切 20 秒一塊，各段挑約 20% 的塊當 val
./train.sh datasets/real1115/data.yaml models/<舊>.pt                  # 接續舊權重；沒有就省略第二個參數（yolo11n.pt）
```
- 為什麼依時段切：每秒 2.5 張，相鄰畫面幾乎一樣，隨機切 val 分數會虛高。切法寫在 `datasets/real1115/split.txt`。
- 訓練參數：`imgsz=640 epochs=100 patience=20 hsv_v=0.6 hsv_s=0.7`（場地燈光干擾，加大亮度增強；色相 `hsv_h` 保持很小，紅色燈圈要維持紅色）。
- 環境變數可改：`EPOCHS BATCH DEVICE IMGSZ NAME`。4060Ti 16GB 可 `BATCH=32`。

## 6. 部署到 Jetson

```bash
./export_and_deploy.sh runs/<NAME>/weights/best.pt
```
1. 訓練機匯出 ONNX（opset 17、固定 640）。
2. 傳到 Jetson `~/tel2026/models/new_<時間>.onnx`（連 `.pt` 一起，下次自動標註用）。
3. **在 Jetson 上**用 `trtexec --fp16` 轉成 engine（沒有 trtexec 就用 ultralytics 從 .pt 轉），先試跑一張圖，成功才換成 `best.engine`；舊的留成 `best.engine.bak-<時間>`。
4. `sudo systemctl restart vision`（會要 Jetson sudo 密碼）。

換回舊模型：`cp ~/tel2026/models/best.engine.bak-<時間> ~/tel2026/models/best.engine && sudo systemctl restart vision`。

## 測流程（不需要 GPU）

```bash
EPOCHS=1 DEVICE=cpu BATCH=8 WORKERS=0 ./train.sh datasets/real1115/data.yaml models/<舊>.pt
DRY_RUN=1 ./export_and_deploy.sh runs/<NAME>/weights/best.pt
```
