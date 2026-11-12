# Jetson 工具程式

全部在 Jetson 的虛擬環境中執行：

```bash
source ~/yolo-env/bin/activate
cd ~/tel2026/jetson_tools
```

| 檔案 | 用途 |
|---|---|
| `nt_test.py` | 測試 Jetson ↔ SystemCore（或筆電模擬）的 NetworkTables 連線 |
| `camera_check.py` | 確認攝影機實際解析度、幀率，存一張截圖 |
| `record.py` | 錄製訓練用影片（SSH 下即可） |
| `calibrate.py` | 棋盤格相機校正：`capture` 拍照、`solve` 計算 |
| `flysky_nt_publisher.py` | 讀 FlySky 接收機 i-BUS，發布到 NT `/FlySky/channels`（μs）、`/FlySky/heartbeat`，給 `FlySkyController` 用 |
| `flysky.service` | 開機自動執行 `flysky_nt_publisher.py` 的 systemd 服務 |
| `chessboard_9x6_25mm.pdf` | 校正用棋盤格，A4 橫式，以「實際大小 / 100%」列印 |
| `vision_nt_publisher.py` | **比賽用**：攝影機 → YOLO → 角度 → NT `/Vision`；瀏覽器看即時畫面 `http://<Jetson IP>:1181/` |
| `vision.service` | 開機自動執行 `vision_nt_publisher.py` |
| `vision_fake_publisher.py` | 假的偵測結果，沒有模型時測 NT 與機器人端 |
| `synth_dataset.py` | 用官方設計圖（`assets/nine_grid_design.jpg`）產生合成訓練資料（PC 上跑） |
| `extract_frames.py` | 從錄影抽出要標註的畫面，跳過重複畫面 |
| `autolabel.py` | 用現有模型先自動標註，人只要修正 |
| `train.py` | 合併合成 + 真實資料訓練 YOLO11（GPU 電腦上跑） |

## 九宮格 YOLO 流程

類別：`0 hole`（Ø40 cm 大洞）、`1 bonus`（Ø20 cm 加分洞）。亮燈不是 YOLO 類別，由 `vision_nt_publisher.py` 看加分洞外圈的紅色比例判斷。

### 1. 合成資料 + v0（4060Ti Ubuntu）

```bash
pip install ultralytics opencv-python
python synth_dataset.py --preview 12 --out preview/        # 先看框有沒有對齊
python synth_dataset.py --out datasets/synth --count 4000
python train.py --synth datasets/synth --name v0 --batch 32
```

### 2. Jetson 環境與速度（遠端）

```bash
# 依 Ultralytics 官方 Jetson 教學，在 yolo-env 安裝對應 JetPack 版本的 torch / torchvision，再：
pip install ultralytics pyntcore
yolo export model=best.pt format=engine half=True imgsz=640   # 一定要在 Jetson 上轉
python vision_nt_publisher.py --model best.engine --print-only   # 看 fps 和延遲
```

### 3. 收真實資料（11/14）

```bash
python camera_check.py --width 1280 --height 720
python calibrate.py capture --count 25 --square <實際量到的 mm>
python calibrate.py solve --square <實際量到的 mm>
python record.py --name 2m5_left_normal          # 依錄影清單錄
python vision_nt_publisher.py --model best.engine --calib camera_calib.yaml --save-dir captures --save-every 1
```

錄影清單（每段 20～40 秒，邊錄邊慢慢移動機器人）：
- 距離：2 m、2.5 m、3 m、3.5 m
- 位置：起始框左、中、右，以及正對各欄
- 狀況：場地燈全開／半暗、洞裡有球／沒球、被人或機器人擋住一部分、機器人移動中（模糊）
- 沒有九宮格的畫面（場館其他方向）幾段，當作負樣本

### 4. 標註與 v1（11/14 晚上）

```bash
python extract_frames.py recordings/ --out frames --every 10
cp captures/*.jpg frames/
python autolabel.py --model runs/v0/weights/best.pt --images frames --conf 0.4
# 用 X-AnyLabeling 或 Roboflow 開 frames/（YOLO 格式）修正標註
python train.py --synth datasets/synth --real frames --name v1 --weights runs/v0/weights/best.pt
```

把 `runs/v1/weights/best.pt` 複製到 Jetson，轉成 engine，放到 `~/tel2026/models/best.engine`，重開 `vision` 服務。

### 開機自動啟動

```bash
sudo cp vision.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vision
journalctl -u vision -f
```

## 常用指令

```bash
python nt_test.py --team 7130            # 連主控制器
python nt_test.py --server <筆電IP>       # 連筆電模擬
python camera_check.py --width 1280 --height 720
python record.py --name 3m_left30_normal
python calibrate.py capture --count 25 --square 25
python calibrate.py solve --square 25
python flysky_nt_publisher.py --print-only     # 只看通道數值，對照搖桿
python flysky_nt_publisher.py --team 7130      # 發布給主控制器
```

### FlySky 開機自動啟動

```bash
sudo usermod -aG dialout $USER                 # 讓使用者能開 /dev/ttyUSB0（重新登入生效）
pip install pyserial pyntcore
sudo cp flysky.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now flysky
journalctl -u flysky -f                        # 看輸出
```

`--square` 請填列印後用尺量到的方格實際邊長（mm）。
拍照、錄影、校正與比賽時，請使用同一個解析度。
