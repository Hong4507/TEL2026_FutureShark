# 給 Claude Code：11/15 上機前的工作

> 背景：今天（11/15）是**最後一天能碰機器人**，隊員出門前、在路上的時候請先把下面的事情做完。
> 完整的決定與理由在 `Robot-Brief.md` 第 8 節「討論紀錄」，這份只列要做的事和驗收條件。
> 做完請在最後的「回報」區塊填寫狀態。

---

## 任務 1：確認／完成 Robot-Brief §8.0 的優先項目（最重要）

依序檢查，沒做完的就照 §8 的內容實作：

| # | 項目 | 參考 |
|---|---|---|
| 1a | 新測距公式（左右 TF-Luna 改量九宮格牆面） | §8.1 |
| 1b | `kRangeReference = GRID_WALL / LOW_WALL` 切換，加上 NT `RangeSensor/LeftAmp`、`RightAmp` | §8.2 |
| 1c | Jetson 存圖模式（見任務 2） | §8.10 |
| 1d | 不跑路徑的自動版本 | §8.7 |
| 1e | 自動流程中飛輪不停轉 | §8.6 |
| 1f | Elastic READY 燈＋起點／半場／目標欄大字 | §8.9 |
| 1g | 濾波／死區／trim、砲台 Luna 動態預期距離、下半場欄選擇＋放寬逾時 | §8.3、8.4、8.8 |

**1a 必須加的單元測試**（常數：`kRightLunaInboardMeters = 0.07042`、`kLeftLunaInboardMeters = 0.4826`、`kBaselineMeters = 0.17072`、`kShooterInboardMeters` 暫定 0.25）：

| dR | dL | 預期 yaw | 預期 wall_dist | 預期 TargetDistance |
|---|---|---|---|---|
| 2.0000 | 2.41218 | 0° | 1.92958 | 2.17958 |
| 2.0000 | 2.43218 | ±6.68°（看 `kYawSign`） | 1.93958 × cos(6.68°) ≈ 1.92642 | ≈ 2.17642 |
| 0（無效） | 0（無效） | — | — | 預設 3.0 |
| 2.0（只有右有效） | 0 | 不算 yaw | 依現有單邊邏輯 | — |

- `LOW_WALL` 模式要和舊公式結果完全一樣（舊測試保留，改成在 `LOW_WALL` 下跑）。

**驗收**：
- `./gradlew build` 和所有測試通過。
- 模擬器能跑（`./gradlew run`）。
- 隊員到學校插上線就能直接部署：把部署指令寫在回報區。

---

## 任務 2：Jetson 存圖模式（`vision_nt_publisher.py`）

- **開關**：NT `/Vision/Record`（boolean，預設 false）＋啟動參數 `--record` 二擇一都可以開。
- **存圖**：
  - 存**原始畫面**，不要畫框。
  - 每秒 2–3 張（`--record-fps`，預設 2.5）。
  - JPEG 品質 90。
  - 路徑 `~/captures/YYYYMMDD/HHMMSS_mmm.jpg`。
- **NT 輸出**：`/Vision/RecordCount`、`/Vision/RecordFreeGB`。
- **保護**：剩餘空間 < 5 GB 自動停止存圖，並在 NT 標示原因；**存圖不能拖慢偵測**（用另一個執行緒或佇列，佇列滿就丟幀）。
- **Elastic 狀態頁**：加一個小燈「錄影中／張數」。
- **驗收**：
  - 在 Jetson 上用 `--record` 跑 1 分鐘，fps 和延遲沒有明顯下降。
  - 圖片數量約 150 張。
  - 回報 Jetson 目前剩餘空間。

---

## 任務 3：YOLO 流程腳本（放在 repo `tools/yolo/`）

目標：**收完資料 → 一行指令開始訓練**。訓練機是 4060Ti Ubuntu。

| 檔案 | 功能 |
|---|---|
| `pull_captures.sh` | `rsync` 從 Jetson（10.71.30.11）拉 `~/captures/` 到訓練機 |
| `autolabel.py` | 用舊模型自動標註 → YOLO txt 格式，conf 門檻 0.25，標不到的圖也保留（當負樣本或人工補標） |
| `make_dataset.py` | 依**錄影時段**分 train/val（不要隨機分，連續畫面很像，會讓 val 分數虛高），產生 `data.yaml`（類別：`hole`、`bonus`） |
| `train.sh` | 建議 `yolo11n`（或和現有模型同大小），`imgsz=640`，`epochs=100`，`patience=20`；加大亮度增強：`hsv_v=0.6`、`hsv_s=0.7`；可以接續舊權重微調 |
| `export_and_deploy.sh` | 匯出 ONNX → 傳到 Jetson → **在 Jetson 上**用 TensorRT 轉成 `best.engine` → 重啟 `vision` 服務 |

⚠️ 注意：
- `best.engine` 是 TensorRT 檔，**只能在建它的那台 Jetson 上用**，訓練機上不能拿它來自動標註。請確認舊模型的 `.pt` 檔在哪裡；找不到的話，就在回報區寫出來。
- 人工修正標註要用什麼工具（X-AnyLabeling、Label Studio 或 labelImg），請挑一個在 Ubuntu 上最快能裝好的，並把安裝指令寫進 `tools/yolo/README.md`。

**驗收**：用任務 2 錄的測試圖跑完整條流程（可以只訓練 1 個 epoch），確認每一步都通。

---

## 任務 4：量測記錄表＋轉換腳本

**4a. 建立 `tools/measure/shot_table_1115.csv`**：

```csv
row,distance_m,pitch_rad,hits_of_5,note
LOW,,,,
LOW,,,,
LOW,,,,
MID,,,,
...（LOW／MID／HIGH／BONUS 各 3 列，共 12 列）
```

**4b. 建立 `tools/measure/constants_1115.csv`**，列出 Robot-Brief §8.12 要量的項目：

```csv
name,value,unit,note
kShooterInboardMeters,,m,
kYawSign,,,
kLunaTrimMeters,,m,
kEncoderMagnetOffsetRot,,rot,
...
```

其他要量的項目：
- 左右 Luna 高度
- 矮牆高度
- 砲台 Luna 高度／仰角／和射球點的水平差
- 飛輪從 0 到就緒的時間

**4c. `tools/measure/apply_measurements.py`**：
- 讀上面兩個 CSV，自動更新 `ShotTable` 的資料和 `Constants` 裡對應的常數，空白欄位就跳過。
- 印出 diff 讓人確認。
- 把 `hits_of_5` 換算成命中率，直接餵給 `strategy_calc.py`，印出建議的 `BonusBalls`／`BallsPerHole`。

**驗收**：用假資料跑一次，`ShotTable` 測試仍然通過。

---

## 回報（Claude Code 填寫，11/15）

> ⚠️ **驗證狀態**：雲端環境的 Maven Central 一直間歇回 404／429，`./gradlew build`／`test` 在這裡**跑不完下載**（不是程式錯誤）。
> 改用 `javac` 直接編譯：**main 和 test 全部編譯通過**；不需要 WPILib 原生函式庫的單元測試 **19/19 通過**
> （`RangeFormulaTest`（1a 測試表 4 列＋LOW_WALL 與舊公式逐點相同）、`TimedMovingAverageTest`、`StatusBoardTest`、`ShotTableTest`）。
> **需要原生函式庫的模擬測試（LaserAlignSim、AutoRoutineSim、AutoTiming、TurretLidar、VisionAlign…）這裡沒跑到**，
> 請隊員出門前在筆電跑一次 `./gradlew build`（會跑全部測試），有紅的把輸出貼回來。

| 任務 | 狀態 | 備註 |
|---|---|---|
| 1a 測距公式 | 完成（模擬測試待筆電跑） | `DualTFLunaRangeSensor.solve()` 純函式；常數照 §8.1。單邊有效 = 該邊 d_frame + `kShooterInboardMeters`。新 NT：`RangeSensor/WallDistance`、`YawErrorDeg`、`RawFrameDiffMeters`（擺正時 = `kLunaTrimMeters` 要填的值） |
| 1b 矮牆切換＋Amp | 完成 | `RangeSensorConstants.kRangeReference = GRID_WALL / LOW_WALL`；NT `RangeSensor/LeftAmp`、`RightAmp`、`Reference`。LOW_WALL 預設常數 = 舊公式（測試逐點比對）。⚠️ 舊公式沒扣兩顆 Luna 的 0.412 m 內縮差，**真的要切 LOW_WALL 時**照 `kLowWallBaselineMeters` 註解改 4 個常數，否則 LaserAlign 角度會錯 |
| 1c／2 存圖模式 | 完成（Jetson 上未測） | `--record`／NT `/Vision/Record`；2.5 張/秒、JPEG 90、`~/captures/YYYYMMDD/HHMMSS_mmm.jpg`；NT `Recording`、`RecordCount`、`RecordFreeGB`、`RecordStatus`、`RecordStopReason`；< 5 GB 自動停；另一個執行緒＋佇列滿丟幀。本機假畫面測：4 秒存 10 張（= 150 張/分）、主迴圈每張只多 0.12 ms。Elastic 狀態頁右下「Jetson錄影」、賽前頁「Jetson 存圖」開關。**Jetson 剩餘空間：雲端連不到 10.71.30.11，無法回報**（啟動時會印、NT `/Vision/RecordFreeGB` 也看得到） |
| 1d 不跑路徑 Auto | 完成（模擬測試待跑） | Elastic 新選單 `Auto Path`：跑路徑／不跑路徑（擺在目標欄前）。不跑路徑時只做 VisionAlign＋射擊 |
| 1e 飛輪不停轉 | 完成 | 自動流程一開始就轉飛輪，`ColumnShot(..., stopFlywheelAtEnd=false)`，流程結束／取消才停；仍保留到速才擊發。模擬測試會檢查流程中飛輪只在最後停 1 次 |
| 1f READY 燈 | 完成 | `Status/Ready`：9 連線（含仰角編碼器）、電池 > 12.5 V、Jetson fps > 10、Auto 已選。不 READY 原因顯示在警示列。狀態頁第一列大字：起點／半場／目標欄／路徑。**Auto 選單預設改成「（未選）」**：沒選 READY 不亮；萬一沒選就啟動，會跑最安全的設定（上半場、不跑路徑、中欄） |
| 1g 其他 §8 項目 | 完成 | §8.3：0.2 s 移動平均（`kLunaFilterSec`）、LaserAlign 死區 1.5°（`kAlignDeadbandDeg`）、`kLunaTrimMeters`。§8.4：預期板距 = (牆距 + `TurretLidar.kInboardMeters`)/cos(仰角)，光束不在中列 ±0.15 m 內顯示「超出範圍」，NT `/Tuning/TurretLidar/Enabled` 關掉顯示「停用」（賽前頁有開關）。§8.8：`Auto Column` 選單（中欄／己方欄＝起點那側）、VisionAlign 總逾時 5→8 s、每次對準失敗重試 1 次 |
| 3 YOLO 腳本 | 完成 | `tools/yolo/`：pull_captures.sh、autolabel.py、make_dataset.py、train.sh、export_and_deploy.sh、README.md（X-AnyLabeling 安裝）。用假 captures（3 個時段）在 CPU 跑完整條：自動標註（含負樣本）→ 依時段切 → 訓練 1 epoch → 匯出 ONNX 都通；傳 Jetson／TensorRT／重啟服務那段要連得到 Jetson 才能測。**舊模型 `.pt` 位置：雲端看不到 Jetson 和訓練機，無法確認**。依 `train.py` 流程應該在訓練機 `runs/v0/weights/best.pt`，或曾複製到 Jetson `~/tel2026/models/best.pt`；`pull_captures.sh` 會自動去 Jetson 找 |
| 4 記錄表＋轉換腳本 | 完成（ShotTable 測試待筆電跑） | `tools/measure/shot_table_1115.csv`（12 列）、`constants_1115.csv`、`apply_measurements.py`（印 diff、確認才寫、空白跳過、命中率 → strategy_calc 建議發數）。假資料 dry-run 正確；`ShotTableTest` 在原始表通過 |

- **部署指令**（筆電，連上機器人網路後）：
  ```bash
  git pull
  ./gradlew build          # 先在有網路的地方跑一次（下載相依＋跑全部測試）
  ./gradlew deploy         # 連上機器人（隊號 7130，SystemCore 10.71.30.2）
  # Jetson 端（存圖模式）：
  scp src/main/java/frc/jetson_tools/vision_nt_publisher.py zheng@10.71.30.11:~/tel2026/jetson_tools/
  ssh -t zheng@10.71.30.11 'sudo systemctl restart vision'
  ```
  Elastic 版面也改了：重新載入 `src/main/deploy/elastic-layout.json`。

- **需要隊員到學校手動處理的事**：
  1. **開機第一件事**：在射擊位置看 `RangeSensor/LeftAmp`、`RightAmp`、`TFLuna/LeftMeters`／`RightMeters`，沿牆走一趟。讀到 0／Amp 太低 → Luna 調回對矮牆，`kRangeReference = LOW_WALL`（＋註解裡 4 個常數）重新部署。**量插值表前就要決定**。
  2. LaserAlign 試一次：`LaserAlign/WrongDirection` = true → `kYawSign = -1`。擺正後讀 `RangeSensor/RawFrameDiffMeters` 填 `kLunaTrimMeters`。
  3. 量 `kShooterInboardMeters`、砲台 Luna 高度／仰角／內縮、`kEncoderMagnetOffsetRot`、飛輪到速時間 → 填 `tools/measure/constants_1115.csv`。
  4. 插值表照 §8.5 量，填 `tools/measure/shot_table_1115.csv`，然後 `python tools/measure/apply_measurements.py` → `./gradlew test` → 部署。
  5. Jetson 存圖：Elastic 賽前頁開「Jetson 存圖」，或 `sudo systemctl edit --full vision` 在 ExecStart 加 `--record`（整天開著；比賽前拿掉）。確認狀態頁「Jetson錄影」張數在增加、`/Vision/RecordFreeGB`、偵測 fps 沒掉。
  6. 確認舊模型 `.pt` 在哪（Jetson `ls ~/tel2026/models/`、訓練機 `ls runs/*/weights/`）。
  7. 砲台 Luna 測 10 分鐘，不行就賽前頁關掉「砲台Luna 啟用」（並把 `TurretLidar` 的預設改 false 再部署，否則重開機會回到開）。
  8. Auto 選單現在有 3 個：`Auto Choices`（半場＋起點，預設未選）、`Auto Path`、`Auto Column`。
