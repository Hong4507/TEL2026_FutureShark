# TEL2026 機器人現況簡報（2026-11-14 晚）

> 給 Claude Chat：這份是機器人目前的完整狀況，讓你跟我（隊員）一起討論、修正細節。
> 程式在另一個 Claude Code 對話裡實作。**討論出的決定請記在文末「討論紀錄」**，之後我會把這份檔案帶回 Claude Code 更新程式與進度。
> 你看不到程式碼，以下的類別名稱、常數、NT 路徑都是實際程式裡的名稱，討論時可以直接引用。

---

## 0. 時程與限制（最重要）

| 日期 | 事項 |
|---|---|
| **11/15（明天）** | **最後一天能碰機器人**，其他日子只能遠端連 Jetson |
| 11/16 ~ 11/20 | 只能遠端／模擬 |
| **11/21** | 比賽 |

- 比賽現場：準備時間與救援卡時間可以改程式；**場地沒有 Wi-Fi**；燈光可能干擾。
- 第二操作手**不能碰電腦**，筆電只顯示狀態。
- 明天預定順序：① Manual Mode 射擊 → ② 收資料＋訓練 YOLO（最花時間，建議一早先收資料讓它在背景訓練）→ ③ 量 TF-Luna 插值表。

---

## 1. 比賽規則重點（TEL2026，2026_rule_0903.pdf）

- **場地**：黑色板子，3×3 個白色 Ø40 cm 大洞；欄中心距約 90 cm、列間距 20 cm；板子傾角約 85°。上排上方有 3 個 Ø20 cm **加分洞**，前 60 秒其中一個隨機亮紅色 LED 燈條（洞外約 5 cm）。起點線離九宮格約 250 cm。
- **前 60 秒必須自動**，且規則要求用 AI 影像辨識（例如 YOLO）。啟動後不能再碰遙控器。
- 禁止可見光瞄準器、雷射筆。
- **上半場（排名賽）**：3 分鐘、27 顆球。每洞 2 分、每洞最多算 3 顆（九宮格滿分 54）；亮燈加分洞每顆 4 分、最多 3 顆，只在前 60 秒算。補球要用救援卡（每隊 3 張）回起點區補，比賽時間繼續跑。同分比序：加分洞 → 打中的不同洞數 → 總顆數。第 1–4 名直接晉級第二輪，5–8 名下半場第一輪分數 ×1.2，9–12 名 ×1.1。
- **下半場（淘汰賽）**：18 顆、不能補球；兩隊共用一個九宮格（球色不同），可以阻擋。每洞最多算 2 顆、加分洞最多 2 顆（8 分）。連線加分：斜線 +8、直線 +4、橫線 +2，每條線只算一次，連線最多 34 分。同分比序：加分洞 → 不同洞數 → 連線數。

---

## 2. 硬體

| 部件 | 細節 |
|---|---|
| 主控制器 | **SystemCore**（WPILib 2027 alpha-8，Java 25，從 roboRIO 移植過來），只有 6 個 Smart I/O |
| 底盤 | 差速、4 顆 SparkMax（CAN 14/22 左、27/1 右），輪徑 4"、減速比 6 |
| IMU | SystemCore 內建 OnboardIMU（NavX 沒有 2027 版，已移除） |
| 射球器 | 上下兩顆飛輪 **TalonFX**（CAN 13 上、11 下，Phoenix Pro，用 FOC），下飛輪速度 = 上的 80%，最高 565 rad/s |
| 砲台仰角 | SparkMax CAN 32 + **CANcoder CAN 15**（讀絕對位置），**只能調仰角，不能左右轉**；範圍 0.698 ~ 1.45 rad；**射球方向朝機器人右側** |
| 扳機 | SparkMax CAN 12，通電 0.15~0.2 秒推一顆球 |
| Storage | 3 顆有刷馬達（左 storage、右 storage、trigger），2 塊 L298N，Smart I/O 0~5 當開關／方向（全速，沒有調速） |
| 測距（車身） | **2 顆 TF-Luna**，接 SystemCore I2C 0（左）／I2C 1（右），用來量到目標的距離＋擺正對牆的角度 |
| 測距（砲台）| **第 3 顆 TF-Luna**，固定裝在砲台正前方、本身有固定仰角（不跟砲台動），I2C 接 Jetson，用來判斷光束是否穿過洞（= 左右對準了某一欄） |
| 協處理器 | Jetson Orin Nano：跑 YOLO 視覺、讀砲台 TF-Luna，經 NetworkTables 傳給 SystemCore |
| 攝影機 | **Limelight 2 當網路攝影機**（MJPEG `http://10.71.30.12:5802/`，640×480，15 fps），實測 HFOV 62.5°／VFOV 48.9°，綠燈已關；YOLO 在 Jetson 跑（規則要求 AI 辨識）。之前試過的 Arducam OV9281 USB 一直斷線、webcam 只有 15 fps 且沒有曝光控制 |
| 遙控器 | FlySky，i-BUS 經 USB 轉 TTL（CP2102）直接接 SystemCore；Jetson 轉發是備援。通道：CH1 LX、CH2 LY、CH3 RY、CH4 RX、CH5 左開關、CH6 右開關 |
| 啟用機器人 | 沒有 Driver Station，用 **MockDS**（程式自己送 DS 封包），FlySky 左開關啟用／停用 |
| 網路 | SystemCore 10.71.30.2、Jetson 10.71.30.11、Limelight 10.71.30.12，用交換器連；隊號 7130 |
| 球 | 機器人可以一次裝 27 顆 |

---

## 3. 軟體架構

- AdvantageKit（IO 層分實機／模擬），log 都在 NT `/AdvantageKit/...`，Elastic 當儀表板。
- 子系統：`Drive`、`Turret`、`Stroage`（拼字就是這樣）、`Vision`、`TurretLidar`（砲台 TF-Luna）、`StatusBoard`（給 Elastic）、`Scoreboard`（射擊記錄，已不顯示）。
- 模擬器能跑（`./gradlew run`），有 50+ 個單元／模擬測試（VisionAlign 閉迴路、完整自動流程、LaserAlign、插值表…）。

### 3.1 操作模式（`Constants.kRobotMode`）
停用時用右搖桿選：**上 = Manual、中間 = Assist、下 = Auto**。
- **Manual**（右開關開）：`ManualTurret`，右搖桿 Y 直接控制仰角，左搖桿 X 往右推擊發，飛輪全速。可用 NT `/Tuning/Manual/PitchRad` 把仰角固定在某個值（量插值表用）。
- **Assist**（右開關開）：`AssistTurret`，依 TF-Luna 距離自動算仰角（目標＝加分洞高度），右搖桿 Y 微調 ±0.1 rad，左搖桿 X 擊發。
- **Auto 模式下的射擊**（右開關開＋右搖桿方向）：`ColumnShot` 自動依距離算仰角、飛輪到速自動擊發；左下 = 低列 1 發、中下 = 中列、右下 = 高列、右上 = 低中高各 2 發。
- **LaserAlign**：右搖桿左上（非停用時），用左右 TF-Luna 擺正對牆（有 2 秒逾時和方向錯誤自動停止）。
- **自動流程（前 60 秒）**：停用狀態 + 左開關 + 右搖桿左上撥一下 → `AutoRoutines.bonusThenColumn`（onTrue，放開搖桿不會取消；左開關關掉才取消）。

### 3.2 自動策略（已和隊員討論定案）
保守：自動只做手動做不到的事、避免碰撞。
1. 跑 PathPlanner 路徑到中欄。
2. 等最多 2 秒找亮燈加分洞 → `VisionAlign LIT` 對準亮燈那欄（找不到就對中欄、不打加分洞）。
3. 射加分洞 → 同一欄的低／中／高。**每發前都用 `VisionAlign NEAREST` 重新對準**（被撞歪也能修回）。
4. 不換欄，換欄交給手動。
- 發數 NT 可調：`/Tuning/Auto/<FIRST|SECOND>/BonusBalls`、`/BallsPerHole`（預設上半場 3/3、下半場 2/2）。依實測命中率用 `strategy_calc.py` 算建議值。
- 模擬：上半場 21.7–23.5 s、下半場 16.4–19.7 s（**不含飛輪加速時間**，ColumnShot 每次結束都會停飛輪）。
- 筆電 Elastic 選單 `Auto Choices`：上／下半場 × 左／右起點。

### 3.3 視覺（YOLO11）
- Jetson `vision_nt_publisher.py`（systemd 服務 `vision`）→ NT `/Vision/targets`（每個偵測 6 個數：class、tx、ty、confidence、width、litScore）＋ heartbeat／latencyMs／fps。約 40 FPS、處理延遲約 20 ms（但現在 Limelight 只有 15 fps）。
- 類別：`hole`（大洞）、`bonus`（加分洞）。**亮燈不靠 YOLO 判斷**，是看加分洞外圈的紅色像素比例（`litScore`），門檻 `Vision/LitThreshold`（預設 0.15）。練習場沒有 LED 燈條。
- 機器人端 `Vision` 把偵測分成欄（欄距驗證，防誤判框），`VisionAlign` 用 P 控制沿牆前後移動對準某一欄：kP 1.5、最高 0.5 m/s、容許誤差 2 cm、連續 3 張畫面達標才結束；找不到 2 s／跟丟 1.5 s／總共 5 s 逾時。
- 目前模型：Jetson 上有 `best.engine`，但資料集主要是合成資料（官方設計圖），**還需要用 Limelight 拍真實場地資料重新訓練**（訓練機：4060Ti Ubuntu 或 1060 Windows）。

### 3.4 距離與仰角
- `RangeSensor/TargetDistance` = 左右 TF-Luna 平均（含傾角修正）+ `kWallToTargetMeters`（2.5）+ `kSensorOffsetMeters`（0.04）；都無效時用預設 3.0 m。
- 仰角原本用彈道公式：`π/2 − atan2(目標高 − 0.52 + d²×0.019358, d)`。
- **新：`ShotTable` 插值表**：每列（低 1.2 m／中 1.8 m／高 2.4 m／加分洞 2.9 m）一張「距離 → 仰角」表，表內線性插值，表外用公式並和表的端點接起來；**表目前是空的 → 行為等於原公式**。明天在 Manual 模式量。

### 3.5 砲台 TF-Luna 對準洞偵測（今天新增，未上機）
- Jetson `turret_tfluna_nt_publisher.py`（服務 `turret-tfluna`，I2C bus 7，100 Hz）→ NT `/TurretLidar/*`。
- 判斷：讀值 0（TF-Luna 前方沒東西或太遠會回傳 0）／訊號 < 100／比預期板子距離遠 > 0.3 m → **穿過洞**；± 0.3 m → 打在板上；近很多 → 被擋住。debounce 0.06 s。
- **只在 Elastic 亮燈，不阻止射擊**。
- 待量：安裝高度、光束仰角、和射球點的水平差。
- ⚠️ **風險**：板子是黑的，TF-Luna 在低反射率時量程約 2.5 m，可能打在板子上也讀 0，那就分不出洞和板子。

### 3.6 Elastic（筆電 1280×512 可視範圍）
- **狀態**分頁（只有顯示）：模式、飛輪（停止／加速 xx%／就緒）、仰角、射出數、電池、警示列（列出所有斷線部件）、9 個連線燈（遙控器、Jetson 視覺、左／右／砲台 TF-Luna、上／下飛輪、仰角編碼器、IMU）、飛輪到速、目標距離、砲台 TF-Luna 距離、對準洞、Auto 階段、Enabled。
- **賽前**分頁：Auto 選單、加分洞發數、手動固定仰角、最後一筆查表樣本。
- 原本的人工記分九宮格已移除。

---

## 4. 已在實機驗證 ✅

- 底盤、飛輪、砲台可動（11/07）；Phoenix 韌體要更新才能配合 2027 函式庫。
- MockDS 可用 FlySky 左開關啟用／停用。
- FlySky 經 USB-TTL 直連 SystemCore 正常（11/14）。
- Jetson 視覺服務、Limelight 串流可用；Limelight HFOV 62.5° 已量、綠燈已關。
- Jetson ↔ SystemCore NT 延遲：中位數 10 ms、最大 31 ms（11/12 修正 NT 發布週期後）。

## 5. 寫好但**還沒上機驗證** ⚠️

| 項目 | 明天要怎麼確認 |
|---|---|
| Storage 三馬達（L298N） | 每顆方向對不對，反了就對調該馬達 OUT 線 |
| 射一發後自動補球（`ballFired` 改看指令） | 射一發 → Storage 反轉 0.3 s 再正轉送球 |
| 砲台 CANcoder 絕對位置 | `kEncoderMagnetOffsetRot`：砲台放最低點讀 `Turret/PitchEncoderAbsRot`，填負值；往上推讀值要變大 |
| VisionAlign 實機精度 | 對準後射一顆，偏哪邊調 `kShooterForwardOfCameraMeters`（目前 0.15 m）；方向反了改 `kImageRightIsRobotForward` |
| LitThreshold | 需要 LED 燈條（練習場沒有） |
| TF-Luna 距離校正 | `kBaselineMeters`（0.30，TODO）、`kWallToTargetMeters`、`kSensorOffsetMeters`；LaserAlign 方向 `kYawSign` |
| 驅動 PID、IMU 安裝方向 | 從 11/07 一直沒測 |
| 插值表、砲台 TF-Luna、Elastic 狀態頁 | 今天晚上才寫 |
| 飛輪加速時間、各列命中率 | 用來決定自動發數、要不要讓飛輪整段自動流程不停轉 |

## 6. 還沒做

- YOLO 用真實場地資料重新訓練（明天收資料）。
- Jetson 錄影回放到模擬器（遠端可做）。
- FlySky 斷訊保護（低優先，模擬測試已部分涵蓋）。

---

## 7. 我想跟你討論的問題（可以補充）

1. **明天時間怎麼排最有效？** 只有一天，項目很多（見第 5 節），哪些可以砍、哪些可以平行做？
2. **插值表怎麼量最省時？** 每列要幾個距離？比賽時實際射擊距離大概固定在起點線附近（約 2.5 m）嗎？要不要只量一個距離就好？
3. **砲台 TF-Luna 對準洞**：如果黑板讀不到，有沒有替代方案？光束仰角該怎麼選（要穿過哪一列）？
4. **YOLO 資料收集**：要拍多少張、哪些角度／距離／燈光？Limelight 15 fps 夠不夠？場地燈光干擾怎麼處理？
5. **自動流程**：飛輪加速時間如果很長，要不要整段不停轉？下半場要不要改先打自己那一側的欄？
6. **比賽當天的檢查流程**（開機 → 確認連線 → 選 Auto → 啟動）要怎麼設計得簡單、不會出錯？

---

## 8. 討論紀錄（請 Claude Chat 在這裡整理結論）

> 格式建議：每一條寫「決定／要改什麼／為什麼」，標上影響的程式部分（類別或常數名稱）。我會把這段帶回 Claude Code 實作。

- 
