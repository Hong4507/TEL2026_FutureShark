# 11/14–11/15 實機檢查清單

只有這兩天能碰機器人（比賽 11/21）。模擬能做的都做完了，這兩天只做**量測與校正**。
照順序做：前面的項目會影響後面的量測。

> ⚠️ NT `/Tuning/*` 改的值**重開機就回到預設**。量到的最終值一定要寫回程式碼（表格「寫回」欄）再部署。
> 每一段做完就填「實測值」欄，最後一節統一寫回並 commit。

---

## 0. 出發前

- [ ] 筆電：最新程式已 build（`./gradlew build`），Elastic 版面 `src/main/deploy/elastic-layout.json` 能載入
- [ ] Jetson：`vision` 服務開機自動啟動（✅ 11/12 已裝好），`best.engine` 是最新模型
- [ ] 帶：捲尺、量角器／水平儀、**紅色 LED 燈條**（練習場沒有，調 LitThreshold 必需）、球（27 顆）、延長線、電池全充
- [ ] 帶：Jetson 用網路線（SystemCore eth0 10.71.30.2 ↔ Jetson 10.71.30.11）

---

## 1. 開機與通訊（約 15 分）

| # | 項目 | 怎麼測 | 合格條件 |
|---|------|--------|----------|
| 1.1 | Jetson → 機器人 NT | 開機等 1 分鐘，看 AdvantageScope／Elastic | `Vision/Connected` = true；`Vision/AgeMs` < 50；`Vision/LatencyMs` ≈ 20；`Vision/Fps` > 25（11/08 實測 40+） |
| 1.2 | Jetson 即時畫面 | 瀏覽器開 `http://10.71.30.11:1181/` | 看得到畫面與框 |
| 1.3 | FlySky 開關啟用／停用 | 左開關上 → 下 → 上（MockDS 改過 sequence 保留） | 每次都能啟用、停用；拔 FlySky 接收機或關遙控器 → 機器人停（failsafe） |
| 1.4 | TF-Luna 左右 | 看 `RangeSensor/TFLuna/LeftConnected`、`RightConnected` | 都 true；手擋左邊 → `LeftMeters` 變小（沒接反） |
| 1.5 | TF-Luna 傾角正負 | 機器人斜對牆（左邊較近） | `TiltRad` 符號和程式假設一致；LaserAlign 會往對的方向轉正 |

---

## 2. 底盤（約 20 分）

| # | 項目 | 怎麼測 | 合格條件 | 寫回 |
|---|------|--------|----------|------|
| 2.1 | IMU 安裝方向 | 架高或在地上手推機器人逆時針轉 90° | `Drive/Gyro/YawRad` 增加約 +1.57（+90°）（逆時針為正）；不對就改 `MountOrientation`（目前 `FLAT`） | `GyroIOOnboardIMU.java` |
| 2.2 | 速度 PID | 低速、中速直走，看 `Drive/LeftVelocityMetersPerSec`、`RightVelocityMetersPerSec` 跟搖桿指令速度比 | 穩態誤差 < 10%，不震盪。目前 kP=0.2、kV=2.283、kS=0（REVLib RPM 換算後縮放過，未實測） | `DriveConstants.java` |
| 2.3 | 直走不偏 | 開 3 m | 左右輪實際距離差 < 3 cm | — |

---

## 3. 攝影機（重新裝在射球方向）（約 30 分）

| # | 項目 | 怎麼測 | 合格條件 | 實測值 | 寫回 |
|---|------|--------|----------|--------|------|
| 3.1 | 攝影機固定 | 裝好後鎖死，**之後不能再動**，動了 3.3 要重量 | — | | — |
| 3.2 | `kImageRightIsRobotForward` | 對著九宮格，機器人**往前**慢慢開 | 目標在畫面往**右**移 → `false`（目前值）；往左 → `true` | | `VisionConstants.java` |
| 3.3 | `kShooterForwardOfCameraMeters` | 量射球出口中心到鏡頭中心，沿機器人前進方向的距離（射球點在鏡頭前方為正） | 目前 0.15 m | ____ m | `VisionConstants.java` |
| 3.4 | 偵測距離與光線 | 從起跑線（距九宮格約 250 cm）看 | 9 個洞都有框；加分洞有框；`Vision/DetectionCount` 穩定 | | — |
| 3.5 | `MinConfidence` | 看框的信心值、有沒有誤判（燈、隊服） | 沒有假框；目前 0.5 | ____ | `/Tuning/Vision/MinConfidence` → `VisionConstants.kDefaultMinConfidence` |

> 3.2、3.3 改完要重新部署才生效（不是 NT 參數）。

---

## 4. 亮燈加分洞（約 20 分）

| # | 項目 | 怎麼測 | 合格條件 | 實測值 |
|---|------|--------|----------|--------|
| 4.1 | 量 LitScore | LED 燈條貼在一個加分洞外約 5 cm，記錄 `Vision/Targets` 裡加分洞那筆的第 6 個值（LitScore，索引 5）：亮燈洞、沒亮的洞，各距離（250 cm、對準距離） | 亮／不亮兩組數字分得開 | 亮：____ 不亮：____ |
| 4.2 | `LitThreshold` | 取兩組中間值，寫進 `/Tuning/Vision/LitThreshold`（目前 0.15） | 三個加分洞輪流亮，`Vision/LitErrorMeters` 都指向亮的那欄；全不亮時沒有誤判 | ____ |
| 4.3 | 場地燈光 | 如果能開強光／換角度再看一次 | 不會把紅色隊服、燈當亮燈 | |

寫回：`VisionConstants.kDefaultLitThreshold`

---

## 5. 射球（約 60 分，最花時間）

| # | 項目 | 怎麼測 | 合格條件 | 實測值 |
|---|------|--------|----------|--------|
| 5.1 | 飛輪起轉時間 | 從 `ColumnShot` 開始到 `isNearTargetSpeeds` 成立（看 log 時間差） | 記下來 | ____ s |
| 5.2 | 低／中／高列命中率 | 對準後每列射 10 發（`ColumnShot` 用 TF-Luna 距離算仰角） | 記命中數；偏高／偏低整列一致 → 調 `FieldConstants.k*TargetHeight` 或彈道係數 `0.019358` | 低 __/10 中 __/10 高 __/10 |
| 5.3 | 加分洞命中率 | 射 10 發 | 記命中數 | __/10 |
| 5.4 | 算策略發數 | `python src/main/java/frc/jetson_tools/strategy_calc.py --bonus 6/10 --hole 14/20` | 把建議值寫進 `/Tuning/Auto/FIRST/BonusBalls`、`BallsPerHole`（下半場 `SECOND`） | |
| 5.5 | （看 5.1）飛輪不停 | 如果起轉 > 1 s：考慮讓自動流程整段飛輪持續轉，不在每次 `ColumnShot.end()` 停 | 決定要不要改 | |

---

## 6. VisionAlign 實機（約 30 分）

| # | 項目 | 怎麼測 | 合格條件 |
|---|------|--------|----------|
| 6.1 | 方向 | `VisionAlignCENTER`，機器人在中間欄左右各偏 30 cm 起步 | 往正確方向移動，沒有越走越遠（越走越遠 → 3.2 設反了） |
| 6.2 | 三欄 | L／C／R 各一次 | 停下後射一發，落點在洞內；誤差 < 2 cm（`/Tuning/VisionAlign/ToleranceMeters`） |
| 6.3 | 速度 | `MaxSpeedMps` 0.5（模擬建議值），會晃就降 | 不衝過頭、不震盪 |
| 6.4 | 亮燈欄 | 燈條貼在左欄加分洞，跑 `VisionAlignLIT` | 移到左欄 |
| 6.5 | 看不到目標 | 拿東西擋鏡頭 | 逾時後停，不亂衝 |

---

## 7. 完整自動流程（60 秒）（約 40 分）

| # | 項目 | 怎麼測 | 合格條件 |
|---|------|--------|----------|
| 7.1 | 選單 | Elastic「Auto Choices」選 上半場-右起點 | `Auto/Stage` 從 Idle 開始變化 |
| 7.2 | 右起點完整跑 | 從起跑區放 27 顆球，撥開關啟動，**之後不碰遙控器** | 60 秒內完成：走位 → 亮燈洞 → 同欄低中高；不撞場地 |
| 7.3 | 左起點 | 同上，選 上半場-左起點 | 同上 |
| 7.4 | 下半場模式 | 選 下半場-*，確認發數上限（加分 2、每洞 2） | 同上 |
| 7.5 | 被撞 | 跑到一半推機器人 ±30 cm | 會重新對準（VisionAlign NEAREST） |
| 7.6 | 時間 | 記每次總時間（模擬 16–24 s，不含起轉） | 加上 5.1 起轉時間後仍 < 60 s |

如果下半場會跟對手搶位：考慮先去自己那側的欄（不是中間），11/15 再決定。

---

## 8. Elastic 記分板

- [ ] 「記分」分頁：自動流程跑完，射球數有自動加；+／− 按鈕能修正
- [ ] 「賽前」分頁：Vision Connected、TF-Luna、Auto Choices 都有值
- [ ] 筆電 14 吋 150% 縮放下版面不被切掉

---

## 9. 收工前（每天）

- [ ] 把所有實測值寫回程式碼（NT 參數重開機會消失）：
  - `VisionConstants`：`kImageRightIsRobotForward`、`kShooterForwardOfCameraMeters`、`kDefaultLitThreshold`、`kDefaultMinConfidence`
  - `VisionAlign`：`kP`、`MaxSpeedMps`、`ToleranceMeters` 預設值
  - `AutoRoutines`：`BonusBalls`、`BallsPerHole` 預設值
  - `DriveConstants`：PID／前饋
  - `GyroIOOnboardIMU`：`MountOrientation`
- [ ] 重新部署，重開機確認值還在
- [ ] 記錄到 `Upgrade.md`
- [ ] git commit（目前只有 Init，所有改動都還沒 commit）
- [ ] 錄練習場影片（Jetson `record.py`），回家繼續訓練 YOLO
- [ ] 下載 AdvantageScope log 備份
