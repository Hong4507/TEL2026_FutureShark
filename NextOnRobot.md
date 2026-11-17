# 下次上機待辦（11/15 晚整理）

> 遠端期間（YOLO 標記）不用管這份。下次能碰機器人時從上往下做。

## 0. 部署
- [ ] 筆電 `git pull`（branch `claude/task-planning-development-6bppjw`）→ `./gradlew build` → `./gradlew deploy`
- [ ] Elastic 重新載入 `src/main/deploy/elastic-layout.json`
- [ ] Jetson：新的 `vision_nt_publisher.py`（存圖）、新模型 `best.engine`（YOLO 訓練完的話）

## 1. 插值表（`tools/measure/shot_table_1115.csv`）
- [ ] 目前 3 點是 11/15 抄的**砲台 Luna 原始斜距**，已用 23° 換算成約 3.9 m：
      低 3.905 m / 1.3744、中 3.905 m / 1.2298、高 3.887 m / 1.0915（各 5/5）。**還沒寫進 ShotTable.java。**
- [ ] 比賽射擊距離（約 2.5 m）每列再補 1 點（填 `distance_m`，或 `luna_slant_m` 填砲台 Luna 原始讀值）
- [ ] 量的時候看 console 那行最後的「和車身Luna差」：幾公分內 OK；差很多 → 車身 Luna 或常數有問題，先解決
- [ ] `python tools/measure/apply_measurements.py` → 看 diff → `y` → `./gradlew build` → deploy

## 2. 砲台 Luna（23°）
- [ ] 手推機器人沿牆走，Elastic「砲台Luna」：洞的位置要變「穿過洞」、板子上是「打在板上」。**黑板也讀 0 → 找洞 Auto 不能用**
- [ ] 機器人要擺在光束打得到洞的距離：牆距約 **3.1 m（中列）** 或 **1.7 m（低列）**，±0.35 m。2.5 m 打在兩列中間的板子
- [ ] Elastic 顯示 `太遠 +0.42m`／`太近 -0.30m` → 照數字前後調

## 3. 找洞 Auto（Elastic 賽前「Auto 方式」= 砲台Luna找洞）
- [ ] 右起點往**車頭**走、左起點往**車尾**走（都往中欄）。方向反了 → 改 `AutoRoutines.Plan.lunaDirection()` 的正負號
- [ ] 「Luna 前進距離」（預設 0.3 m）依起點位置調
- [ ] 被擋住時原地轉找洞：右起點逆時針、左起點順時針
- [ ] NT 看 `LunaAuto/AlignResult`、`ScanFound`、`Auto/Stage`
- [ ] ⚠️ 規則要求前 60 秒用 AI 影像辨識：先問裁判不用視覺的 Auto 可不可以。不行就「Auto 方式」選「視覺」

## 4. Limelight（30°）＋ YOLO
- [ ] 存圖畫面**整個偏藍、燈光過曝**：在 Limelight 網頁 `http://10.71.30.12:5801` 調白平衡（紅色要是紅色）、曝光調低並固定。
      亮燈加分洞是看紅色像素，偏藍可能讓 litScore 失效 → 用紅色 LED 燈條＋紙板試 `Vision/Targets` 第 6 個值
- [ ] YOLO 維持舊模型（11/17 在真實存圖上抽 114 張：75% 抓到 ≥3 個大洞）。上機看 `Vision/DetectionCount` 穩不穩
- [ ] VisionAlign 對準後射一顆，偏哪邊調 `kShooterForwardOfCameraMeters`
