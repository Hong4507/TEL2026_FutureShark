# 比賽日上機計畫（2 小時）

> **目標：確認現有功能在實機能跑，然後凍結程式。不加新功能、不在現場修 bug。**
> 每段有時間上限，到了沒過 → 用「失敗時」那欄降級、記下來、往下一段。
> 卡住時：Elastic 截圖＋下面「看這些 NT」的值貼給 Claude。

---

## 前一晚（在家）
- [ ] `git pull`（branch `claude/task-planning-development-6bppjw`）→ **`.\gradlew build`**（學校可能下載不了相依）
- [ ] 電池全充、編號，電表量 > 12.5 V
- [ ] 抄好下面的「操作卡」給操作手
- [ ] FlySky failsafe 設好（斷訊 → 左右開關回「關」）

---

## 操作卡（給操作手）
**選模式**：左開關「關」→ 右搖桿擺好位置 → 左開關「開」
| 右搖桿 | 模式 |
|---|---|
| 左上 | **開始自動流程**（比賽開始時用） |
| 上 | Manual（手動仰角，量表用） |
| 中間 | Assist |
| 下 | **Auto 模式**（比賽手動期用這個） |
| 右上 | 送球（Storage＋Trigger 一直轉，清球用） |

**Auto 模式射擊**：右開關開＋右搖桿 → 左下＝低、右下＝中、右上＝高、正下＝低中高各 2（**按住到球射出**）
**對齊**：右搖桿左上（Elastic「對齊方式」預設砲台Luna找洞）
**左開關關 = 全部停**

**比賽流程**
1. 90 秒準備：車頭朝九宮格、裝球、左開關關 → Elastic 賽前頁選 **Auto（半場＋起點）** → 狀態頁 **READY** 燈
2. 開始：**右搖桿壓左上 → 左開關打開** → 放下遙控器（自動約 25–35 s）
3. 60 秒後：左開關關 → 右搖桿往下 → 左開關開（= Auto 模式）→ 射剩下的球

---

## 0:00–0:15　部署＋基本檢查 **P0**
- [ ] `.\gradlew deploy` → Elastic 連上（隊號 7130）
- [ ] 狀態頁 9 個連線燈：**底盤、上下飛輪、仰角編碼器**一定要綠；其他紅的記下來
- [ ] FlySky：`FlySky/ChannelsUs` 四個搖桿、兩個開關都會動
- [ ] Manual 模式右開關開：飛輪轉、仰角跟右搖桿動；左搖桿往右擊發後 Storage／Trigger 會送下一顆球（`Storage/ShootFeedState`）
- [ ] 送球模式（右搖桿右上開機）：Storage＋Trigger 轉，左開關關就停
- **失敗時**：新程式有問題 → `git checkout 183158c` 部署上一個實機跑過的版本，Auto 改成人工擺位

---

## 0:15–0:55　自動流程 ×3 **P0 最重要**
機器人放起點（**車頭朝九宮格、離九宮格 5.8–7.0 m**）。Elastic：Auto 選擇＝上半場＋右起點、Auto 方式＝砲台Luna找洞。

每次看 `Auto/Stage` 依序跑過：
`BlindApproach` → `TurnToWall` →（`MeasureRetry`）→ `DriveToRange` → `SquareUp` → `Luna-Approach` → `Luna-Shoot` → `Done`

| 看到 | 正常 | 不正常 → 怎麼辦 |
|---|---|---|
| 盲走 2.55 m | 直直往九宮格走 | **歪掉**：底盤／IMU 有問題 → 改 `BlindApproachMeters` = 0，人工擺到 3.25 m |
| 轉 90° | **逆時針**（往左轉），右側朝九宮格 | **往右轉** → `StartYawToWallDeg` = **+90** |
| `Auto/StartDistanceMeters` | 3.2–4.5 的數字 | 一直 NaN（量不到黑板）→ 人工擺位版（見下） |
| 調整後 Elastic「目標距離」 | ≈ 3.25 | 差很多 → 看 `DriveDistance/ErrorMeters` |
| 「砲台Luna」欄 | `中列 打在板上`，經過洞時 `穿過洞` | 永遠不會穿過洞 → 看 `TurretLidar/MinusBaselineMeters`：洞的位置 < 0.10 → `/Tuning/TurretLidar/HoleMarginMeters` 設成它的一半 |
| 射 9 發 | 大部分進 | 偏同一邊 → 記下來；仰角不對 → 插值表問題（回家再處理） |

**人工擺位版（找洞一直失敗時）**：`BlindApproachMeters` = 0、`StartYawToWallDeg` = 0、賽前頁「砲台Luna 啟用」關 → 機器人**右側朝九宮格、擺在某一欄正前方 3.25 m** → 自動只射 9 發。

看這些 NT：`Auto/Stage`、`Auto/StartDistanceMeters`、`TurnByAngle/ErrorDeg`、`DriveDistance/ErrorMeters`、`LunaAuto/AlignResult`、`LunaAuto/AlignState`、`RangeSensor/Source`

---

## 0:55–1:25　手動期射擊 **P0**
Auto 模式、機器人在 3.25 m：
- [ ] 右搖桿 左下／右下／右上 各射 1 發，進對的列
- [ ] 正下：低中高各 2 發（按住）
- [ ] 右搖桿左上（對齊）：`LunaAuto/AlignState` 跑到 DONE，停下時「砲台Luna」= `穿過洞`
- [ ] 洞後面有黑箱子的話：看 `TurretLidar/MinusBaselineMeters` 對著洞的值，記下來
- **失敗時**：哪個方向不對就比賽不用那個；對齊不行 → 「對齊方式」改 LaserAlign 或人工對

**Assist 射 3 發就停**（有時間才查，10 分鐘）：記下 `Assist/FireCount`、`Storage/ShotsDetected`、`Turret/TriggerVelocityRpm` 最高值 → **比賽不用 Assist，用 Auto 模式**

---

## 1:25–1:45　比賽演練 ×2 **P0 不能砍**
照「操作卡」的比賽流程跑 2 次，計時：
- 90 秒準備內完成選 Auto＋READY 燈亮
- 自動流程跑完（時間：____ s／____ s）
- 換 Auto 模式射完剩下的球
- 記：哪一步出錯、READY 燈有沒有誤報

---

## 1:45–2:00　凍結 **P0**
1. 在 Elastic 改過的值**全部寫回程式**（重開機會消失）：`StartYawToWallDeg`、`BlindApproachMeters`、`ShootDistanceMeters`、`HoleMarginMeters`、`ShotDetectRpm`、`Luna 前進距離`…
2. `.\gradlew build` → `.\gradlew deploy` → `git commit` + push
3. **之後不再改程式**
4. 換上充好的電池，確認 READY 燈

---

## 結果表
| 項目 | 結果 | 備註 |
|---|---|---|
| 部署＋基本檢查 | | 紅燈： |
| 自動 1 | | 停在哪個 Stage／幾發進 |
| 自動 2 | | |
| 自動 3 | | |
| 手動射擊（左下／右下／右上／正下） | | |
| 右搖桿對齊 | | |
| 洞後箱子 MinusBaseline | | ____ m |
| 演練 1／2 | | 自動時間： |
| 寫回程式的值 | | |
