# roboRIO → SystemCore 升級紀錄

本文件記錄將 TEL2026 機器人程式從舊版 roboRIO 移植到 SystemCore（WPILib 2027 alpha）所做的所有修改。

- 修改日期：2026-11-07
- 驗證狀態：`./gradlew build` 通過（原本約 200 個編譯錯誤）
- 已部署到實體 SystemCore，底盤與砲台在實機上可正常運作（2026-11-07）
- 模擬器可正常啟動，FlySky→MockDS enable、底盤直走/轉向已用 `sim_smoke_test.py` 自動測試（2026-11-10），見「模擬器」一節
- 環境：GradleRIO `2027.0.0-alpha-8`、Java 25、WPILib `2027_alpha7`

專案的 `build.gradle` 與 vendordeps 原本就已設定為 SystemCore（GradleRIO 2027），問題主要出在 Java 程式碼與第三方函式庫。

---

## I/O 埠重新分配

SystemCore 只有 **6 個 Smart I/O 埠（0–5）**（已從 HAL 原始碼確認：`NUM_SMART_IO = 6`，實機與模擬器相同），roboRIO 時代的配置需要 12 個，因此重新分配如下：

| 用途 | 原 roboRIO 埠 | 新配置 | 檔案 |
|---|---|---|---|
| Storage 方向控制 | DIO 0、1、2、3 | **Smart I/O 0、1、2、3** | `subsystems/storage/StorageConstants.java` |
| Storage / Trigger 馬達（PWM Spark） | PWM 8、9 | **Smart I/O 4、5** | `subsystems/storage/StorageConstants.java` |
| FlySky 遙控接收機 | DIO 4、5、6、7、8、9（PWM 訊號） | **iBus 經 USB 轉 TTL 模組接 SystemCore USB**（2026-11-13 起；Jetson + NT 保留為備援） | `controller/FlySkyController.java`、`controller/IBusSerialReader.java` |
| 兩顆 TF-Luna 測距 | 協處理器，經 NetworkTables | **直接接 SystemCore I2C 0（左）、I2C 1（右）** | `FSLib/rangefinder/RangeSensorConstants.java` |

腳位對照：DIO 0→0、DIO 1→1、DIO 2→2、DIO 3→3、PWM 8→4、PWM 9→5。**實體接線也要跟著改。**

---

## FlySky 遙控器：USB 轉 TTL 直接接 SystemCore（2026-11-13）

```
FlySky 接收機 --iBus(UART 115200)--> USB-TTL 模組 --USB--> SystemCore（/dev/ttyUSB0）
```

- 接線：接收機 i-BUS 訊號腳 → USB-TTL **RX**；GND 共地；接收機 5V 接 USB-TTL 的 5V。USB-TTL 的 TX 不用接。
- **WPILib 2027 alpha-8 的 `SerialPort` 在 SystemCore 上沒實作**（HAL 的 `systemcore/SerialPort.cpp` 全部回傳 `HAL_HANDLE_ERROR`），
  所以 `IBusSerialReader` 用 `stty -F <port> 115200 cs8 -cstopb -parenb raw -echo clocal ... min 1 time 0` 設定序列埠，
  再用 `FileInputStream` 在背景執行緒直接讀。依序嘗試 `FlySkyController.kSerialPorts`（`/dev/ttyUSB0`、`/dev/ttyUSB1`、`/dev/ttyACM0`）。
- USB 被拔掉或讀取失敗會每 1 秒重新開啟；同樣的錯誤只印一次到 console（`[FlySky] ...`）。
- **來源優先順序**：USB-TTL 0.1 秒內有正確封包 → 用 USB-TTL；否則改讀 Jetson 的 NT（下方舊架構）；兩個都沒有 → 全部歸零（failsafe 不變）。
  模擬器（`RobotBase.isReal() == false`）不開序列埠，只讀 NT，所以 `sim_smoke_test.py` 照常可用。
- **不要兩邊同時接同一台接收機的訊號**也沒關係（USB-TTL 優先），但換回 USB-TTL 後 Jetson 的 `flysky` 服務可以停掉：`sudo systemctl disable --now flysky`。
- Log（AdvantageKit）：`FlySky/Source`（`kSerial`／`kNetworkTables`／`kNone`）、`FlySky/ChannelsUs`、
  `FlySky/Serial/Port`、`FlySky/Serial/Frames`、`FlySky/Serial/BadFrames`（一直增加 → 接觸不良或鮑率錯）。
- ⚠️ 實機待確認：`stty` 存在且機器人程式的使用者有 `/dev/ttyUSB0` 的讀取權限（沒有的話 console 會印 `stty 失敗（權限不足？）`，
  用 `ls -l /dev/ttyUSB0` 看群組，把執行機器人程式的使用者加進 `dialout` 群組）。
- `IBusParserTest`：單一封包、垃圾資料後重新同步、零碎讀取、checksum 錯誤。

## FlySky 遙控器（備援）：Jetson + NetworkTables

### 架構

```
FlySky 接收機 --iBus(UART 115200)--> Jetson --NetworkTables--> SystemCore
```

### NetworkTables 格式（Jetson 必須照這個發布）

| Topic | 型別 | 內容 |
|---|---|---|
| `/FlySky/channels` | `double[]` | iBus 各通道脈寬（μs，約 1000~2000），index 0 = CH1 |
| `/FlySky/heartbeat` | `int` | 每收到一個 iBus 封包就 +1 |

- 機器人端用 `(值 - 1500) / 500` 正規化到 -1.0 ~ 1.0，與舊版 PWM 讀法完全相同。
- **斷線保護**：超過 `kTimeoutSeconds`（0.1 秒）heartbeat 沒有變化，所有搖桿回傳 0、開關回傳 false。
  （舊版沒訊號時搖桿會回傳 -1，等於全速；新版改為歸零比較安全。）
- 用 heartbeat 而不是直接看 `channels` 的更新時間，是因為 NetworkTables 不會重送相同的值，
  搖桿不動時 `channels` 就不會更新，會被誤判成斷線。

### 通道對應（`FlySkyController.IBusChannel`）

已依實際遙控器確認（2026-11-07）：

| 功能 | iBus 通道 | index |
|---|---|---|
| 左搖桿 X | CH1 | 0 |
| 左搖桿 Y | CH2 | 1 |
| 右搖桿 Y | CH3 | 2 |
| 右搖桿 X | CH4 | 3 |
| 左開關 | CH5 | 4 |
| 右開關 | CH6 | 5 |

也可以用 `new CommandFlySkyController(leftX, leftY, rightX, rightY, leftSwitch, rightSwitch)` 指定通道。

### Jetson 端範例（Python，需要 `pyserial`、`pyntcore`）

```python
import ntcore
import serial

SERIAL_PORT = "/dev/ttyTHS1"     # 接收機接到的 UART，依實際接線修改
SYSTEMCORE_IP = "10.71.30.2"     # SystemCore 的 IP，依實際網路修改

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("jetson-flysky")
inst.setServer(SYSTEMCORE_IP)

table = inst.getTable("FlySky")
channels_pub = table.getDoubleArrayTopic("channels").publish()
heartbeat_pub = table.getIntegerTopic("heartbeat").publish()

ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.1)
buf = bytearray()
heartbeat = 0

while True:
    buf += ser.read(64)
    while len(buf) >= 32:
        # iBus 封包：0x20 0x40 + 14 通道 (uint16 little-endian) + checksum
        if buf[0] != 0x20 or buf[1] != 0x40:
            del buf[0]
            continue
        frame = bytes(buf[:32])
        del buf[:32]
        checksum = 0xFFFF - sum(frame[:30])
        if checksum != (frame[30] | (frame[31] << 8)):
            continue
        channels = [frame[2 + 2 * i] | (frame[3 + 2 * i] << 8) for i in range(14)]
        channels_pub.set([float(c) for c in channels])
        heartbeat += 1
        heartbeat_pub.set(heartbeat)
        inst.flush()
```

> 注意：遙控器訊號現在要經過 Jetson 和網路，Jetson 當機、網路斷線都會讓遙控失效（機器人會因斷線保護而停住）。
> 賽前請測試延遲與拔線後的行為。

---

## 1. `build.gradle`

### 新增相依套件

```groovy
// AdvantageKit: read version from vendordep so @AutoLog generator matches the library
def akitJson = new groovy.json.JsonSlurper().parseText(new File(projectDir.getAbsolutePath() + "/vendordeps/AdvantageKit.json").text)

dependencies {
    ...
    // AdvantageKit @AutoLog annotation processor (generates *AutoLogged classes)
    annotationProcessor "org.littletonrobotics.akit:akit-autolog:$akitJson.version"

    // WPILib 2027 no longer bundles Jackson; LimelightHelpers needs it
    implementation 'com.fasterxml.jackson.core:jackson-databind:2.15.2'

    // Lombok (used by FSLib SwerveModuleConstants @Builder)
    compileOnly 'org.projectlombok:lombok:1.18.42'
    annotationProcessor 'org.projectlombok:lombok:1.18.42'
}
```

| 套件 | 原因 |
|---|---|
| `akit-autolog` | 之前缺少 AdvantageKit 的註解處理器，`DriveIOInputsAutoLogged`、`GyroIOInputsAutoLogged`、`TurretIOInputsAutoLogged` 等類別沒有被產生 |
| `jackson-databind` | WPILib 2027 不再內附 Jackson，`LimelightHelpers` 需要 |
| `lombok` | `FSLib/swerve/SwerveModuleConstants` 使用 `@Builder` |

### 排除 MapleSim 相關程式

```groovy
// MapleSim has no 2027 / SystemCore build yet, so its game-piece sim classes are excluded.
// Re-enable once a MapleSim vendordep for 2027 is available.
sourceSets {
    main {
        java {
            exclude 'frc/robot/gamepiece/**'
        }
    }
}
```

`robot/gamepiece/CargoOnField.java`、`CargoOnFly.java` 依賴 MapleSim，而 MapleSim 目前沒有 2027 版本。檔案保留未刪除，等 MapleSim 出 2027 版後移除此 `exclude` 即可恢復。

---

## 2. 無 2027 版本的函式庫

| 函式庫 | 處理方式 |
|---|---|
| NavX（Studica） | SystemCore 沒有 MXP 擴充埠，且 Studica 沒有 2027 版 → 改用 SystemCore 內建 IMU |
| URCL | 沒有 2027 版 → 移除 `Logger.registerURCL(URCL.startExternal())` |
| MapleSim | 沒有 2027 版 → 排除 `gamepiece` 套件，移除 `SimulatedArena` 呼叫 |

---

## 3. 逐檔修改

### `robot/Constants.java`
- 新增 SystemCore CAN 匯流排設定：
  ```java
  public static final CANPort kCANPort = CANPort.CAN_S0;
  ```
  SystemCore 有 5 個 CAN 埠（`CAN_S0` ~ `CAN_S4`），**請改成馬達實際接線的那一個**。所有 CAN 裝置都使用此常數。

### `robot/Robot.java`
- 移除 `org.ironmaple.simulation.SimulatedArena`、`org.littletonrobotics.urcl.URCL`、`Pose3d` 的 import
- 移除 `Logger.registerURCL(URCL.startExternal());`（留下註解說明）
- `m_autonomousCommand.schedule()` → `CommandScheduler.getInstance().schedule(m_autonomousCommand)`
- `simulationPeriodic()` 中的 MapleSim 遊戲物件模擬移除（留下註解）

### `robot/RobotContainer.java`
- `GyroIONavX` → `GyroIOOnboardIMU`
- `SmartDashboard.putString(...)` → `Logger.recordOutput(...)`

### `robot/MockDS.java`
- `SmartDashboard.putString(...)` → `Logger.recordOutput(...)`

### `robot/commands/drive/Align.java`
- `SmartDashboard.putNumber(...)` → `Logger.recordOutput(...)`

### `FSLib/ros/laser/RPLidarA1/LaserScanMath.java`
- `SmartDashboard.putNumber(...)` → `Logger.recordOutput(...)`

> 註：WPILib 2027 移除了 `SmartDashboard`。這些數值現在發布在 AdvantageKit 的輸出 topic（`/AdvantageKit/RealOutputs/...`），不再出現在 SmartDashboard。

### `FSLib/statemachine/StateMachine.java`
- 所有 `m_xxxCommand.schedule()` → `CommandScheduler.getInstance().schedule(m_xxxCommand)`

### `FSLib/statemachine/StateSubsystemBase.java`
- WPILib 2027 移除了 `Sendable` / `SendableBuilder` / `SendableRegistry`
- `implements Subsystem, Sendable` → `implements Subsystem, TelemetryLoggable`
- 移除 `SendableRegistry.add(this, name)`
- `initSendable(SendableBuilder)` → `logTo(TelemetryTable)`，並新增 `getTelemetryType()` 回傳 `"StateSubsystem"`

### `robot/commands/drive/LaserAlign.java`
- `DifferentialDrive.WheelSpeeds` → `DifferentialDrive.WheelVelocities`

### `robot/commands/turret/TurretTest.java`
- `XboxController.getPOV()` 已移除，改用：
  - `getPOV() == 0` → `getDpadUpButton()`
  - `getPOV() == 180` → `getDpadDownButton()`
  - `getPOV() == 270` → `getDpadLeftButton()`

### `robot/controller/PWMInput.java` → 刪除
- 原本用 `Counter` 量測接收機 PWM 脈寬（WPILib 2027 已移除 `Counter`）
- 接收機改接 Jetson 後不再需要，已刪除

### `robot/controller/FlySkyController.java`
- 改為從 NetworkTables `/FlySky/channels`、`/FlySky/heartbeat` 讀取（格式見上方「FlySky 遙控器」章節）
- 新增 `IBusChannel` 通道對應常數、`isConnected()`、斷線時歸零的保護
- 建構子參數從「DIO 埠號」改為「iBus 通道 index」
- 對外介面（`getLeftX()`、`getLeftSwitch()` 等）不變，`RobotContainer` 不需修改

### `robot/controller/CommandFlySkyController.java`
- 自訂建構子的參數改為 iBus 通道 index
- 2026-11-13：建構時把 `FlySkyController.logTelemetry()` 綁到預設 EventLoop，每個 loop 記一次 FlySky log

### `robot/controller/IBusSerialReader.java`（新增，2026-11-13）
- 在 SystemCore 上直接讀 USB-TTL 的 i-BUS，詳見「FlySky 遙控器：USB 轉 TTL」章節
- `FlySkyController`：實機建立 `IBusSerialReader`；新增 `getSource()`、`getChannelsMicroseconds()`、`logTelemetry()`；`isConnected()` 改為任一來源有資料即可

### `jetson_tools/flysky_nt_publisher.py`（Jetson 端）
- 修正：`/FlySky/channels` 原本發布已正規化的 -1~1，與機器人端（期望 μs，自己再正規化）不符，
  會導致所有搖桿卡在 -1、開關永遠 false。改為發布 μs 原始值。
- `--channels` 限制 6~14（機器人讀 CH1~CH6）
- 未指定 `--server` 時改為依序嘗試 `10.TE.AM.2`、`robot.local`。舊版 pyntcore 的 `setServerTeam` 找不到 SystemCore，`nt_test.py` 也一起改了。
- NT 連線／斷線時會印出狀態，可以用 `journalctl -u flysky -f` 確認
- `jetson_tools/README.md` 補上 `flysky_nt_publisher.py`、`flysky.service` 的說明與安裝步驟

### `FSLib/rangefinder/DualTFLunaRangeSensor.java`（TF-Luna 改接 I2C）
- 原本從 NT 表 `TFLuna` 讀取協處理器發布的資料，改為用 `org.wpilib.hardware.bus.I2C` 直接讀取
- 每顆讀暫存器 0x00 起 6 bytes：距離（cm）、訊號強度、溫度；I2C 傳輸失敗就視為該顆無效
- 同一個控制週期內只讀一次 I2C（5 ms 快取），多個 command 同時呼叫不會重複讀
- 距離／偏航計算公式、有效值判斷、AdvantageKit log 路徑（`RangeSensor/...`）維持不變；新增 `LeftConnected`、`LeftStrength`、`LeftTemperatureC` 等 log
- `RangeSensorConstants`：刪除 `kTFLunaTable`、`kStaleTimeoutSec`，新增 `kLeft/RightTFLunaPort`、`kLeft/RightTFLunaAddress`
- 硬體：TF-Luna 第 5 腳（CFG）要接 GND 才會進 I2C 模式；兩顆接同一個埠時要先改其中一顆的位址

### `robot/subsystems/drive/Drive.java`
- `DifferentialDriveWheelSpeeds` → `DifferentialDriveWheelVelocities`
- `kinematics.toWheelSpeeds(...)` → `kinematics.toWheelVelocities(...)`
- `wheelSpeeds.leftMetersPerSecond / rightMetersPerSecond` → `wheelVelocities.left / right`
- `Rotation2d.kZero` → `Rotation2d.ZERO`
- `odometry.getPoseMeters()` → `odometry.getPose()`
- 移除重複與未使用的 import（`RobotState`、`MatchType`、`DriverStationErrors`、重複的 `Alliance`）

### `robot/subsystems/drive/DriveIOSim.java`
`DifferentialDrivetrainSim` 的方法去掉了單位後綴：

| 舊 | 新 |
|---|---|
| `getLeftPositionMeters()` | `getLeftPosition()` |
| `getRightPositionMeters()` | `getRightPosition()` |
| `getLeftVelocityMetersPerSecond()` | `getLeftVelocity()` |
| `getRightVelocityMetersPerSecond()` | `getRightVelocity()` |
| `getLeftCurrentDrawAmps()` | `getLeftCurrentDraw()` |
| `getRightCurrentDrawAmps()` | `getRightCurrentDraw()` |

### `robot/subsystems/drive/DriveIOSpark.java`（REVLib 2027，⚠️ 請實機驗證）
- import 變更：
  - `SparkBase.ControlType` → `SparkLowLevel.ControlType`
  - `SparkBase.PersistMode` → `com.revrobotics.PersistMode`
  - `SparkBase.ResetMode` → `com.revrobotics.ResetMode`
- 建構子：`new SparkMax(id, type)` → `new SparkMax(Constants.kCANPort, id, type)`
- **REVLib 2027 移除了 `positionConversionFactor` / `velocityConversionFactor`**，編碼器固定回傳馬達「圈」與「RPM」：
  - 刪除 `encoder.positionConversionFactor(...).velocityConversionFactor(...)` 設定
  - 讀值時在程式中換算：
    ```java
    inputs.leftPositionMeters = leftEncoder.getPosition().get() * kPositionConversionFactor;
    inputs.leftVelocityMetersPerSec = leftEncoder.getVelocity().get() * kVelocityConversionFactor;
    ```
  - 速度設定點改成 RPM：`metersPerSec / kVelocityConversionFactor`
  - PID 增益乘上 `kVelocityConversionFactor`，讓誤差單位從 m/s 換成 RPM 後數學上行為不變：
    ```java
    .pid(kP * kVelocityConversionFactor, kI * kVelocityConversionFactor, kD * kVelocityConversionFactor);
    ```
- `setReference(...)` → `setSetpoint(...)`
- `getAppliedOutput()`、`getBusVoltage()`、`getOutputCurrent()`、`getPosition()`、`getVelocity()` 現在回傳 `Signal<Double>`，需加 `.get()`

### `robot/subsystems/drive/GyroIONavX.java` → 刪除，改為新檔 `GyroIOOnboardIMU.java`
```java
private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

inputs.connected = true;
inputs.yawRad = imu.getYawRadians();
inputs.yawVelocityRadPerSec = imu.getGyroRateZ();
```
- 內建 IMU 遵循 WPILib 慣例（逆時針為正），所以不用像 NavX 那樣取負號
- 若 SystemCore 不是平放安裝，請把 `MountOrientation.FLAT` 改成 `LANDSCAPE` 或 `PORTRAIT`

### `robot/subsystems/drive/LocalADStarAK.java`
- `org.wpilib.math.util.Pair` → `org.wpilib.util.Pair`

### `robot/subsystems/turret/TurretReal.java`
- 新增 `CANBus canBus = new CANBus(Constants.kCANPort)`
- Phoenix 6：`new TalonFX(id)` → `new TalonFX(id, canBus)`；`new CANcoder(id)` → `new CANcoder(id, canBus)`
- REVLib：`new SparkMax(id, type)` → `new SparkMax(Constants.kCANPort, id, type)`
- `PersistMode` / `ResetMode` import 改為 `com.revrobotics.*`
- MAXMotion 設定：`maxVelocity(...)` → `cruiseVelocity(...)`、`allowedClosedLoopError(...)` → `allowedProfileError(...)`
- `getAppliedOutput()`、`getBusVoltage()`、`getOutputCurrent()` 加 `.get()`
- **射球 TalonFX 不轉的排查**（2026-11-07）：實測只有仰角 SparkMax 會動，上下兩顆 TalonFX 不會轉
  - 原因是 TalonFX 的 Phoenix 韌體太舊，跟 2027 版 Phoenix 6 函式庫不相容，不是程式的問題。更新韌體後射球馬達就正常了。
  - 排查期間加的 FOC 開關（`kShooterUseFOC`）和診斷 log（`Turret/up*`、`Turret/down*`）已移除。隊上有 Phoenix Pro 授權，所以射球馬達維持原本的 `withEnableFOC(true)`。

### `robot/subsystems/storage/Stroage.java`
- PWM `Spark.set(speed)` → `Spark.setThrottle(speed)`
- **改為直接用 `PWM` 驅動 L298N 的 ENA / ENB**（2026-11-07）：
  - `Spark` 送的是伺服馬達用的脈衝，在 roboRIO（週期約 5 ms）上速度 1 約等於 40% 出力。SystemCore 的 PWM 預設週期是 20 ms，只剩約 10%。
  - 現在週期設為 5 ms，速度 1 送 2000 μs 的高電位，維持約 40% 出力。速度 0 會完全關閉，以前是約 30%。
  - `disableStorage()` / `disableTrigger()` 也會關閉 PWM。機器人 disable 時，`periodic()` 會自動停止兩顆馬達。
  - 接線：ENA←Smart I/O 4、IN1←0、IN2←1；ENB←Smart I/O 5、IN3←2、IN4←3。L298N 的 ENA / ENB 跳線帽要拔掉，並且要和 SystemCore 共地。

### `robot/subsystems/storage/StorageConstants.java`
- 腳位改為 SystemCore Smart I/O 0~5：
  - `kStorageReversePort` = 0、`kStorageForwardPort` = 1、`kTriggerReversePort` = 2、`kTriggerForwardPort` = 3（DIO 編號不變）
  - `kStorageSpeedPort`：PWM 8 → Smart I/O 4
  - `kTriggerSpeedPort`：PWM 9 → Smart I/O 5

---

## 九宮格視覺（YOLO11）（2026-11-07 新增）

YOLO 只負責**水平對準**九宮格的某一欄；牆面角度仍由 TF-Luna（`LaserAlign`）處理。

### 架構
```
攝影機 → Jetson：vision_nt_publisher.py（YOLO11 + 去畸變 + 亮燈判斷）--NT /Vision--> SystemCore：Vision → VisionAlign
```
- YOLO 類別：`0 hole`（Ø40 cm 大洞）、`1 bonus`（Ø20 cm 加分洞）。
- **亮燈判斷不靠 YOLO**：YOLO 找出加分洞後，Jetson 計算洞外一圈的紅色像素比例（`litScore`）。這樣不需要亮燈的訓練資料，門檻在機器人端調。
- NT 格式寫在 `VisionConstants.java` 開頭，Python 端要一致：`targets`（每個偵測 6 個數字）、`heartbeat`、`latencyMs`、`fps`。

### 機器人端新增 / 刪除
- `subsystems/vision/`：`VisionConstants`、`VisionIO`、`VisionIONT`（實機）、`VisionIOSim`（模擬：依 pose 算出看到的洞）、`Vision`（分欄、找亮燈欄）
- `commands/drive/VisionAlign.java`：目標可選 `LIT`、`LEFT`、`MID`、`RIGHT`、`NEAREST`。先鎖定一欄，之後用里程計補償並追蹤同一欄，以 P 控制沿牆直走。連續 3 張畫面誤差 ≤ 2 cm 就結束。
- PathPlanner named commands：`VisionAlignLIT`、`VisionAlignLEFT`、`VisionAlignMID`、`VisionAlignRIGHT`、`VisionAlignNEAREST`。按鈕綁定沒有改。
- `commands/drive/Align.java` → 刪除。舊版讀 `/Vision/poses`（像素），沒有被用到，已由 `VisionAlign` 取代。
- 現場可調參數（NT `/Tuning/...`，不用重新部署）：`Vision/MinConfidence`、`Vision/LitThreshold`、`VisionAlign/kP`、`MaxSpeedMps`、`MinSpeedMps`、`ToleranceMeters`
- 單元測試：`src/test/java/frc/robot/subsystems/vision/VisionTest.java`（分欄邏輯）

### ⚠️ 11/14 實機必須確認
1. `VisionConstants.kImageRightIsRobotForward`：機器人往前開時，目標應該往畫面右邊移動（預設 false）。方向錯的話，對準時會越開越遠。
2. `kShooterForwardOfCameraMeters`（預設 0.15 m）：對準後實際射一顆，偏哪邊就調這個值。
3. `Vision/LitThreshold`：用 LED 燈條實測。

### Jetson 端新增（`jetson_tools/`）
`synth_dataset.py`、`vision_nt_publisher.py`、`vision_fake_publisher.py`、`extract_frames.py`、`autolabel.py`、`train.py`、`vision.service`、`assets/nine_grid_design.jpg`（官方設計圖縮小版）。用法見 `jetson_tools/README.md`。

## 模擬器（2026-11-10 新增）

- **啟動指令改為 `./gradlew run`**：2027 版 GradleRIO 已移除 `simulateJava`（會報 `The simulateJava task has been removed. Use run instead.`）。
- **`build.gradle`**：`wpi.sim.addDriverstation().defaultEnabled = true`，預設載入 `halsim_ds_socket`。機器人靠 `MockDS` 送 UDP 到 127.0.0.1:1110 來 enable，模擬器沒載入這個擴充就收不到。
- **`DriveIOSim.java`**：disable 時把電壓歸零並重設 PID（用 `RobotState.isDisabled()`；2027 的 `DriverStation` 沒有 `isDisabled()`）。原本閉迴路會沿用最後的目標速度，disable 後模擬底盤還會繼續跑。實機由 HAL 切斷輸出，不受影響。
- **新增 `jetson_tools/sim_smoke_test.py`**：在筆電上假裝 Jetson 送 FlySky 資料，自動測 enable、直走、原地轉、Jetson 斷線保護。先開模擬器，再 `python sim_smoke_test.py`。
- ⚠️ 假 FlySky 發布端一定要每次 `inst.flush()`：pyntcore 預設 100 ms 才送一次，等於機器人端 `FlySkyController.kTimeoutSeconds = 0.1`，會被判斷斷線，左開關變 false → `MockDS` 停止又重開 → 序號歸零 → DS socket 擋掉封包（`Blocking colliding packet`）→ enable 閃斷。實機的 `flysky_nt_publisher.py` 已有 flush。
- **`MockDS.java`**：封包序號改成欄位 `m_sendCount`，`stop()` 後再 `start()` 會接續序號；`start()` 開頭先 `stop()`，避免兩個執行緒同時送。原本每次重開序號歸零，DS socket 會擋掉序號倒退的封包，FlySky 短暫斷線後要多 disable 一段時間。斷線保護不變（左開關 false 仍會停止 MockDS → disable）。模擬驗證：斷線恢復後 0.6 秒重新 enable，log 無 `Blocking colliding packet`（2026-11-10）。⚠️ 下次實機時確認一次 FlySky 開關 enable/disable 正常。

- **`VisionIOSim.java`**：新增建構子 `VisionIOSim(pose, litColumn, seed)`，可指定亮燈欄與亂數種子（測試用），原本的建構子不變。
- **新增 `src/test/.../commands/drive/VisionAlignSimTest.java`**：VisionAlign 閉迴路模擬（DriveIOSim + VisionIOSim，時間步長要用 `Constants.kDefaultPeriod` = 10 ms，和 DriveIOSim 一致）。
  - LEFT/MID/RIGHT 各 3 個起點、LIT 3 欄：12/12 通過，最後誤差 ±1.8 cm 以內。
  - `VA_SWEEP=1 ./gradlew test --tests "*VisionAlignSimTest.sweep" -i` 印出敏感度掃描（2026-11-10 結果）：
    - 距離不準（真實 2.0–4.0 m、假設 3.0 m）：誤差 +5.8 ~ −2.4 cm，影響小（只透過 `kShooterForwardOfCameraMeters` 放大）。
    - **車身沒擺正影響很小**（更正）：砲台只有俯仰、射擊方向和攝影機平行，車身偏 10° 時球落點誤差仍在 2.1 cm 內。先前「偏 2° → 10 cm」是量錯指標（量射球點沿牆位置，而非射擊線打到九宮格平面的落點），測試已改用落點 `hitError`。
    - ⚠️ **移動距離上限約 1 m**：`kMaxSpeed` 0.25 m/s + `kTotalTimeoutSec` 5 s，要移動超過約 1.05 m 會逾時；起點太遠看不到可辨識的欄位時 1 秒就放棄。
- **新增 `src/test/.../AutoTimingSimTest.java`**：量自動流程每段時間（2026-11-10，模擬，飛輪瞬間到速）：RS_C2_Auto 3.5 s、VisionAlign 中→左 5.0 s（⚠️ 逾時，沒對準，落點差約 14 cm）、中→左以外的換欄 3.8–4.5 s、每發 ColumnShot 1.2 s（實機另加飛輪加速；`ColumnShot.end()` 會停飛輪，每段都要重新加速）。合計 24 s。
  - 模擬限制：`VisionIOSim` 沒有模擬延遲（看到的是當下 pose），模擬讀不到 TF-Luna 所以 `LaserAlign` 不會動作（路徑結束時車頭 81°）。
- **`VisionIOSim.java` 加入延遲**：偵測結果改用 `latencySeconds`（預設 60 ms）之前的 pose 產生，`latencyMs` 也回報同一個值。
- **速度掃描**（`VA_SPEED=1 ./gradlew test --tests "*VisionAlignSimTest.speedSweep" -i`，2026-11-10）：`kMaxSpeed` 0.25–1.0 m/s、延遲 60/120 ms，落點誤差全部在 ±1.5 cm 內；移動 1 m 的時間 0.25 m/s 為 4.7 s、0.5 m/s 為 3.1 s、1.0 m/s 為 2.5 s。模擬沒有動態模糊與漏偵測，實機速度以 0.5 m/s 起步驗證。

## 自動策略與流程（2026-11-10 新增）

**策略**（依 `2026_rule_0903.pdf`）：自動只做手動做不到的事。加分格只在前 60 秒算分，且同分評比第一項就是加分格，所以自動一定要先打加分格；之後留在同一欄把低中高打完，不換欄（避免和對手相撞），換欄交給手動。每一發前都用 `VisionAlign NEAREST` 重新對準（已對準時約 0.1 秒），被撞歪也會修回來。
- 上半場（`Half.FIRST`）：加分格 3 顆（12 分）+ 同欄每洞 3 顆（18 分）= 12 發 30 分。
- 下半場（`Half.SECOND`）：加分格 2 顆（8 分）+ 同欄每洞 2 顆（12 分 + 直線 4 分）= 8 發 24 分。
- 等 2 秒（`AutoRoutines.kLitWaitSeconds`）還看不到亮燈：不打加分格，改對中欄打大洞。

**新增 `commands/AutoRoutines.java`**：`bonusThenColumn(drive, vision, turret, half, startAuto)`。各階段寫在 `Auto/Stage`。

**`RobotContainer.java`**
- 新增 `LoggedNetworkChooser` 選單 `Auto Choices`（2027 AdvantageKit 的 `LoggedDashboardChooser` 已改名）：上半場/下半場 × 右/左起點，預設「上半場-右起點」。選單存 `Supplier<Command>`，每次啟動都產生新指令。
- ⚠️ 自動啟動（左開關 + 右搖桿左上 + `kDisable`）由 `whileTrue` 改成 `onTrue`：規則規定啟動後不能碰遙控器，原本搖桿一放開自動就會被取消。左開關關掉時一併取消。
- ⚠️ `LaserAlign` 的右搖桿左上綁定加上 `isDisable().negate()`：原本在 `kDisable` 模式下會和自動同時觸發、兩個都要 drive 而互相打斷。
- `getAutonomousCommand()` 改回傳選單的指令（目前用 MockDS teleop 啟動，不會走到這裡）。

**`VisionAlign.java`**：`MaxSpeedMps` 預設 0.25 → 0.5 m/s（模擬含延遲驗證過；NT 可調）。

**新增 `src/test/.../AutoRoutineSimTest.java`**：完整自動流程模擬，攔截砲台扳機記錄每一發的射擊線落點。8 個情境全過（亮燈左/中/右/沒亮、上下半場、射到一半被推 ±30 cm）：上半場 21.7–23.5 s、下半場 16.4–19.7 s，落點誤差 ≤ 2.7 cm；被推之後下一發前就修回來（多花約 1.5–3 s）。時間不含飛輪加速（每發要重新加速），即使每發加 2 秒也在 50 秒內。

**依命中率調整發數**（2026-11-10）：`AutoRoutines.Half` 的發數改成 NT 可調 `/Tuning/Auto/<FIRST|SECOND>/BonusBalls`、`/BallsPerHole`（預設為規則上限 3/3、2/2；限制 0–10），自動流程啟動時才讀取，比賽前 90 秒準備時間改了就生效；實際使用的值記在 `Auto/BonusBalls`、`Auto/BallsPerHole`。建議值用 `jetson_tools/strategy_calc.py --bonus 進/射 --hole 進/射` 算（期望值比較：第 k 發加分格的邊際期望 vs 留給大洞的 2×命中率；預設至少 1 發保底同分評比）。不做比賽中即時調整：沒有進球偵測，手動階段由操作員看。注意加分格發數加多時，若飛輪加速很慢，自動可能來不及打完大洞（加分格先打，大洞可交給手動）。

**⚠️ 11/14 實機要確認**：飛輪加速時間、各高度與加分格命中率、自動啟動流程（筆電選 Auto → 左開關 → 右搖桿左上撥一下放開）。

## Elastic 記分板（2026-11-10 新增）

版面檔 `src/main/deploy/elastic-layout.json`（grid 32，配合 14 吋 1920×1080、縮放 150% → Elastic 可用約 1280×570，整個版面限制在 1280×512 內）：
- 分頁「記分」（比賽中）：左邊 4×3 格（加分、高、中、低 × 左、中、右，從操作員視角），每格大字顯示 `●●○`（實心 = 已記、空心 = 還能算分，超過上限顯示 `+n`，亮燈加分格前面有 ★），旁邊是並排的高 + 和 −（各 64×128），欄與欄之間留 32 空隙防誤按。右邊：預估分數、半場、未歸類 + 清、Auto 階段、Jetson。
- 分頁「賽前」：Auto 選擇、半場、Jetson 視覺、兩個半場的加分格發數（可直接改）、全部歸零（放這裡避免比賽中誤按）。
- 2026-11-10 改版原因：原本單頁 1600×704 超出螢幕，+／− 是 64×64 上下疊，扣掉標題列幾乎按不到。
- 載入：部署後 Elastic → File → Load Layout from Robot（`Robot.java` 新增 `WebServer.start(5800, deploy 資料夾)`）；或直接開這個 json 檔。

**資料來源：射擊數 + 手動修正**（機器人沒有進球偵測）
- 新增 `subsystems/scoreboard/Scoreboard.java`：ColumnShot 射出的球自動 +1（列 = 瞄準高度，欄 = 視覺 `getAimedColumn`；加分格用記住的亮燈欄）。手動 / Assist 射擊或分辨不出欄位 → 記在「未歸類」，按該格的 + 會歸過去（未歸類 −1）。沒進的球按 −。自動流程啟動時自動歸零。
- 預估分數依規則計算（下半場含連線分），假設記到的都有進。
- `Turret.java`：`setTrigger` 偵測扳機斷電→通電 = 射出一發（`getShotCount`、`getLastShotAimHeight`、`setAimHeight`）。原本的 `ballFired()`（扳機電壓 < 3V）大部分時間都成立，沒拿來計數。
- `ColumnShot.java`：開始時 `turret.setAimHeight(目標高度)`，結束時設回 NaN。
- `Vision.java`：新增 `getAimedColumn`、`getLitColumn`。
- `RobotContainer.java`：Auto 選單值改成 `AutoOption(half, startAuto)`，記分板用它決定每格上限（上半場 3、下半場 2）。
- 2027 的 `LoggedNetworkChooser` NT 格式是 `/Auto Choices/{.type=Selectable, default, options, selected/value}`，機器人從 `selected/tune` 讀選擇（不是舊 SendableChooser 的 `selected`／`active`）。2026-11-10 用模擬器確認 Elastic 2027 alpha7 的 ComboBox Chooser 會寫 `selected/tune`，機器人有跟著變。
- `RobotContainer.java`：開機時 `Auto/Stage` 記成 `Idle`，不然沒跑過自動前 Elastic「Auto 階段」是空的。
- ⚠️ 規則：筆電只能由操作遙控器的那位選手操作，比賽中按 +／− 要由他來按。2026-11-10 已用模擬器 + Elastic 開過，+ 按鈕有計數。

## 視覺 NT 端到端測試（2026-11-12）

`vision_fake_publisher.py` → 模擬器機器人（`VisionIONT`），用 `jetson_tools/vision_nt_e2e_test.py` 測，10/10 通過。

- `RobotContainer`：模擬器設環境變數 `VISION_IO=nt` 就改用 `VisionIONT`（讀 NT），不設則用 `VisionIOSim`。
  ```powershell
  $env:VISION_IO="nt"; ./gradlew run
  python src/main/java/frc/jetson_tools/vision_nt_e2e_test.py
  ```
- `VisionIONT`：
  - 用 `getAtomic().timestamp != 0` 判斷有沒有真的收過 Jetson 的值，修掉開機時假連線 0.25 秒。
  - 新增輸入 `Vision/AgeMs`：**機器人時鐘**算的「距離上次收到新結果幾毫秒」。原本用發布端的 NT timestamp 去減，跨程序時鐘沒對齊，測到 70~130 秒，已改掉。完整資料年齡 ≈ `LatencyMs` + `AgeMs`。
- ⚠️ `vision_nt_publisher.py`、`vision_fake_publisher.py`：NT4 publisher 預設每 0.1 s 才合併送一次，機器人看到的視覺資料會晚 0~100 ms（實測 AgeMs 中位數 50、最大 100 ms）。改成 `PubSubOptions(periodic=0.01, sendAll=True)`，並在每張結果更新 heartbeat 後 `inst.flush()`。改完中位數 10 ms、最大 31 ms（= 30 Hz 一張的間隔）。**Jetson 上要更新 `vision_nt_publisher.py`。**
- 測試涵蓋：沒有 Jetson 時 Connected=false、topic 與數值（三欄誤差、亮燈欄）、AgeMs、斷線 0.25 s 內判定並清空偵測、重連後數值是新的、沒亮燈時 LitErrorMeters = NaN。
- ✅ 已更新到 Jetson 並裝成開機自動啟動的 systemd 服務 `vision`（2026-11-12）：
  - Jetson 實際路徑：程式 `~/tel2026/jetson_tools/`、模型 `~/tel2026/models/best.engine`；`vision.service` 與 README 已改成這組路徑。舊版備份為 `vision_nt_publisher.py.bak-1112`。
  - 還沒有 `camera_calib.yaml`，服務先拿掉 `--calib`，用 `--hfov 70` 估角度（對準時目標在畫面中央，影響很小）。有校正檔後再加回。
  - 更新程式：`scp` 到 Jetson 後 `sudo systemctl restart vision`；看 log：`journalctl -u vision -f`。

## 誤判防護與 LaserAlign 安全機制（2026-11-12）

- `Vision.columnError`：原本「看到三群就當成左中右、兩群相距夠遠就當成左右」，一個誤判的框（燈、隊服）就會讓 VisionAlign 對到空牆。改成只接受**間距像九宮格**的組合：
  - 三欄：兩個間距要相等（差 ≤ 0.18 m），且是欄距 0.9 m 的 0.6~1.6 倍（TF-Luna 讀錯或用預設距離時欄距會縮放）。看到四群以上時挑最像的三群，多的當雜訊。
  - 只看到兩群：間距要在 1.35~2.25 m 才當成左右兩欄。三群以上又湊不出九宮格時不猜，回傳 empty（VisionAlign 1 秒後放棄，比對錯好）。
  - `VisionTest` 新增 10 個邊界測試（空畫面、單欄、邊緣半欄、框不準、各種假框、距離誤差），修改前有 4 個失敗。
- `LaserAlign`：原本沒有逾時；`kYawSign` 或左右 TF-Luna 接反時會一直轉到 TF-Luna 失去牆面。新增：
  - 2 秒逾時。
  - 誤差比開始時多 0.15 rad（≈ 8.6°）就停，並記錄 `LaserAlign/WrongDirection` = true → 實機看到就把 `kYawSign` 改成 -1。
  - `LaserAlign/ErrorRad` 記錄。
- `DualTFLunaRangeSensor.yawRad(left, right)`：偏航角公式抽成靜態函式；`RangeSensors.setForTesting()` 讓測試換假感測器。
- `LaserAlignSimTest`（12 個）：用模擬位置算兩顆 TF-Luna 的距離。方向對時 ±30° 內 1.1 s 轉正（誤差 2.7°）；方向反時 0.5 s 停、只多歪約 11°。結論：`kYawSign = +1` 對應「程式的 Left 是車尾那顆」（TF-Luna 朝機器人右側時）。
- 檢查清單新增 1.6 `kBaselineMeters`、1.7 目標距離校正（`kWallToTargetMeters`、`kSensorOffsetMeters`）。

## 序列埠／攝影機改用固定路徑（2026-11-14）

- `FlySkyController.kSerialPorts`：改成 CP2102 的 `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`，找不到才退回 `/dev/ttyUSB0`。移除 `/dev/ttyACM0`，因為那是 **Limelight**，原本 USB-TTL 沒認到時會誤開它。實機 hexdump 確認 i-BUS 封包與 checksum 正常；程式用 `systemcore` 使用者執行，已在 `dialout` 群組。
- `vision.service`：加上 `--device /dev/v4l/by-id/usb-Arducam_..._OV9281_USB_Camera_UC762-video-index0`。Arducam OV9281 在 USB 斷線重連後從 video0 跳到 video1，服務就打不開攝影機。
- `vision_nt_publisher.py`：攝影機斷線時原本會卡在 `LatestFrame.get()` 永遠等新畫面，程式沒結束，`Restart=always` 不會觸發。改成 2 秒沒新畫面就丟例外結束（第一張多等 3 秒）；打不開攝影機改成結束碼 1。`vision.service` 加 `StartLimitIntervalSec=0`，避免連續失敗後 systemd 放棄重啟。
- 同日 OV9281 換到別的 USB 孔仍反覆斷線，改用 Generalplus「WEB CAM」（1b3f:1147，彩色）；`vision.service` 的 `--device` 改成 `/dev/v4l/by-id/usb-WCM_USB_WEB_CAM-video-index0`。視角未知，仍用預設 `--hfov 70`，待量測。
- 便宜 webcam 設定：`camera_check.lock_camera()` 關自動對焦，開機時讓自動曝光／白平衡調 1.5 秒後鎖住（`--exposure`、`--wb`：`auto`／`lock`／數字，不支援時只警告）。`vision_nt_publisher.py` 與 `camera_check.py` 都加了這兩個參數；`vision.service` 改用 640x480。實機 `v4l2-ctl -l` 顯示 jsing wb1 只有 brightness、power_line_frequency，沒有曝光／白平衡控制，所以 service 用 `--exposure auto --wb auto`。實測只有約 15 fps，原因待確認（暗處自動曝光拉長，或攝影機本身上限）。
- `VisionIOSim.frameRateHz` 改成可調（預設 30），`VisionAlignSimTest.fpsSweep`（`VA_FPS=1`）比較幀率與延遲：30 fps／60 ms、15 fps／150 ms、10 fps／200 ms 的落點誤差都在 1.4 cm 內、時間差 < 0.2 s，VisionAlign 靠里程計補幀間移動，15 fps 夠用。模擬沒有模擬動態模糊。
- 同日又換回 Arducam OV9281：`vision.service` 改回 OV9281 的 by-id 路徑、預設 1280x720、`--exposure lock --wb auto`（webcam 的參數留在註解）。OV9281 是**黑白**，原本的紅色像素亮燈判斷永遠是 0 → `lit_score` 偵測到三通道幾乎相同時改數亮度 ≥ 200 的像素（`MONO_LIT_MIN`）。⚠️ 本機沒有 OpenCV，未實測；`LitThreshold` 要在有燈條的場地重新調。
- OV9281 換線後仍約每 10 秒 USB 斷線一次：
  - `vision_nt_publisher.LatestFrame`：讀不到畫面 0.3 秒就在程式內重開攝影機（套用已鎖定的曝光，不再等 1.5 秒），不必整個重啟、重載模型；10 秒都重連不上才結束交給 systemd。`open_camera` 設 `CAP_PROP_READ_TIMEOUT_MSEC` 500（OpenCV 有支援時），避免 read() 卡 10 秒。假攝影機測試：斷線後 0.7 秒恢復。
  - `VisionAlign`：`kLostTimeoutSec` 0.5 → 1.5、`kAcquireTimeoutSec` 1.0 → 2.0。看丟期間靠里程計追，到位就停著等。
  - `VisionIOSim.dropoutFrom/Until` 模擬斷線；`VisionAlignSimTest.survivesCameraDropout`：斷線 1.2 秒發生在開始後 0/0.3/0.8/1.5 秒，改之前 3/4 失敗（落點差 0.60/0.24/0.08 m），改之後全過（≤ 1.7 cm，時間 2.1~3.5 s）。全部測試通過。
- Limelight 2 當網路攝影機（試用）：`open_camera` 遇到 `http://`、`https://`、`rtsp://` 改用 FFmpeg 讀串流，曝光／白平衡交給 Limelight 網頁；斷線重連同樣適用。`vision.service` 只加註解範例（`--device http://10.71.30.12:5802/ --hfov 59.6`），實測串流在 port **5802**、640x480、15 fps 後，`vision.service` 改用 Limelight 2（`--device http://10.71.30.12:5802/ --hfov 59.6`），OV9281 與 webcam 參數留在註解。Limelight 預設 IP 10.TE.AM.11 會和 Jetson 衝突，要先改成 .12；需要網路交換器連 SystemCore、Jetson、Limelight。規則要求自動期用 AI 辨識，所以 YOLO 仍在 Jetson 跑。
- 實機確認（2026-11-14）：Limelight 2 視角 **HFOV 62.5°、VFOV 48.9°**（取代規格值 59.6），`vision.service` 改成 `--hfov 62.5`；綠色補光 LED 已關。FlySky 經 USB-TTL 讀取上機測試正常。
- ⚠️ 實機 dmesg 看到 OV9281 反覆 `USB disconnect`／`error -71`（接在 Jetson 的 USB 2.0 Hub 上），這是硬體連線問題，固定路徑救不了：要換線、直插 Jetson 本體 USB 孔。

## Storage 三馬達、補球觸發、砲台絕對編碼器（2026-11-14）

- **Storage 改成三顆馬達**（左 storage、右 storage、trigger），用兩塊 L298N。Smart I/O 0~5 全部接 IN 腳，改用 `DigitalOutput` 只控制開關與方向；ENA／ENB 的 jumper 要**插著**（常開）→ 馬達開啟就是全速（roboRIO 時代約 40%）。
  - 0 Trigger 反轉、1 Trigger 正轉、2 左 Storage 反轉、3 左 Storage 正轉、4 右 Storage 反轉、5 右 Storage 正轉（`StorageConstants`）。
  - `Stroage` 對外方法不變；`setStorageSpeed` 同時控制左右兩顆，速度只看是否為 0。兩顆方向相反時請對調該馬達的 OUT 線。
- **`Turret.ballFired()`**：原本看扳機 SparkMax 回報電壓 `< 3`，扳機馬達 inverted 時回報值可能是負的 → 永遠成立 → `Reload` 從不觸發，Storage 一直停在 `Normal` 整理球。改成看程式下的指令（扳機通電 → 斷電）。⚠️ 未上機確認。第一顆球仍不會預先補上（第一次射擊可能空射）。
- **砲台仰角改用 CANcoder 絕對位置**：拿掉開機 `setPosition`（原本把開機位置當成最低角度），改讀 `getAbsolutePosition()`，最低點 = 0 圈，行程約 0.82 圈。新增 `kEncoderMagnetOffsetRot`（⚠️ **待上機校正**，目前 0）與 `kEncoderDiscontinuityPointRot = 0.9`，log `Turret/PitchEncoderAbsRot`。

## 插值表、砲台 TF-Luna、Elastic 狀態頁（2026-11-14）

- **射球仰角插值表 `ShotTable`**（`subsystems/turret/ShotTable.java`）：每列（低／中／高／加分格）一張「`RangeSensor/TargetDistance` → 仰角 rad」表，表內線性插值；表外用原彈道公式加上端點差（連續、不跳）；某列少於 2 點就完全用原公式（所以表空著時行為和以前一樣）。`ColumnShot`、`AssistTurret` 改呼叫 `ShotTable.pitchRad()`，彈道係數 `0.019358` 移到 `ShotTable.kBallisticCoefficient`。log：`ShotTable/Row`、`Source`（TABLE／EXTRAPOLATED／FORMULA）、`DistanceMeters`、`PitchRad`。`ShotTableTest` 4 項。
- **`ManualTurret` 量表輔助**：NT `/Tuning/Manual/PitchRad` > 0 → 仰角固定在這個值（不看搖桿）；每次擊發印一行 `ShotTable 樣本 {距離, 仰角}, // 實際仰角、飛輪是否到速、砲台 TF-Luna 距離`（console + `ShotTable/Sample`）。量測步驟：Checklist §5.6。
- **砲台 TF-Luna（第三顆）**：裝在砲台水平正前方、照九宮格的洞，I2C 接 Jetson（排針 pin 3/5 = `/dev/i2c-7`，位址 0x10）。Jetson 跑 `jetson_tools/turret_tfluna_nt_publisher.py`（smbus2，100 Hz，NT 表 `TurretLidar`：distance／strength／temperature／heartbeat／i2cErrors，periodic 10 ms + flush），服務檔 `turret-tfluna.service`。機器人端 `subsystems/turretlidar/*`（`TurretLidarIONT`，0.25 s 沒新 heartbeat 算斷線；有效範圍與強度門檻同左右兩顆）。**目前只記錄與顯示，沒有拿來控制**；量表時一併記下，之後可決定改用它當查表距離。安裝：Checklist §5.7。
- **Elastic 改版**：移除「記分」分頁（第二操作手不能碰電腦），新增「狀態」分頁，資料來自新的 `StatusBoard`（`/AdvantageKit/RealOutputs/Status/...`）：模式、飛輪（停止／加速 xx%／就緒）、仰角、射出數、電池、目標距離、砲台 TF-Luna 距離、Auto 階段、Enabled，以及 9 個連線燈（遙控器、Jetson 視覺、左右 TF-Luna、砲台 TF-Luna、上下飛輪 TalonFX、仰角 CANcoder、IMU）；警示列列出所有未連線的部件與電池低（< 11.5 V）。「賽前」分頁拿掉「全部歸零」，加上「手動固定仰角」和「最後一筆查表樣本」。`Scoreboard` 程式保留（仍會記 log），只是不再顯示。
  - **砲台 TF-Luna 用途確定（同日）**：只用來確認**左右對準了某一欄**（光束穿過洞），選哪一列仍由 ShotTable 調仰角；只在 Elastic 亮燈（「對準洞」，沒對準是灰色不是紅色），**不阻止射擊**。感測器固定水平安裝、本身有固定仰角，不跟砲台動。`TurretLidar.classify()`：讀值 0／訊號 < 100（TF-Luna 前方沒東西時回傳 0）或比預期板子距離遠 > 0.3 m → `THROUGH_HOLE`；± 0.3 m 內 → `BOARD`；近很多 → `BLOCKED`；debounce 0.06 s。預期距離 = (TargetDistance + `kHorizontalOffsetMeters`) / cos(`kMountPitchDeg`)；`BeamRow` 顯示光束在板面的高度落在哪一列。安裝高度、仰角、水平差 **待上機量測**（Checklist §5.8）。`TurretLidarTest` 6 項。⚠️ 九宮格是黑板，TF-Luna 在低反射率時量程只有約 2.5 m，要先確認打在板上讀得到。
  - Elastic「砲台TF-Luna」距離：未連線顯示「未連線」，讀值 0／訊號弱顯示「超出範圍」，不再顯示 0。左右兩顆的原始 log `RangeSensor/TFLuna/*Meters` 仍會出現 0（TF-Luna 原始值），但 `*Valid` = false，不會被拿去算距離（改用 fallback 3.0 m）。
  - `TurretIO` 新增 `upConnected`／`downConnected`／`encoderConnected`（Phoenix `BaseStatusSignal.isAllGood`）；`Drive.isGyroConnected()`；`DualTFLunaRangeSensor.isLeft/RightConnected()`；開機先 log `Mode = disable`。

---

## 4. 其他待確認事項

1. **I/O 接線**：Storage 照「I/O 埠重新分配」改接；FlySky 接收機接到 Jetson（✅ 已確認 NT 傳輸、通道對應與搖桿軸方向，2026-11-07）。2026-11-13 改為 USB-TTL 直接接 SystemCore，⚠️ 待實機確認 `FlySky/Source` = `kSerial`。
2. **TF-Luna I2C**：第 5 腳接 GND（I2C 模式）；左接 I2C 0、右接 I2C 1。在 AdvantageScope 確認 `RangeSensor/TFLuna/LeftConnected`、`RightConnected` 為 true，並確認左右沒有接反（`TiltRad` 正負）。
3. **CAN 埠**：✅ `Constants.kCANPort` 與實際接線一致（2026-11-07）。
4. **底盤 PID**：REVLib 單位換算改在程式中處理，PID 增益已等比例縮放，請在實機上確認速度控制正常。
5. **IMU 安裝方向**：確認 `GyroIOOnboardIMU` 的 `MountOrientation`。
6. **MockDS**：✅ 已在 SystemCore 實機確認可以用 FlySky 左開關啟用和停用機器人（2026-11-07）。
7. **待函式庫出 2027 版後可恢復**：MapleSim（移除 `build.gradle` 的 `exclude`、恢復 `Robot.simulationPeriodic()`）、URCL（恢復 `Logger.registerURCL`）。
8. **砲台 CANcoder 校正**：`kEncoderMagnetOffsetRot` 先 0 部署 → 砲台放最低點 → 讀 `Turret/PitchEncoderAbsRot` → 填它的負值。往上推時讀值應變大，變小代表要反轉 `SensorDirection`。
9. **Storage 補球**：射一發後 `Turret/ShotCount` +1，Storage 應反轉 0.3 秒再正轉送球。

## LaserAlign NT 診斷輸出（2026-11-15）

- 問題：`LaserAlign/*` 只有指令執行時才寫，開機後 NT 看不到；測距無效（yaw = null）時指令第一個週期就結束，`ErrorRad` 永遠不會出現。
- 改法（`commands/drive/LaserAlign.java`）：建構時就發布所有 key；新增
  - `LaserAlign/Active`：指令是否在跑
  - `LaserAlign/YawValid`：這個週期測距是否有效（false 時 `ErrorRad` = NaN）
  - `LaserAlign/EndReason`：`NotRun`／`Running`／`Aligned`／`NoYaw`／`WrongDirection`／`Timeout`／`Interrupted`
- NT 路徑：`/AdvantageKit/RealOutputs/LaserAlign/...`。觸發條件：非 Manual 模式＋已 enable＋lefttop。

## 11/15 量測寫回（第一批）

**方向約定**：射球方向 = 機器人右側（面向九宮格）。隊員量測時把「面向九宮格」當前方，所以他說的「右側 frame」= **車尾**、「左側 frame」= **車頭**。程式裡的「右 frame」= 面向九宮格那側（深度基準）。

| 常數 | 舊值 | 新值 | 來源 |
|---|---|---|---|
| `kShooterInboardMeters`（**DownShooter 輪心**深度） | 0.25 | 0.16953 | 基準改成 DownShooter 輪心：Hood 調仰角時出球點會動，輪心不會 |
| `kLunaTrimMeters` | 0.0 | 0.0 | 實測 -0.61 mm，可忽略 |
| `kEncoderMagnetOffsetRot` | 0.0 | 0.230469 | 已確認最低點讀 ≈ 0 |
| `TurretLidar.kMountHeightMeters` | 0.52 | 0.48029 | 實測 |
| `TurretLidar.kMountPitchDeg` | 0 | 30 | 已確認 |
| `TurretLidar.kForwardOfShooterMeters`（新增） | — | 0.15880 | 光達在 DownShooter 輪心前方（朝九宮格）；→ 光達深度 10.73 mm（實測一致） |
| `TurretLidar.kInboardMeters` | 0.25 | `kShooterInboardMeters − kForwardOfShooterMeters` | 改成算出來的 |
| `VisionConstants.kShooterForwardOfCameraMeters` | 0.15 | 0.24241 | Limelight 在砲台 Luna 右方（面向九宮格的右＝車尾）242.41 mm |
| `VisionConstants.kCameraHeightMeters` | 0.45 | 0.46459 | 只用在模擬 |
| `VisionConstants.kCameraBehindShooterMeters`（新增） | — | 0.16165 | Limelight 在砲台 Luna 後方 320.45 mm → 深度 331.18 − 169.53 mm |

- `Vision.cameraDistance()` = TargetDistance + `kCameraBehindShooterMeters`：tx → 公尺改用鏡頭到板面距離。`Vision.periodic()`、`VisionAlign.execute()` 都用它。
- `AutoRoutineSimTest` 加假測距（依模擬位置），原本 DualTFLuna 在模擬回預設 3.0 m（實際 2.5 m），`kShooterForwardOfCameraMeters` 變大後不跑路徑的案例落點偏 5 cm 超出容許。⚠️ 實機同理：**TargetDistance 不準會讓 VisionAlign 偏**，偏差 ≈ kShooterForwardOfCameraMeters × 距離誤差比例。
- `apply_measurements.py`：新增 `kCameraHeightMeters`、`kCameraBehindShooterMeters`、`kTurretLunaForwardOfShooterMeters`；`turretLunaInboardMeters`、`cameraBehindTurretLunaMeters` 只記錄。
- CSV 要存成 UTF-8（Excel 存會壞），長度單位公尺。
