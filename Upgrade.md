# roboRIO → SystemCore 升級紀錄

本文件記錄將 TEL2026 機器人程式從舊版 roboRIO 移植到 SystemCore（WPILib 2027 alpha）所做的所有修改。

- 修改日期：2026-11-07
- 驗證狀態：`./gradlew build` 通過（原本約 200 個編譯錯誤）
- **尚未**部署到實體 SystemCore，也**尚未**跑過模擬器，執行期行為未經測試
- 環境：GradleRIO `2027.0.0-alpha-8`、Java 25、WPILib `2027_alpha7`

專案的 `build.gradle` 與 vendordeps 原本就已設定為 SystemCore（GradleRIO 2027），問題主要出在 Java 程式碼與第三方函式庫。

---

## I/O 埠重新分配

SystemCore 只有 **6 個 Smart I/O 埠（0–5）**（已從 HAL 原始碼確認：`NUM_SMART_IO = 6`，實機與模擬器相同），roboRIO 時代的配置需要 12 個，因此重新分配如下：

| 用途 | 原 roboRIO 埠 | 新配置 | 檔案 |
|---|---|---|---|
| Storage 方向控制 | DIO 0、1、2、3 | **Smart I/O 0、1、2、3** | `subsystems/storage/StorageConstants.java` |
| Storage / Trigger 馬達（PWM Spark） | PWM 8、9 | **Smart I/O 4、5** | `subsystems/storage/StorageConstants.java` |
| FlySky 遙控接收機 | DIO 4、5、6、7、8、9（PWM 訊號） | **接到 Jetson（iBus），經 NetworkTables 傳給機器人** | `controller/FlySkyController.java` |
| 兩顆 TF-Luna 測距 | 協處理器，經 NetworkTables | **直接接 SystemCore I2C 0（左）、I2C 1（右）** | `FSLib/rangefinder/RangeSensorConstants.java` |

腳位對照：DIO 0→0、DIO 1→1、DIO 2→2、DIO 3→3、PWM 8→4、PWM 9→5。**實體接線也要跟著改。**

---

## FlySky 遙控器：Jetson + NetworkTables

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

### `robot/subsystems/storage/Stroage.java`
- PWM `Spark.set(speed)` → `Spark.setThrottle(speed)`

### `robot/subsystems/storage/StorageConstants.java`
- 腳位改為 SystemCore Smart I/O 0~5：
  - `kStorageReversePort` = 0、`kStorageForwardPort` = 1、`kTriggerReversePort` = 2、`kTriggerForwardPort` = 3（DIO 編號不變）
  - `kStorageSpeedPort`：PWM 8 → Smart I/O 4
  - `kTriggerSpeedPort`：PWM 9 → Smart I/O 5

---

## 4. 其他待確認事項

1. **I/O 接線**：Storage 照「I/O 埠重新分配」改接；FlySky 接收機接到 Jetson（✅ 已確認 NT 傳輸與通道對應）。
2. **TF-Luna I2C**：第 5 腳接 GND（I2C 模式）；左接 I2C 0、右接 I2C 1。在 AdvantageScope 確認 `RangeSensor/TFLuna/LeftConnected`、`RightConnected` 為 true，並確認左右沒有接反（`TiltRad` 正負）。
3. **CAN 埠**：確認 `Constants.kCANPort` 與實際接線一致。
4. **底盤 PID**：REVLib 單位換算改在程式中處理，PID 增益已等比例縮放，請在實機上確認速度控制正常。
5. **IMU 安裝方向**：確認 `GyroIOOnboardIMU` 的 `MountOrientation`。
6. **MockDS**：✅ 已在 SystemCore 實機確認可以用 FlySky 左開關啟用和停用機器人（2026-11-07）。
7. **待函式庫出 2027 版後可恢復**：MapleSim（移除 `build.gradle` 的 `exclude`、恢復 `Robot.simulationPeriodic()`）、URCL（恢復 `Logger.registerURCL`）。
