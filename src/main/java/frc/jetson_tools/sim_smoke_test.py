#!/usr/bin/env python3
"""模擬器冒煙測試：假裝是 Jetson 送 FlySky 資料，自動 enable 機器人、開車，檢查底盤與視覺有沒有反應。

先在另一個終端機跑 `./gradlew run`（模擬器），等出現 "Robot program startup complete" 再跑：
  python sim_smoke_test.py

測試步驟：
  1. 送中立搖桿 → 左開關打開 → MockDS 應讓機器人 enable
  2. 左搖桿 Y 推到底 2 秒 → 左右輪位置都應該改變（直走）
  3. 左搖桿 X 推到底 2 秒 → 左右輪應反向（原地轉）
  4. 停止 heartbeat（模擬 Jetson 斷線）→ 馬達電壓應歸零、機器人應 disable
  5. heartbeat 恢復 → 應在 1.5 秒內重新 enable
期間印出 Vision/DetectionCount 給參考。
"""
import argparse
import threading
import time

import ntcore

NEUTRAL = 1500.0
LOW = 1000.0
HIGH = 2000.0


class FakeFlySky:
    """以 50 Hz 發布 /FlySky/channels 與 /FlySky/heartbeat。"""

    def __init__(self, inst):
        self.inst = inst
        table = inst.getTable("FlySky")
        self.channels_pub = table.getDoubleArrayTopic("channels").publish()
        self.heartbeat_pub = table.getIntegerTopic("heartbeat").publish()
        self.channels = [NEUTRAL] * 10
        self.channels[4] = LOW  # CH5 左開關關
        self.channels[5] = LOW  # CH6 右開關關
        self.alive = True
        self.running = True
        self.count = 0
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            if self.alive:
                self.count += 1
                self.channels_pub.set(list(self.channels))
                self.heartbeat_pub.set(self.count)
                # 和 flysky_nt_publisher.py 一樣每次都 flush；不 flush 的話 pyntcore 每 100ms 才送一次，
                # 會撞到機器人端 0.1 秒的斷線逾時，造成 MockDS 反覆重開、enable 閃斷。
                self.inst.flush()
            time.sleep(0.02)

    def set(self, ch, value):
        self.channels[ch] = value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", default="127.0.0.1")
    args = parser.parse_args()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("sim-smoke-test")
    inst.setServer(args.server, 5810)

    ak = inst.getTable("AdvantageKit")
    enabled = ak.getSubTable("DriverStation").getBooleanTopic("Enabled").subscribe(False)
    drive = ak.getSubTable("Drive")
    left_pos = drive.getDoubleTopic("LeftPositionMeters").subscribe(float("nan"))
    right_pos = drive.getDoubleTopic("RightPositionMeters").subscribe(float("nan"))
    left_vel = drive.getDoubleTopic("LeftVelocityMetersPerSec").subscribe(float("nan"))
    right_vel = drive.getDoubleTopic("RightVelocityMetersPerSec").subscribe(float("nan"))
    left_volts = drive.getDoubleTopic("LeftAppliedVolts").subscribe(float("nan"))
    right_volts = drive.getDoubleTopic("RightAppliedVolts").subscribe(float("nan"))
    det_count = ak.getSubTable("RealOutputs").getSubTable("Vision").getIntegerTopic("DetectionCount").subscribe(-1)

    for _ in range(50):
        if inst.isConnected():
            break
        time.sleep(0.1)
    if not inst.isConnected():
        print("FAIL 連不到 NT 伺服器，模擬器有開嗎？")
        return 1
    print("已連上 NT")

    fly = FakeFlySky(inst)
    results = []

    def check(name, ok, detail=""):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'} {name} {detail}")

    def state():
        return (f"enabled={enabled.get()} L={left_pos.get():.3f}m R={right_pos.get():.3f}m "
                f"vL={left_vel.get():.2f} vR={right_vel.get():.2f} "
                f"VL={left_volts.get():.2f} VR={right_volts.get():.2f} detections={det_count.get()}")

    time.sleep(1.0)
    print("初始：", state())

    # 1. enable
    fly.set(4, HIGH)
    t0 = time.time()
    while time.time() - t0 < 5 and not enabled.get():
        time.sleep(0.1)
    check("左開關打開後 enable", enabled.get(), f"({time.time() - t0:.1f}s)")

    flicker = [0]
    def watch():
        while fly.alive:
            if not enabled.get():
                flicker[0] += 1
            time.sleep(0.02)
    threading.Thread(target=watch, daemon=True).start()

    # 2. 直走
    l0, r0 = left_pos.get(), right_pos.get()
    fly.set(1, LOW)
    time.sleep(2.0)
    print("直走中：", state())
    fly.set(1, NEUTRAL)
    time.sleep(1.0)
    dl, dr = left_pos.get() - l0, right_pos.get() - r0
    check("左搖桿 Y 直走", abs(dl) > 0.1 and abs(dr) > 0.1 and dl * dr > 0, f"dL={dl:.3f} dR={dr:.3f}")

    # 3. 原地轉
    l0, r0 = left_pos.get(), right_pos.get()
    fly.set(0, HIGH)
    time.sleep(2.0)
    print("轉向中：", state())
    fly.set(0, NEUTRAL)
    time.sleep(1.0)
    dl, dr = left_pos.get() - l0, right_pos.get() - r0
    check("左搖桿 X 原地轉", abs(dl) > 0.05 and abs(dr) > 0.05 and dl * dr < 0, f"dL={dl:.3f} dR={dr:.3f}")

    # 4. 斷線保護：一邊推搖桿一邊停 heartbeat
    fly.set(1, LOW)
    time.sleep(1.0)
    print("斷線前：", state())
    fly.alive = False
    check("enable 期間沒有閃斷", flicker[0] == 0, f"(disabled 取樣 {flicker[0]} 次)")
    time.sleep(1.5)
    print("斷線後：", state())
    # 模擬底盤沒有煞車，輪速會慢慢滑停，所以看馬達電壓
    stopped = abs(left_volts.get()) < 0.05 and abs(right_volts.get()) < 0.05
    check("Jetson 斷線後馬達電壓歸零", stopped)
    check("Jetson 斷線後 disable", not enabled.get())

    # 5. 斷線恢復：FlySky 回來、左開關仍開著 → 應在約 1 秒內重新 enable（MockDS 先送 50 個 disabled 封包）
    fly.set(1, NEUTRAL)
    fly.alive = True
    t0 = time.time()
    while time.time() - t0 < 5 and not enabled.get():
        time.sleep(0.05)
    dt = time.time() - t0
    check("FlySky 恢復後重新 enable", enabled.get() and dt < 1.5, f"({dt:.2f}s)")

    fly.set(4, LOW)
    time.sleep(0.5)
    fly.running = False
    print(f"\n{sum(results)}/{len(results)} 通過")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
