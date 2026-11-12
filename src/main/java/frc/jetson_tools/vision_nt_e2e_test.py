#!/usr/bin/env python3
"""NT 端到端測試：vision_fake_publisher.py → 模擬器機器人（VisionIONT）。

先用 VisionIONT 開模擬器（PowerShell）：
  $env:VISION_IO="nt"; ./gradlew run
等出現 "Robot program startup complete" 再跑：
  python vision_nt_e2e_test.py

檢查：開機沒有假連線、topic 與數值對得上、資料年齡（NT 延遲）、Jetson 斷線後多久判定斷線並清空、重連、沒亮燈。
模擬器沒有 TF-Luna，距離用 kFallbackDistanceMeters = 3.0，所以假資料也用 3.0 m。
"""
import argparse
import math
import os
import statistics
import subprocess
import sys
import time

import ntcore

HERE = os.path.dirname(os.path.abspath(__file__))
DISTANCE = 3.0  # = RangeSensorConstants.kFallbackDistanceMeters
SHOOTER_FORWARD = 0.15  # = VisionConstants.kShooterForwardOfCameraMeters
SPACING = 0.90
TIMEOUT_S = 0.25  # = VisionConstants.kTimeoutSeconds

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {detail}")


def expected_columns(offset):
    # kImageRightIsRobotForward = false → error = -d*tan(tx) - k = -x - k；由大到小排序
    return sorted((-(offset + (c - 1) * SPACING) - SHOOTER_FORWARD for c in range(3)), reverse=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--server", default="127.0.0.1")
    args = p.parse_args()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("vision-e2e-test")
    inst.setServer(args.server, 5810)
    ak = inst.getTable("AdvantageKit")
    vin = ak.getSubTable("Vision")
    vout = ak.getSubTable("RealOutputs").getSubTable("Vision")
    connected = vin.getBooleanTopic("Connected").subscribe(False)
    age = vin.getDoubleTopic("AgeMs").subscribe(float("nan"))
    count = vout.getIntegerTopic("DetectionCount").subscribe(-1)
    cols = vout.getDoubleArrayTopic("ColumnErrorsMeters").subscribe([])
    lit = vout.getDoubleTopic("LitErrorMeters").subscribe(float("nan"))
    # 確認機器人在跑 VisionIONT：它會訂閱 /Vision/heartbeat
    for _ in range(50):
        if inst.isConnected():
            break
        time.sleep(0.1)
    if not inst.isConnected():
        print("連不上模擬器 NT（127.0.0.1:5810）")
        return 2
    time.sleep(1.0)

    def wait_for(pred, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if pred():
                return time.monotonic() - t0
            time.sleep(0.005)
        return None

    def start_fake(offset, lit_col):
        return subprocess.Popen(
            [sys.executable, "vision_fake_publisher.py", "--server", args.server,
             "--offset", str(offset), "--distance", str(DISTANCE), "--lit", str(lit_col)],
            cwd=HERE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 1. 還沒有 Jetson：不能顯示連線
    check("沒有 Jetson 時 Connected=false", not connected.get(), f"Connected={connected.get()}")
    if connected.get():
        print("   → 可能有別的 publisher 在跑，或模擬器用的是 VisionIOSim（要設 VISION_IO=nt）")

    # 2. 開假資料
    offset = 0.3
    fake = start_fake(offset, 2)
    try:
        t = wait_for(lambda: connected.get() and count.get() == 12, 5.0)
        check("假資料連上、12 個偵測", t is not None, f"{t:.2f}s" if t else f"count={count.get()}")
        time.sleep(0.3)
        exp = expected_columns(offset)
        got = list(cols.get())
        ok = len(got) == 3 and all(abs(a - b) < 0.01 for a, b in zip(got, exp))
        check("ColumnErrorsMeters 數值", ok, f"got={[round(x, 3) for x in got]} exp={[round(x, 3) for x in exp]}")
        exp_lit = -(offset + SPACING) - SHOOTER_FORWARD  # lit=2 → c=2
        check("LitErrorMeters（右欄亮）", abs(lit.get() - exp_lit) < 0.01, f"got={lit.get():.3f} exp={exp_lit:.3f}")

        ages = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            a = age.get()
            if not math.isnan(a):
                ages.append(a)
            time.sleep(0.02)
        if ages:
            ages.sort()
            p95 = ages[int(len(ages) * 0.95) - 1]
            check("資料新鮮度 AgeMs（距上次新結果）p95 < 60 ms", p95 < 60,
                  f"median={statistics.median(ages):.1f} p95={p95:.1f} max={ages[-1]:.1f} ms（假資料 30 Hz + 20 ms 週期，正常 0~40 ms）")
        else:
            check("資料年齡 AgeMs", False, "一直是 NaN")
    finally:
        fake.kill()
        fake.wait()

    # 3. Jetson 斷掉
    t = wait_for(lambda: not connected.get(), 2.0)
    check(f"斷線判定（kTimeout={TIMEOUT_S}s）", t is not None and t < TIMEOUT_S + 0.15,
          f"{t:.2f}s" if t is not None else "2 秒內都沒判定斷線")
    t = wait_for(lambda: count.get() == 0 and len(cols.get()) == 0, 1.0)
    check("斷線後清空偵測（不會拿舊資料對準）", t is not None, f"count={count.get()} cols={list(cols.get())}")

    # 4. 重連（heartbeat 從 1 重新算）+ 沒亮燈
    fake = start_fake(-0.2, -1)
    try:
        t = wait_for(lambda: connected.get() and count.get() == 12, 5.0)
        check("重連", t is not None, f"{t:.2f}s（含啟動 Python）" if t else "")
        time.sleep(0.3)
        exp = expected_columns(-0.2)
        got = list(cols.get())
        check("重連後數值是新的", len(got) == 3 and all(abs(a - b) < 0.01 for a, b in zip(got, exp)),
              f"got={[round(x, 3) for x in got]}")
        check("沒亮燈 → LitErrorMeters = NaN", math.isnan(lit.get()), f"got={lit.get()}")
    finally:
        fake.kill()
        fake.wait()

    print(f"\n{sum(results)}/{len(results)} PASS")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
