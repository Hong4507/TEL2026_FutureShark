#!/usr/bin/env python3
"""把 11/15 量測記錄寫回程式：ShotTable 的資料表、各 Constants，並算自動流程建議發數。

讀：
  shot_table_1115.csv   row,distance_m,pitch_rad,hits_of_5,note
                        row = LOW／MID／HIGH／BONUS；distance_m 用 NT RangeSensor/TargetDistance；
                        pitch_rad 用 ManualTurret 收斂後的仰角（/Tuning/Manual/PitchRad）
  constants_1115.csv    name,value,unit,note
空白欄位一律跳過（不會把程式裡原本的值清掉）。

做：
  1. 改 ShotTable.java 的 kLow／kMid／kHigh／kBonus（有填距離＋仰角的列才算一點）
  2. 改 RangeSensorConstants／TurretConstants／TurretLidar／VisionConstants 裡對應的常數
  3. 印出 diff，確認後才寫入（--yes 不問、--dry-run 只印不寫）
  4. hits_of_5 → 命中率 → 呼叫 jetson_tools/strategy_calc.py，印建議的 BonusBalls／BallsPerHole

用法（在 repo 根目錄或這個資料夾都可以）：
  python tools/measure/apply_measurements.py --dry-run
  python tools/measure/apply_measurements.py            # 看完 diff 輸入 y
寫完記得：./gradlew test，然後部署。
"""
import argparse
import csv
import difflib
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
JAVA = os.path.join(REPO, "src", "main", "java", "frc")
SHOT_TABLE = os.path.join(JAVA, "robot", "subsystems", "turret", "ShotTable.java")
STRATEGY_DIR = os.path.join(JAVA, "jetson_tools")

ROW_ARRAYS = {"LOW": "kLow", "MID": "kMid", "HIGH": "kHigh", "BONUS": "kBonus"}

# CSV 名稱 → (檔案, Java 常數名稱, 型別)
CONSTANTS = {
    "kShooterInboardMeters": ("FSLib/rangefinder/RangeSensorConstants.java", "kShooterInboardMeters", "double"),
    "kYawSign": ("FSLib/rangefinder/RangeSensorConstants.java", "kYawSign", "sign"),
    "kLunaTrimMeters": ("FSLib/rangefinder/RangeSensorConstants.java", "kLunaTrimMeters", "double"),
    "kRangeReference": ("FSLib/rangefinder/RangeSensorConstants.java", "kRangeReference", "reference"),
    "kEncoderMagnetOffsetRot": ("robot/subsystems/turret/TurretConstants.java", "kEncoderMagnetOffsetRot", "double"),
    "kTurretLunaHeightMeters": ("robot/subsystems/turretlidar/TurretLidar.java", "kMountHeightMeters", "double"),
    "kTurretLunaPitchDeg": ("robot/subsystems/turretlidar/TurretLidar.java", "kMountPitchDeg", "double"),
    "kTurretLunaInboardMeters": ("robot/subsystems/turretlidar/TurretLidar.java", "kInboardMeters", "double"),
    "kShooterForwardOfCameraMeters": ("robot/subsystems/vision/VisionConstants.java", "kShooterForwardOfCameraMeters", "double"),
}
# 只記錄、不寫回程式的項目
RECORD_ONLY = {"leftLunaHeightMeters", "rightLunaHeightMeters", "lowWallHeightMeters", "flywheelSpinUpSec"}


def blank(v):
    return v is None or str(v).strip() == ""


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return [{k.strip(): (v or "").strip() for k, v in r.items() if k} for r in csv.DictReader(f)]


def read_shots(path):
    """回傳 ({row: [(d, pitch, hits, note)]}, {row: [hits, shots]})，並檢查格式。"""
    points = {r: [] for r in ROW_ARRAYS}
    hits = {r: [0, 0] for r in ROW_ARRAYS}
    errors = []
    for i, r in enumerate(read_csv(path), start=2):
        row = r.get("row", "").upper()
        if row not in ROW_ARRAYS:
            errors.append(f"第 {i} 行：row「{row}」不是 LOW／MID／HIGH／BONUS")
            continue
        d, p, h = r.get("distance_m"), r.get("pitch_rad"), r.get("hits_of_5")
        if not blank(h):
            n = int(float(h))
            if not 0 <= n <= 5:
                errors.append(f"第 {i} 行：hits_of_5 = {n} 不在 0~5")
            hits[row][0] += n
            hits[row][1] += 5
        if blank(d) and blank(p):
            continue
        if blank(d) or blank(p):
            errors.append(f"第 {i} 行：distance_m 和 pitch_rad 要一起填")
            continue
        d, p = float(d), float(p)
        if not 0.5 <= d <= 6:
            errors.append(f"第 {i} 行：距離 {d} m 不合理")
        if not 0.6 <= p <= 1.5:
            errors.append(f"第 {i} 行：仰角 {p} rad 超出砲台行程（0.698~1.45）")
        points[row].append((d, p, None if blank(h) else int(float(h)), r.get("note", "")))
    return points, hits, errors


def render_rows(points):
    """Java 陣列內容（每點一行）。"""
    out = ""
    for d, p, h, note in sorted(points):
        comment = []
        if h is not None:
            comment.append(f"{h}/5")
        if note:
            comment.append(note.replace("*/", ""))
        out += f"    {{{d:.3f}, {p:.4f}}},{'  // ' + ' '.join(comment) if comment else ''}\n"
    return out


def update_shot_table(src, points):
    for row, name in ROW_ARRAYS.items():
        pts = points[row]
        if not pts:
            continue  # 沒量的列保持原樣（空的 → 用公式）
        if len(pts) == 1:
            print(f"⚠️ {row} 只有 1 點：ShotTable 要 2 點以上才查表，1 點時這列仍用公式。"
                  f"建議在同仰角再量一個距離，或用 ±0.3 m 補一點")
        if len({d for d, *_ in pts}) != len(pts):
            print(f"⚠️ {row} 有重複的距離，後面的會蓋掉前面的")
        pattern = re.compile(r"(static final double\[\]\[\] " + name + r" = \{\n)(.*?)(  \};)", re.S)
        if not pattern.search(src):
            raise SystemExit(f"❌ ShotTable.java 找不到 {name} 陣列")
        src = pattern.sub(lambda m: m.group(1) + render_rows(pts) + m.group(3), src, count=1)
    return src


def format_value(kind, raw):
    if kind == "sign":
        v = float(raw)
        if v not in (1.0, -1.0):
            raise SystemExit(f"❌ kYawSign 只能是 1 或 -1（填了 {raw}）")
        return f"{v:.1f}"
    if kind == "reference":
        v = raw.upper()
        if v not in ("GRID_WALL", "LOW_WALL"):
            raise SystemExit(f"❌ kRangeReference 只能是 GRID_WALL 或 LOW_WALL（填了 {raw}）")
        return f"RangeReference.{v}"
    return repr(float(raw))


def update_constant(src, name, value, path):
    pattern = re.compile(r"(public static final \w+ " + name + r" = )([^;]+)(;)")
    if not pattern.search(src):
        raise SystemExit(f"❌ {path} 找不到常數 {name}")
    return pattern.sub(lambda m: m.group(1) + value + m.group(3), src, count=1)


def strategy(hits):
    sys.path.insert(0, STRATEGY_DIR)
    import strategy_calc as sc

    bonus_hit, bonus_n = hits["BONUS"]
    hole_hit = sum(hits[r][0] for r in ("LOW", "MID", "HIGH"))
    hole_n = sum(hits[r][1] for r in ("LOW", "MID", "HIGH"))
    print("\n== 命中率（hits_of_5）==")
    for r, (h, n) in hits.items():
        print(f"  {r:5s} {h:2d}/{n:2d}" + (f" = {h / n:.0%}" if n else "  （沒量）"))
    if not bonus_n or not hole_n:
        print("命中率資料不夠（BONUS 和大洞都要有），跳過 strategy_calc")
        return
    p_bonus, p_hole = bonus_hit / bonus_n, hole_hit / hole_n
    rates = [hits[r][0] / hits[r][1] for r in ("LOW", "MID", "HIGH") if hits[r][1]]
    print(f"  加分洞 {p_bonus:.0%}、大洞合計 {p_hole:.0%}（最差一列 {min(rates):.0%}）")
    print("\n== 建議發數（strategy_calc.py，依大洞合計命中率）==")
    for key, h in sc.HALVES.items():
        n = sc.best_bonus_shots(p_bonus, p_hole, h["bonus_cap"], 1)
        print(f"  {h['name']}：/Tuning/Auto/{key}/BonusBalls = {n}   /Tuning/Auto/{key}/BallsPerHole = {h['hole_cap']}")
    print("  → 在 Elastic「賽前」分頁填，或改 AutoRoutines.Half 的預設值。完整表：")
    print(f"     python {os.path.relpath(os.path.join(STRATEGY_DIR, 'strategy_calc.py'))} "
          f"--bonus {bonus_hit}/{bonus_n} --hole {hole_hit}/{hole_n}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--shots", default=os.path.join(HERE, "shot_table_1115.csv"))
    parser.add_argument("--constants", default=os.path.join(HERE, "constants_1115.csv"))
    parser.add_argument("--dry-run", action="store_true", help="只印 diff，不寫檔")
    parser.add_argument("--yes", action="store_true", help="不問直接寫")
    args = parser.parse_args()

    points, hits, errors = read_shots(args.shots)
    if errors:
        print("❌ shot_table 有問題：\n  " + "\n  ".join(errors))
        raise SystemExit(1)

    edits = {}  # path → new source

    def current(path):
        if path not in edits:
            with open(path, encoding="utf-8") as f:
                edits[path] = f.read()
        return edits[path]

    edits[SHOT_TABLE] = update_shot_table(current(SHOT_TABLE), points)

    recorded = []
    for r in read_csv(args.constants):
        name, value = r.get("name", ""), r.get("value", "")
        if blank(name) or blank(value):
            continue
        if name in RECORD_ONLY:
            recorded.append(f"{name} = {value} {r.get('unit', '')}")
            continue
        if name not in CONSTANTS:
            print(f"⚠️ 不認得的常數 {name}，跳過")
            continue
        rel, java_name, kind = CONSTANTS[name]
        path = os.path.join(JAVA, rel)
        edits[path] = update_constant(current(path), java_name, format_value(kind, value), rel)

    changed = []
    for path, new in edits.items():
        with open(path, encoding="utf-8") as f:
            old = f.read()
        if old == new:
            continue
        changed.append(path)
        rel = os.path.relpath(path, REPO)
        sys.stdout.writelines(difflib.unified_diff(old.splitlines(True), new.splitlines(True), "a/" + rel, "b/" + rel))
    if recorded:
        print("\n== 只記錄（不改程式）==\n  " + "\n  ".join(recorded))
    strategy(hits)

    if not changed:
        print("\n沒有要改的地方（CSV 都是空的？）")
        return
    if args.dry_run:
        print(f"\n--dry-run：{len(changed)} 個檔案會被修改，沒有寫入")
        return
    if not args.yes and input(f"\n寫入 {len(changed)} 個檔案？[y/N] ").strip().lower() != "y":
        print("取消")
        return
    for path in changed:
        with open(path, "w", encoding="utf-8") as f:
            f.write(edits[path])
    print("已寫入。接著：./gradlew test → ./gradlew deploy")


if __name__ == "__main__":
    main()
