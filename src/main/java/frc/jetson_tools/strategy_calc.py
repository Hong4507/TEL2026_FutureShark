#!/usr/bin/env python3
"""依實測命中率，算出自動流程「加分格射幾發」的建議值（規則：2026_rule_0903.pdf）。

用法（11/14 實測後）：
  python strategy_calc.py --bonus 6/10 --hole 14/20
  python strategy_calc.py --bonus 0.55 --hole 0.75

  --bonus  加分格（Ø20cm）命中率：小數，或「進球/發數」
  --hole   大洞（Ø40cm）命中率：同上（低中高混在一起量，或填最差的那個高度）
  --min-bonus  加分格至少射幾發（預設 1：同分評比第一項是「有沒有投進加分格」，數學上不划算時也保底）

算法：第 k 發打加分格多拿的期望分數 = 4 ×（前 k−1 發還沒進滿上限的機率）× 加分格命中率，
和「這顆球留給大洞」的期望分數 2 × 大洞命中率比較，划算就繼續加發數。
上半場球數（27）剛好等於大洞格數上限（9 洞 × 3），下半場（18）也是（9 洞 × 2），所以每多射一發加分格，
就少一顆球給大洞。大洞每洞的發數維持規則上限即可（多射的球改到別的洞期望值更高，換洞交給手動）。

下半場大洞另有連線分，這裡沒有算進去，所以下半場的建議值偏向多打加分格；連線很重要的話可以少 1 發。

算出來後在 NT 設定（比賽前 90 秒準備時間也能改，不用重新部署）：
  /Tuning/Auto/FIRST/BonusBalls、/Tuning/Auto/SECOND/BonusBalls
"""
import argparse
from math import comb

HALVES = {
    "FIRST": {"name": "上半場", "bonus_cap": 3, "hole_cap": 3, "balls": 27},
    "SECOND": {"name": "下半場", "bonus_cap": 2, "hole_cap": 2, "balls": 18},
}
BONUS_POINTS = 4
HOLE_POINTS = 2
MAX_SHOTS = 8


def parse_rate(text):
    if "/" in text:
        hit, total = text.split("/")
        return int(hit) / int(total)
    return float(text)


def expected_scored(n, p, cap):
    """射 n 發、命中率 p、最多算 cap 顆時，期望算分顆數。"""
    return sum(comb(n, k) * p**k * (1 - p) ** (n - k) * min(k, cap) for k in range(n + 1))


def best_bonus_shots(p_bonus, p_hole, cap, min_bonus):
    n = 0
    while n < MAX_SHOTS:
        gain = BONUS_POINTS * (expected_scored(n + 1, p_bonus, cap) - expected_scored(n, p_bonus, cap))
        if gain <= HOLE_POINTS * p_hole:
            break
        n += 1
    return max(n, min_bonus)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bonus", required=True, help="加分格命中率，例如 0.55 或 6/10")
    parser.add_argument("--hole", required=True, help="大洞命中率，例如 0.75 或 14/20")
    parser.add_argument("--min-bonus", type=int, default=1, help="加分格至少射幾發（同分評比保底）")
    args = parser.parse_args()

    p_bonus = parse_rate(args.bonus)
    p_hole = parse_rate(args.hole)
    print(f"加分格命中率 {p_bonus:.0%}、大洞命中率 {p_hole:.0%}\n")

    for key, h in HALVES.items():
        n = best_bonus_shots(p_bonus, p_hole, h["bonus_cap"], args.min_bonus)
        print(f"== {h['name']}：/Tuning/Auto/{key}/BonusBalls = {n}，BallsPerHole = {h['hole_cap']} ==")
        print("   加分格發數  期望加分格分數  期望全場分數（剩下的球全打大洞）  至少進 1 顆的機率")
        for shots in range(0, max(n, h["bonus_cap"]) + 3):
            bonus_pts = BONUS_POINTS * expected_scored(shots, p_bonus, h["bonus_cap"])
            hole_pts = HOLE_POINTS * p_hole * (h["balls"] - shots)
            any_hit = 1 - (1 - p_bonus) ** shots
            mark = "  ← 建議" if shots == n else ""
            print(f"   {shots:>6}      {bonus_pts:>8.1f}        {bonus_pts + hole_pts:>12.1f}"
                  f"                  {any_hit:>5.0%}{mark}")
        print()


if __name__ == "__main__":
    main()
