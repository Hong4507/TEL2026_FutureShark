#!/usr/bin/env python3
"""抽稀存圖：每秒 2.5 張，相鄰畫面幾乎一樣，全部人工修正太浪費。先抽稀再自動標註／人工修正。

每個錄影時段（相鄰兩張間隔 > --gap 秒就算新時段，和 make_dataset.py 一樣）：
  1. 每隔 --min-dt 秒留一張；
  2. 留下來的超過 --max-per-session 張，就再平均挑到那個數量（避免某一段，例如近距離，佔太多）。
沒留的圖（連同同名 .txt）**搬到** <images>_thinned/（保留子資料夾），不會刪除，要用時搬回來即可。

用法：
  python thin.py --images captures                     # 先看會留幾張（不動檔案）
  python thin.py --images captures --apply             # 真的搬
  python thin.py --images captures --min-dt 1.0 --max-per-session 500 --apply
"""
import argparse
import os
import shutil

from make_dataset import IMAGE_EXTS, sessions, timestamp


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--min-dt", type=float, default=1.2, help="兩張留下的圖至少隔幾秒（預設 1.2 ≈ 留 1/3）")
    parser.add_argument("--max-per-session", type=int, default=400, help="每個時段最多留幾張（0 = 不限）")
    parser.add_argument("--gap", type=float, default=60.0)
    parser.add_argument("--apply", action="store_true", help="真的搬檔案；沒加只顯示結果")
    args = parser.parse_args()

    root = os.path.normpath(args.images)
    out_root = root + "_thinned"
    images = [os.path.join(d, f) for d, _, fs in os.walk(root) for f in fs if f.lower().endswith(IMAGE_EXTS)]

    drop, total_keep = [], 0
    for g in sessions(images, args.gap):
        keep, last = [], None
        for p in g:
            t = timestamp(p)
            if t is None or last is None or (t - last).total_seconds() >= args.min_dt:
                keep.append(p)
                if t is not None:
                    last = t
        if args.max_per_session and len(keep) > args.max_per_session:
            n = args.max_per_session
            keep = [keep[round(i * (len(keep) - 1) / (n - 1))] for i in range(n)] if n > 1 else keep[:1]
        ks = set(keep)
        drop += [p for p in g if p not in ks]
        total_keep += len(keep)
        t0, t1 = timestamp(g[0]), timestamp(g[-1])
        span = f"{t0:%H:%M:%S}–{t1:%H:%M:%S}" if t0 else os.path.dirname(g[0])
        print(f"  時段 {span}  {len(g):5d} 張 → 留 {len(keep):4d}")

    print(f"共 {len(images)} 張 → 留 {total_keep} 張，搬走 {len(drop)} 張到 {out_root}/")
    if not args.apply:
        print("（只是預覽；確定後加 --apply）")
        return
    for p in drop:
        for src in (p, os.path.splitext(p)[0] + ".txt"):
            if os.path.exists(src):
                dst = os.path.join(out_root, os.path.relpath(src, root))
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.move(src, dst)
    print(f"完成。要全部搬回來：rsync -a --remove-source-files {out_root}/ {root}/")


if __name__ == "__main__":
    main()
