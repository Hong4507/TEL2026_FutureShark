#!/usr/bin/env python3
"""NetworkTables 連線測試：Jetson 每秒發布一次 JetsonTest/heartbeat。

用法：
  連 SystemCore（依隊號找）：  python nt_test.py --team 7130
  連指定 IP（例如筆電模擬）：  python nt_test.py --server 192.168.1.50

在 AdvantageScope / OutlineViewer 看到 JetsonTest/heartbeat 一直增加，就代表通了。
按 Ctrl+C 結束。
"""
import argparse
import time

import ntcore


def configure_nt_server(inst, team, server):
    """設定 NT 伺服器位址。

    指定 --server 時只連那個 IP。否則依序嘗試 10.TE.AM.2 與 robot.local。
    舊版 pyntcore 的 setServerTeam 只會找 roboRIO 的主機名稱，找不到 SystemCore，所以這裡自己列。
    回傳一段描述文字，用來顯示。
    """
    if server:
        inst.setServer(server)
        return server
    servers = [f"10.{team // 100}.{team % 100}.2", "robot.local"]
    inst.setServer([(s, 5810) for s in servers])
    return " / ".join(servers)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--team", type=int, default=7130, help="隊號，用來找主控制器")
    parser.add_argument("--server", type=str, default=None, help="直接指定 NT 伺服器 IP（優先於 --team）")
    args = parser.parse_args()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("jetson-nt-test")
    target = configure_nt_server(inst, args.team, args.server)

    table = inst.getTable("JetsonTest")
    heartbeat_pub = table.getIntegerTopic("heartbeat").publish()
    time_pub = table.getDoubleTopic("jetson_time").publish()

    print(f"連線目標：{target}，按 Ctrl+C 結束")
    count = 0
    try:
        while True:
            heartbeat_pub.set(count)
            time_pub.set(time.time())
            if inst.isConnected():
                conns = ", ".join(f"{c.remote_ip}:{c.remote_port}" for c in inst.getConnections())
                print(f"[已連線] heartbeat={count}  伺服器={conns}")
            else:
                print(f"[未連線] heartbeat={count}  正在嘗試連線 {target} ...")
            count += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n結束")
    finally:
        inst.stopClient()


if __name__ == "__main__":
    main()
