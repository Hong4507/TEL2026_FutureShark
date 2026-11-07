# Jetson 工具程式

全部在 Jetson 的虛擬環境中執行：

```bash
source ~/yolo-env/bin/activate
cd ~/vision
```

| 檔案 | 用途 |
|---|---|
| `nt_test.py` | 測試 Jetson ↔ SystemCore（或筆電模擬）的 NetworkTables 連線 |
| `camera_check.py` | 確認攝影機實際解析度、幀率，存一張截圖 |
| `record.py` | 錄製訓練用影片（SSH 下即可） |
| `calibrate.py` | 棋盤格相機校正：`capture` 拍照、`solve` 計算 |
| `flysky_nt_publisher.py` | 讀 FlySky 接收機 i-BUS，發布到 NT `/FlySky/channels`（μs）、`/FlySky/heartbeat`，給 `FlySkyController` 用 |
| `flysky.service` | 開機自動執行 `flysky_nt_publisher.py` 的 systemd 服務 |
| `chessboard_9x6_25mm.pdf` | 校正用棋盤格，A4 橫式，以「實際大小 / 100%」列印 |

## 常用指令

```bash
python nt_test.py --team 7130            # 連主控制器
python nt_test.py --server <筆電IP>       # 連筆電模擬
python camera_check.py --width 1280 --height 720
python record.py --name 3m_left30_normal
python calibrate.py capture --count 25 --square 25
python calibrate.py solve --square 25
python flysky_nt_publisher.py --print-only     # 只看通道數值，對照搖桿
python flysky_nt_publisher.py --team 7130      # 發布給主控制器
```

### FlySky 開機自動啟動

```bash
sudo usermod -aG dialout $USER                 # 讓使用者能開 /dev/ttyUSB0（重新登入生效）
pip install pyserial pyntcore
sudo cp flysky.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now flysky
journalctl -u flysky -f                        # 看輸出
```

`--square` 請填列印後用尺量到的方格實際邊長（mm）。
拍照、錄影、校正與比賽時，請使用同一個解析度。
