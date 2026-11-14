package frc.robot.controller;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * 直接在 SystemCore 上讀 FlySky 接收機的 i-BUS（經 USB 轉 TTL 模組，Linux 上是 /dev/ttyUSB0 之類）。
 *
 * <p>WPILib 2027 alpha 的 {@code SerialPort} 在 SystemCore 上還沒實作（HAL 全部回傳錯誤），
 * 所以這裡用 {@code stty} 設定 115200 8N1 raw，再用 {@link FileInputStream} 直接讀裝置檔。
 * 讀取在背景執行緒進行；USB 被拔掉會每 {@link #kRetryPeriodSeconds} 秒重新嘗試開啟。
 *
 * <p>接線：接收機 i-BUS 訊號腳 → USB-TTL 的 RX；GND 共地；接收機 5V 由 USB-TTL 的 5V 供電。TX 不用接。
 */
public class IBusSerialReader implements AutoCloseable {
  /** i-BUS 封包長度：0x20 0x40 + 14 通道 (uint16 LE) + checksum (uint16 LE)。 */
  static final int kFrameLength = 32;
  static final int kNumChannels = 14;
  static final int kBaudRate = 115200;
  static final double kRetryPeriodSeconds = 1.0;

  /** 一個解出來的 i-BUS 封包。 */
  public record Frame(int[] channels, long receivedNanos) {
    /** 封包收到到現在經過的秒數。 */
    public double ageSeconds() {
      return (System.nanoTime() - receivedNanos) / 1e9;
    }
  }

  private final String[] portCandidates;
  private final Thread thread;
  private volatile boolean running = true;
  private volatile Frame latest = null;
  private volatile String openPort = null;
  private volatile InputStream openStream = null;
  private volatile long badFrames = 0;
  private volatile long frameCount = 0;

  /**
   * @param portCandidates 依序嘗試的序列埠裝置檔，例如 {@code /dev/ttyUSB0}
   */
  public IBusSerialReader(String... portCandidates) {
    this.portCandidates = portCandidates.clone();
    thread = new Thread(this::run, "IBusSerialReader");
    thread.setDaemon(true);
    thread.start();
  }

  /** 最新一個正確的封包，還沒收到過時為 null。 */
  public Frame getLatestFrame() {
    return latest;
  }

  /** 目前開著的序列埠，沒開成功時為 null。 */
  public String getOpenPort() {
    return openPort;
  }

  /** checksum 錯誤的封包數（接觸不良、鮑率錯時會一直增加）。 */
  public long getBadFrames() {
    return badFrames;
  }

  /** 收到的正確封包總數。 */
  public long getFrameCount() {
    return frameCount;
  }

  @Override
  public void close() {
    running = false;
    closeStream();
    thread.interrupt();
  }

  private void run() {
    IBusParser parser = new IBusParser();
    byte[] buf = new byte[64];
    String lastError = null;
    while (running) {
      String port = findPort();
      if (port == null) {
        lastError = logOnce(lastError, "[FlySky] 找不到 USB-TTL 序列埠（試過 " + String.join(", ", portCandidates) + "）");
        sleepRetry();
        continue;
      }
      try {
        configurePort(port);
        try (InputStream in = new FileInputStream(port)) {
          openStream = in;
          openPort = port;
          parser.reset();
          lastError = null;
          System.out.println("[FlySky] 已開啟 " + port + "，讀取 i-BUS");
          while (running) {
            int n = in.read(buf);
            if (n < 0) throw new IOException("序列埠關閉（USB 被拔掉？）");
            for (int[] channels : parser.feed(buf, n)) {
              latest = new Frame(channels, System.nanoTime());
              frameCount++;
            }
            badFrames = parser.getBadFrames();
          }
        }
      } catch (IOException | InterruptedException e) {
        if (!running) break;
        lastError = logOnce(lastError, "[FlySky] " + port + " 讀取失敗：" + e.getMessage());
      } finally {
        openStream = null;
        openPort = null;
      }
      sleepRetry();
    }
  }

  private String findPort() {
    for (String p : portCandidates) {
      if (new File(p).exists()) return p;
    }
    return null;
  }

  /**
   * 用 stty 設成 115200 8N1、raw、不等數據機訊號（clocal）、至少收到 1 byte 才回傳。
   * 每次開啟前都要做，因為 USB 重新插上後設定會回到預設值。
   */
  private static void configurePort(String port) throws IOException, InterruptedException {
    Process p = new ProcessBuilder(
        "stty", "-F", port, Integer.toString(kBaudRate),
        "cs8", "-cstopb", "-parenb", "raw", "-echo", "clocal", "-crtscts", "-ixon", "-ixoff",
        "min", "1", "time", "0")
        .redirectErrorStream(true)
        .start();
    String output = new String(p.getInputStream().readAllBytes()).trim();
    if (!p.waitFor(2, TimeUnit.SECONDS)) {
      p.destroyForcibly();
      throw new IOException("stty 逾時");
    }
    if (p.exitValue() != 0) {
      throw new IOException("stty 失敗（權限不足？）：" + output);
    }
  }

  private void closeStream() {
    InputStream in = openStream;
    if (in != null) {
      try {
        in.close();
      } catch (IOException ignored) {
        // 關閉時的錯誤不重要
      }
    }
  }

  private void sleepRetry() {
    try {
      Thread.sleep((long) (kRetryPeriodSeconds * 1000));
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      running = false;
    }
  }

  /** 同樣的錯誤只印一次，避免每秒洗版。 */
  private static String logOnce(String last, String message) {
    if (!message.equals(last)) System.err.println(message);
    return message;
  }

  /** 把序列埠收到的位元組切成 i-BUS 封包並驗證 checksum。和 jetson_tools/flysky_nt_publisher.py 相同邏輯。 */
  static final class IBusParser {
    private final byte[] buf = new byte[kFrameLength * 8];
    private int len = 0;
    private long badFrames = 0;

    void reset() {
      len = 0;
    }

    long getBadFrames() {
      return badFrames;
    }

    /** 餵入新的位元組，回傳這次解出來的所有封包（每個封包是 14 個通道的 μs 值）。 */
    List<int[]> feed(byte[] data, int count) {
      List<int[]> frames = new ArrayList<>();
      int offset = 0;
      while (offset < count) {
        int n = Math.min(count - offset, buf.length - len);
        System.arraycopy(data, offset, buf, len, n);
        len += n;
        offset += n;
        parse(frames);
      }
      return frames;
    }

    private void parse(List<int[]> frames) {
      int start = 0;
      while (len - start >= kFrameLength) {
        if ((buf[start] & 0xFF) != 0x20 || (buf[start + 1] & 0xFF) != 0x40) {
          start++; // 不同步，往後找標頭
          continue;
        }
        int sum = 0;
        for (int i = 0; i < 30; i++) sum += buf[start + i] & 0xFF;
        int checksum = 0xFFFF - (sum & 0xFFFF);
        int received = (buf[start + 30] & 0xFF) | ((buf[start + 31] & 0xFF) << 8);
        if (checksum != received) {
          badFrames++;
          start++;
          continue;
        }
        int[] channels = new int[kNumChannels];
        for (int i = 0; i < kNumChannels; i++) {
          int lo = buf[start + 2 + 2 * i] & 0xFF;
          int hi = buf[start + 3 + 2 * i] & 0xFF;
          channels[i] = (lo | (hi << 8)) & 0x0FFF;
        }
        frames.add(channels);
        start += kFrameLength;
      }
      System.arraycopy(buf, start, buf, 0, len - start);
      len -= start;
    }
  }
}
