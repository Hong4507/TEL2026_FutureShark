package frc.robot.controller;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

class IBusParserTest {
  private static byte[] frame(int... channels) {
    byte[] f = new byte[IBusSerialReader.kFrameLength];
    f[0] = 0x20;
    f[1] = 0x40;
    for (int i = 0; i < IBusSerialReader.kNumChannels; i++) {
      int v = i < channels.length ? channels[i] : 1500;
      f[2 + 2 * i] = (byte) v;
      f[3 + 2 * i] = (byte) (v >> 8);
    }
    int sum = 0;
    for (int i = 0; i < 30; i++) sum += f[i] & 0xFF;
    int checksum = 0xFFFF - (sum & 0xFFFF);
    f[30] = (byte) checksum;
    f[31] = (byte) (checksum >> 8);
    return f;
  }

  private static int[] expected(int... first) {
    int[] out = new int[IBusSerialReader.kNumChannels];
    for (int i = 0; i < out.length; i++) out[i] = i < first.length ? first[i] : 1500;
    return out;
  }

  @Test
  void parsesSingleFrame() {
    var parser = new IBusSerialReader.IBusParser();
    byte[] f = frame(1000, 2000, 1500, 1234, 1000, 2000);
    List<int[]> frames = parser.feed(f, f.length);
    assertEquals(1, frames.size());
    assertArrayEquals(expected(1000, 2000, 1500, 1234, 1000, 2000), frames.get(0));
  }

  @Test
  void resyncsAfterGarbageAndSplitReads() {
    var parser = new IBusSerialReader.IBusParser();
    byte[] f = frame(1100, 1900);
    byte[] stream = new byte[3 + f.length * 2];
    stream[0] = 0x20; // 假標頭
    stream[1] = 0x55;
    stream[2] = 0x40;
    System.arraycopy(f, 0, stream, 3, f.length);
    System.arraycopy(f, 0, stream, 3 + f.length, f.length);

    // 一次只餵 5 bytes，模擬序列埠零碎的讀取
    List<int[]> frames = new ArrayList<>();
    for (int i = 0; i < stream.length; i += 5) {
      int n = Math.min(5, stream.length - i);
      byte[] chunk = new byte[n];
      System.arraycopy(stream, i, chunk, 0, n);
      frames.addAll(parser.feed(chunk, n));
    }
    assertEquals(2, frames.size());
    assertArrayEquals(expected(1100, 1900), frames.get(1));
  }

  @Test
  void rejectsBadChecksum() {
    var parser = new IBusSerialReader.IBusParser();
    byte[] f = frame(1000);
    f[30] ^= 0x01;
    assertEquals(0, parser.feed(f, f.length).size());
    assertEquals(1, parser.getBadFrames());
  }
}
