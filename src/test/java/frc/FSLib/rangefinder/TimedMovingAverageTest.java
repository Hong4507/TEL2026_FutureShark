package frc.FSLib.rangefinder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TimedMovingAverageTest {
  @Test
  void averagesWithinWindow() {
    TimedMovingAverage f = new TimedMovingAverage(0.2);
    assertEquals(1.0, f.calculate(0.00, 1.0, true), 1e-12);
    assertEquals(1.5, f.calculate(0.05, 2.0, true), 1e-12);
    assertEquals(2.0, f.calculate(0.10, 3.0, true), 1e-12);
    // t = 0.26：t=0 和 t=0.05 掉出窗口
    assertEquals(3.5, f.calculate(0.26, 4.0, true), 1e-12);
  }

  @Test
  void invalidReadingsAreNotAveraged() {
    TimedMovingAverage f = new TimedMovingAverage(0.2);
    f.calculate(0.0, 2.0, true);
    assertEquals(0.0, f.calculate(0.01, 0.0, false), 1e-12); // 無效時回傳原值（呼叫端看 valid）
    assertEquals(2.5, f.calculate(0.02, 3.0, true), 1e-12);  // 0 沒被算進去
  }

  @Test
  void zeroWindowPassesThrough() {
    TimedMovingAverage f = new TimedMovingAverage(0);
    assertEquals(7.0, f.calculate(0, 7.0, true), 1e-12);
    assertEquals(8.0, f.calculate(0.01, 8.0, true), 1e-12);
  }
}
