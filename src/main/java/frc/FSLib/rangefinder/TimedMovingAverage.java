package frc.FSLib.rangefinder;

import java.util.ArrayDeque;

/**
 * 依時間窗的移動平均（Robot-Brief §8.3）：只平均最近 {@code windowSec} 秒內的「有效」讀值。
 * 讀值無效時不加入，但舊的有效讀值超過時間窗就會被丟掉。窗口內沒有有效值時回傳這一次的原始值。
 */
public final class TimedMovingAverage {
  private record Sample(double time, double value) {}

  private final double windowSec;
  private final ArrayDeque<Sample> samples = new ArrayDeque<>();
  private double sum = 0;

  public TimedMovingAverage(double windowSec) {
    this.windowSec = windowSec;
  }

  public double calculate(double timeSec, double value, boolean valid) {
    if (windowSec <= 0) return value;
    if (valid && Double.isFinite(value)) {
      samples.addLast(new Sample(timeSec, value));
      sum += value;
    }
    while (!samples.isEmpty() && samples.peekFirst().time() <= timeSec - windowSec) {
      sum -= samples.removeFirst().value();
    }
    if (!valid || samples.isEmpty()) return value;
    return sum / samples.size();
  }

  public void reset() {
    samples.clear();
    sum = 0;
  }
}
