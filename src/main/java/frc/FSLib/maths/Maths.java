package frc.FSLib.maths;

public class Maths {
  public static double findNearestLinear(double[] arr, int target) {
    if (arr.length == 0) {
      return Double.NaN;
    }
    double closest = arr[0];
    double smallestDiff = Math.abs(target - arr[0]);

    for (int i = 1; i < arr.length; i++) {
      double currentDiff = Math.abs(target - arr[i]);
      if (currentDiff < smallestDiff) {
        smallestDiff = currentDiff;
        closest = arr[i];
      }
    }
    return closest;
  }
}
