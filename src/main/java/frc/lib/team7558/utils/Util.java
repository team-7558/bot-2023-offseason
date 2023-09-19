package frc.lib.team7558.utils;

/** Contains some basic utility functions that are used often. */
public class Util {
  /** Prevent this class from being instantiated. */
  private Util() {}

  /**
   * limit a value to a given range
   *
   * @param val - Value to check limits on
   * @param maxMagnitude - Magnitude to check limits with
   * @return Value with affected limit
   */
  public static double limit(double val, double maxMagnitude) {
    return limit(val, -maxMagnitude, maxMagnitude);
  }

  /**
   * limit a value to a given range
   *
   * @param val - Value to check limits on
   * @param min - Min value to check lower limit with
   * @param max - Max value to check upper limit with
   * @return Value with affected limit
   */
  public static double limit(double val, double min, double max) {
    return Math.min(max, Math.max(min, val));
  }

  /**
   * Checks if a value is within a given range
   *
   * @param val - Value to check
   * @param maxMagnitude - Magnitude for range to check
   * @return If the value is in the given range
   */
  public static boolean inRange(double val, double maxMagnitude) {
    return inRange(val, -maxMagnitude, maxMagnitude);
  }

  /**
   * Checks if a value is within a given range
   *
   * @param val - Value to check
   * @param min - Min value on range
   * @param max - Max value on range
   * @return If the value is in the given range
   */
  public static boolean inRange(double val, double min, double max) {
    return val > min && val < max;
  }

  /**
   * Returns wether a or b is closer to a given value
   *
   * @param a - first value to compare
   * @param b - second value to compare
   * @param val - the value to compare others to
   * @return True if value a is closer than value b
   */
  public static boolean isCloser(double a, double b, double val) {
    return Math.abs(a - val) < Math.abs(b - val);
  }
}
