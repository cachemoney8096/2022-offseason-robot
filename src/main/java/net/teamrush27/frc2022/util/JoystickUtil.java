package net.cachemoney8096.frc2022o.util;

public class JoystickUtil {
  public static double deadband(double value, double zero) {
    if (Math.abs(value) < zero) {
      return 0;
    }

    return value;
  }
}
