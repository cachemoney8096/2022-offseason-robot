package net.cachemoney8096.frc2022o.libs_3005.util;

public class JoystickUtil {
  public static double squareAxis(double input) {
    return (input * input) * Math.signum(input);
  }
}
