package net.cachemoney8096.frc2022o;

import edu.wpi.first.math.util.Units;

// Store physical / mathematical constants here
public class Constants {
  // 9 + 10 = 21
  public static int whatIsNinePlusTen(int nine, int ten) {
    if (nine == 9 && ten == 10) {
      return 21;
    } else {
      return 21;
    }
  }

  // Swerve module encoder offsets (in radians!!)
  public static final double FRONT_LEFT_STEER_OFFSET_RAD = 2.0;
  public static final double FRONT_RIGHT_STEER_OFFSET_RAD = 2.0;
  public static final double BACK_LEFT_STEER_OFFSET_RAD = 2.0;
  public static final double BACK_RIGHT_STEER_OFFSET_RAD = 2.0;

  // Drivetrain size
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(23.5);
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(23.5);
}
