package net.cachemoney8096.frc2022o;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Store physical / mathematical constants here */
public class Constants {
  /** Placeholder, if this value is used it's because we haven't figured out the right value yet */
  public static final double PLACEHOLDER_DOUBLE = 21.0;

  /** Placeholder, if this value is used it's because we haven't figured out the right value yet */
  public static final int PLACEHOLDER_INT = 21;

  /**
   * Swerve module encoder offsets (in radians!!). This is the output of the absolute encoder on the
   * module when the wheel is pointed forwards (towards intake).
   */
  public static final double FRONT_LEFT_STEER_OFFSET_RAD = PLACEHOLDER_DOUBLE,
      FRONT_RIGHT_STEER_OFFSET_RAD = PLACEHOLDER_DOUBLE,
      BACK_LEFT_STEER_OFFSET_RAD = PLACEHOLDER_DOUBLE,
      BACK_RIGHT_STEER_OFFSET_RAD = PLACEHOLDER_DOUBLE;

  // Constants for swerve
  public static final class Drivetrain {

    /** Drivetrain size */
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(20.5),
        WHEEL_BASE_METERS = Units.inchesToMeters(20.5);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

    public static final double WHEEL_DIAMETER_INCHES = 3.0;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
    public static final double DRIVE_REDUCTION = 4.8;
    public static final boolean DRIVE_INVERTED = false;
    public static final double STEER_REDUCTION = 11.25;
    public static final boolean STEER_INVERTED = true;

    public static final double DRIVE_ENCODER_POSITION_FACTOR =
        (WHEEL_DIAMETER_METERS * Math.PI) / (double) DRIVE_REDUCTION;

    public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
        ((WHEEL_DIAMETER_METERS * Math.PI) / (double) DRIVE_REDUCTION) / 60.0;

    public static final double STEERING_ENCODER_POSITION_FACTOR = (2 * Math.PI) / STEER_REDUCTION;
    public static final double STEERING_ENCODER_VELOCITY_FACTOR =
        ((2 * Math.PI) / STEER_REDUCTION) / 60.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER_METERS * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 3 * Math.PI;

    public static final int DRIVE_CURRENT_LIMIT_AMPS = 50;
  }

  /** Gear ratio from the hood encoder (external) to actual hood position */
  public static final double HOOD_EXTERNAL_ENCODER_RATIO = 8.0333;

  /** Gear ratio from motor to the driving gear shaft */
  public static final double HOOD_GEARBOX_RATIO = 18.9;

  /** Scalar for hood external encoder from [0,1] to real degrees */
  public static final double HOOD_EXTERNAL_ENCODER_SCALAR = 360 / HOOD_EXTERNAL_ENCODER_RATIO;

  /** Scalar for hood motor encoder from rotations to real degrees */
  public static final double HOOD_MOTOR_ENCODER_SCALAR =
      360 / (HOOD_GEARBOX_RATIO * HOOD_EXTERNAL_ENCODER_RATIO);

  /** Scalar for hood motor encoder from RPM to real degrees per seconds */
  public static final double HOOD_MOTOR_ENCODER_VELOCITY_SCALAR = HOOD_MOTOR_ENCODER_SCALAR / 60;

  /** Gear ratio from the shooter encoder (internal) to the shooter wheel position */
  public static final double SHOOTER_ENCODER_RATIO = 1.0;

  /**
   * Pitch angle from horizontal of the limelight in degrees (0 = straight ahead, 90 = straight up)
   */
  public static final double LIMELIGHT_PITCH_DEGREES = 36;

  /** Height of the limelight on the robot from the carpet in meters */
  public static final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(35.4);

  /** Height of the vision targets above the carpet */
  public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(103.0);
}
