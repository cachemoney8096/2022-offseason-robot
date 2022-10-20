package net.cachemoney8096.frc2022o;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import net.cachemoney8096.frc2022o.libs_3005.controller.PIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import net.cachemoney8096.frc2022o.libs_3005.util.LinearInterpolatedTable2d;

/**
 * Use this class to store numbers that are arbitrary but can be tuned (such as thresholds, etc.)
 */
public class Calibrations {
  public static final class Drivetrain {
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD =
        new SimpleMotorFeedforward(0.0467, 3.3076, 0.01897);
    public static final SimpleMotorFeedforward STEER_FEEDFORWARD =
        new SimpleMotorFeedforward(0.35233, 0.39185, 0.0058658);

    public static final PIDGains STEER_PID_GAINS = new PIDGains(8.0, 0.00, 0.3);
    public static final PIDGains DRIVE_PID_GAINS = new PIDGains(0.04, 0.0, 0.0);

    public static final TrapezoidProfile.Constraints STEER_TRAPEZOID_CONSTRAINTS =
        new TrapezoidProfile.Constraints(20, 200);

    /** Auton path finding controllers */
    public static final PIDController PATH_X_CONTROLLER = new PIDController(0.100506, 0.0, 0.0),
    PATH_Y_CONTROLLER = new PIDController(0.1, 0.0, 0.0);

    /** High profile constraints = pure P controller */
    public static final ProfiledPIDController PATH_THETA_CONTROLLER =
        new ProfiledPIDController(
            9.0, 0.0, 0.80, new TrapezoidProfile.Constraints(1000.0, 100000.0));

    /** Controller on module speed for rotating to target
     * Input degrees, output [0,1]
    */
    public static final PIDController ROTATE_TO_TARGET_PID_CONTROLLER = new PIDController(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);
  }

  /** For translation commands above this threshold (in [0,1]), heading lock will not apply */
  public static final double HEADING_LOCK_TRANSLATION_COMMAND_THRESHOLD = 0.1;
  /** For rotation commands below this threshold (in [0,1]), heading lock may apply */
  public static final double HEADING_LOCK_ROTATION_COMMAND_THRESHOLD = 0.5;

  /** How long (in seconds) to eject cargo to the front before returning to intaking */
  public static final double EJECT_CARGO_FRONT_SECONDS = 1.0;

  /** How long (in seconds) to eject cargo to the front before returning to intaking */
  public static final double EJECT_CARGO_BACK_SECONDS = 1.0;

  /** Threshold for proximity from color sensor to consider there to be something there */
  public static final int COLOR_SENSOR_PROXIMITY_THRESHOLD = 100;

  /** Power for intake/indexer motors in [-1.0, 1.0] */
  public static final double INTAKE_ONE_POWER = 1.0,
      INTAKE_TWO_POWER = 0.5,
      INTAKE_EJECT_POWER = -1.0,
      INDEXER_ONE_POWER = 0.5,
      INDEXER_THREE_POWER = 0.5,
      INDEXER_EJECT_POWER = -1.0;

  /** Shooter PID */
  public static final double SHOOTER_kP = Constants.PLACEHOLDER_DOUBLE,
      SHOOTER_kI = Constants.PLACEHOLDER_DOUBLE,
      SHOOTER_kD = Constants.PLACEHOLDER_DOUBLE,
      SHOOTER_kF = Constants.PLACEHOLDER_DOUBLE,
      SHOOTER_RANGE_RPM = Constants.PLACEHOLDER_DOUBLE;

  /** Hood PID */
  public static final double HOOD_kP = Constants.PLACEHOLDER_DOUBLE,
      HOOD_kI = Constants.PLACEHOLDER_DOUBLE,
      HOOD_kD = Constants.PLACEHOLDER_DOUBLE,
      HOOD_kF = Constants.PLACEHOLDER_DOUBLE,
      HOOD_POSITION_TOLERANCE_DEG = Constants.PLACEHOLDER_DOUBLE;

  /**
   * Hood setpoint table for shooter. X = target linear distance from limelight Y = Hood setpoint in
   * degrees
   */
  public static final LinearInterpolatedTable2d HOOD_TABLE =
      new LinearInterpolatedTable2d()
          .withPair(0.0, 1.0)
          .withPair(1.0, 3.0)
          .withPair(0.5, 2.0)
          .withPair(2.0, 5.0);

  /**
   * Shooter speed setpoint table for shooter. X = target linear distance from limelight Y = Shooter
   * speed in RPM
   */
  public static final LinearInterpolatedTable2d SHOOTER_TABLE =
      new LinearInterpolatedTable2d()
          .withPair(0.0, 1.0)
          .withPair(1.0, 3.0)
          .withPair(0.5, 2.0)
          .withPair(2.0, 5.0);

  /** Tolerance for angle to goal for shooting (in degrees) */
  public static final double SHOOTER_TARGET_ALIGNMENT_TOLERANCE_DEG = 5.0;
}
