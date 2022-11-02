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
    /** For output value in volts, input m/s */
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD =
        new SimpleMotorFeedforward(0.0467, 2.4, 0.0);

    /** For output value in volts, input rad/s */
    public static final SimpleMotorFeedforward STEER_FEEDFORWARD =
        new SimpleMotorFeedforward(0.18233, 0.19, 0.0);

    /** For output value in [0,1], input rad/s */
    public static final PIDGains STEER_PID_GAINS = new PIDGains(4, 0.00, 0.05);
    
    /** For output value in [0,1], input m/s */
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

    /** Controller on module speed for rotating to target Input degrees, output [0,1] */
    public static final PIDController ROTATE_TO_TARGET_PID_CONTROLLER =
        new PIDController(
            Constants.PLACEHOLDER_DOUBLE,
            Constants.PLACEHOLDER_DOUBLE,
            Constants.PLACEHOLDER_DOUBLE);
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
      INTAKE_TWO_POWER = 0.75,
      INTAKE_EJECT_POWER = -1.0,
      INDEXER_ONE_POWER = 0.5,
      INDEXER_THREE_POWER = 0.5,
      INDEXER_EJECT_POWER = -1.0,
      INTAKE_BACKWARDS_POWER = -1.0,
      INDEXER_BACKWARDS_POWER = -1.0,
      INTAKE_FORWARDS_POWER = 1.0,
      INDEXER_FORWARDS_POWER = 1.0;

  /** Shooter PID */
  public static final double SHOOTER_kP = 0.0002,
      SHOOTER_kI = 0.000000001,
      SHOOTER_kD = 0,
      SHOOTER_kF = 0.000197,
      SHOOTER_RANGE_RPM = 80.0,
      SHOOTER_I_ZONE = 100;

  /** Hood PID */
  public static final double HOOD_kP = 0.2,
      HOOD_kI = 0.001,
      HOOD_kD = 0,
      HOOD_kF = 0,
      HOOD_IZone = 0.4,
      HOOD_POSITION_TOLERANCE_DEG = 0.3;

  /**
   * Hood setpoint table for shooter. X = target linear distance from limelight Y = Hood setpoint in
   * degrees
   */
  public static final LinearInterpolatedTable2d HOOD_TABLE =
      new LinearInterpolatedTable2d()
      .withPair(1.0, 11.0)
      .withPair(1.25, 12.0)
      .withPair(1.50, 16.0)
      .withPair(1.75, 18.0)
      .withPair(2.0, 20.0)
      .withPair(2.25, 24.0)
      .withPair(2.5, 25.0)
      .withPair(2.75, 26.0)
      .withPair(3.0, 27.0)
      .withPair(3.25, 28.0)
      .withPair(3.5, 29.0)
      .withPair(3.75, 31.0)
      .withPair(4.0, 32.0)
      .withPair(4.25, 33.0)
      .withPair(4.5, 34.0)
      .withPair(4.75, 34.0)
      .withPair(5.0, 34.0)
      .withPair(5.5, 35.0)
      .withPair(6.0, 35.0)
      .withPair(6.5, 35.0)
      .withPair(7.0, 35.0);

  /**
   * Shooter speed setpoint table for shooter. X = target linear distance from limelight Y = Shooter
   * speed in RPM
   */
  public static final LinearInterpolatedTable2d SHOOTER_TABLE =
      new LinearInterpolatedTable2d()
        .withPair(1.0, 2050)
        .withPair(1.25, 2050)
        .withPair(1.50, 2200)
        .withPair(1.75, 2200)
        .withPair(2.0, 2200)
        .withPair(2.25, 2200)
        .withPair(2.5, 2300)
        .withPair(2.75, 2300)
        .withPair(3.0, 2350)
        .withPair(3.25, 2400)
        .withPair(3.5, 2420)
        .withPair(3.75, 2450)
        .withPair(4.0, 2500)
        .withPair(4.25, 2520)
        .withPair(4.5, 2600)
        .withPair(4.75, 2700)
        .withPair(5.0, 2710)
        .withPair(5.5, 2780)
        .withPair(6.0, 2900)
        .withPair(6.5, 3000)
        .withPair(7.0, 3150);

  /** Tolerance for angle to goal for shooting (in degrees) */
  public static final double SHOOTER_TARGET_ALIGNMENT_TOLERANCE_DEG = 5.0;
}
