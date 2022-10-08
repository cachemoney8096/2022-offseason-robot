package net.cachemoney8096.frc2022o;

/**
 * Use this class to store numbers that are arbitrary but can be tuned (such as
 * thresholds, etc.)
 */
public class Calibrations {
  /**
   * For translation commands above this threshold (in [0,1]), heading lock will
   * not apply
   */
  public static final double HEADING_LOCK_TRANSLATION_COMMAND_THRESHOLD = 0.1;
  /**
   * For rotation commands below this threshold (in [0,1]), heading lock may apply
   */
  public static final double HEADING_LOCK_ROTATION_COMMAND_THRESHOLD = 0.5;

  /**
   * How long (in seconds) to eject cargo to the front before returning to
   * intaking
   */
  public static final double EJECT_CARGO_FRONT_SECONDS = 1.0;

  /**
   * How long (in seconds) to eject cargo to the front before returning to
   * intaking
   */
  public static final double EJECT_CARGO_BACK_SECONDS = 1.0;

  /**
   * Threshold for proximity from color sensor to consider there to be something
   * there
   */
  public static final int COLOR_SENSOR_PROXIMITY_THRESHOLD = Constants.PLACEHOLDER_INT;

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
      HOOD_RANGE_DEG = Constants.PLACEHOLDER_DOUBLE;
}
