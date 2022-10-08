package net.cachemoney8096.frc2022o;

/** Store CAN and port IDs here */
public class RobotMap {
  /** Driver/Operator controller indices */
  public static final int DRIVER_CONTROLLER_INDEX = 0, OPERATOR_CONTROLLER_INDEX = 1;

  /** Drivetrain CAN IDs */
  public static final int DRIVE_FRONT_LEFT_ID = 1,
      STEER_FRONT_LEFT_ID = 2,
      DRIVE_FRONT_RIGHT_ID = 3,
      STEER_FRONT_RIGHT_ID = 4,
      DRIVE_BACK_LEFT_ID = 5,
      STEER_BACK_LEFT_ID = 6,
      DRIVE_BACK_RIGHT_ID = 7,
      STEER_BACK_RIGHT_ID = 8;

  /** Intake CAN IDs */
  public static final int INTAKE_MOTOR_ONE_ID = 9,
      INTAKE_MOTOR_TWO_ID = 10,
      INTAKE_MOTOR_THREE_ID = 11;

  /** Indexer CAN IDs */
  public static final int INDEXER_MOTOR_ONE_ID = 12,
      INDEXER_MOTOR_TWO_ID = 13,
      INDEXER_MOTOR_THREE_ID = 14;

  /** Shooter CAN IDs */
  public static final int SHOOTER_MOTOR_ONE_ID = 15, SHOOTER_MOTOR_TWO_ID = 16, HOOD_MOTOR_ID = 17;

  /** Climber CAN IDs */
  public static final int CLIMBER_MOTOR_RIGHT_ID = 18, CLIMBER_MOTOR_LEFT_ID = 19;

  /** CAN IDs for things other than motor controllers */
  public static final int PIGEON_IMU_ID = 20;

  /** Sensor DIO */
  public static final int SWERVE_FRONT_LEFT_DIO = 0,
      SWERVE_FRONT_RIGHT_DIO = 1,
      SWERVE_BACK_LEFT_DIO = 2,
      SWERVE_BACK_RIGHT_DIO = 3,
      INDEXER_CARGO_DIO = 4,
      INTAKE_CARGO_DIO = 5,
      HOOD_ENCODER_DIO = 6;

  /** Pneumatics Control Module (PCM) ports */
  public static final int COMPRESSOR_MODULE_ID = 0,
      LEFT_INTAKE_SOLENOID_CHANNEL = 0,
      RIGHT_INTAKE_SOLENOID_CHANNEL = 1;
}
