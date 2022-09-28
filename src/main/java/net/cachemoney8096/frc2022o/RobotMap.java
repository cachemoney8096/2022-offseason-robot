package net.cachemoney8096.frc2022o;

// Store CAN and port IDs here
public class RobotMap {
  // Drivetrain CAN IDs
  public static final int DRIVE_FRONT_LEFT_ID = 1;
  public static final int STEER_FRONT_LEFT_ID = 2;
  public static final int DRIVE_FRONT_RIGHT_ID = 3;
  public static final int STEER_FRONT_RIGHT_ID = 4;
  public static final int DRIVE_BACK_LEFT_ID = 5;
  public static final int STEER_BACK_LEFT_ID = 6;
  public static final int DRIVE_BACK_RIGHT_ID = 7;
  public static final int STEER_BACK_RIGHT_ID = 8;

  // Intake CAN IDs
  public static final int INTAKE_MOTOR_ONE_ID = 9;
  public static final int INTAKE_MOTOR_TWO_ID = 10;
  public static final int INTAKE_MOTOR_THREE_ID = 11;

  // Indexer CAN IDs
  public static final int INDEXER_MOTOR_ONE_ID = 12;
  public static final int INDEXER_MOTOR_TWO_ID = 13;
  public static final int INDEXER_MOTOR_THREE_ID = 14;

  // Shooter CAN IDs
  public static final int SHOOTER_MOTOR_ONE_ID = 15;
  public static final int SHOOTER_MOTOR_TWO_ID = 16;
  public static final int HOOD_MOTOR_ID = 17;

  // CAN IDs for things other than motor controllers
  public static final int PIGEON_IMU_ID = 18;

  // Sensor DIO
  public static final int SWERVE_FRONT_LEFT_DIO = 0;
  public static final int SWERVE_FRONT_RIGHT_DIO = 1;
  public static final int SWERVE_BACK_LEFT_DIO = 2;
  public static final int SWERVE_BACK_RIGHT_DIO = 3;
  public static final int INDEXER_CARGO_DIO = 4;
  public static final int INTAKE_CARGO_DIO = 5;
  public static final int HOOD_ENCODER_DIO = 6;

  // Pneumatics Control Module (PCM) ports
  public static final int COMPRESSOR_MODULE_ID = 0;
  public static final int LEFT_INTAKE_SOLENOID_CHANNEL = 0;
  public static final int RIGHT_INTAKE_SOLENOID_CHANNEL = 1;
}
