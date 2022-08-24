// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2022;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /** Intake Constants **/
    public static final int INTAKE_FALCON_CAN_ID = 25;
    public static final int INTAKE_SOLENOID_EXTEND = 9;
    public static final int INTAKE_SOLENOID_RETRACT = 8;

    /** Hopper Constants **/
    public static final int HOPPER_SPARK_MAX_LEADER_CAN_ID = 26;
    public static final int HOPPER_SPARK_MAX_FOLLOWER_CAN_ID = 27;
    public static final int REAR_HOPPER_F500_CAN_ID = 27;
    public static final int HOPPER_SOLENOID_BRAKE = 10;
    public static final int HOPPER_DIO_CARGO_SENSOR_FRONT = 0;
    public static final int HOPPER_DIO_CARGO_SENSOR_MID = 1;
    public static final int HOPPER_DIO_CARGO_SENSOR_REAR = 2;
    public static final int HOPPER_DIO_CARGO_SENSOR_INDEXER = 3;

    /** Index Wheel Constants **/
    public static final int INDEX_WHEEL_SPARK_MAX_LEADER_CAN_ID = 28;
    public static final int INDEX_WHEEL_SPARK_MAX_FOLLOWER_CAN_ID = 29;

    public static final int PNEUMATIC_HUB_CAN_ID = 2;
    public static final int CANDLE_CAN_ID = 50;

    /** limelight constants **/
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24.25);
    public static final double UPPER_HUB_HEIGHT_METERS = Units.inchesToMeters(104);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);
    public static final double CAMERA_OFFSET = Units.inchesToMeters(12.6); //Camera offset from center of rotation

    /** climber constants **/
	public static final int CLIMBER_MOTOR_LEADER_CANID = 35;
    public static final int CLIMBER_MOTOR_FOLLOWER_CANID = 36;
    public static final int CLIMBER_ELEVATOR_RELEASE_ID = 11;
    public static final int CLIMBER_HOOK_REAR_OPEN = 15;
    public static final int CLIMBER_HOOK_REAR_CLOSE = 14;
    public static final int CLIMBER_HOOK_FRONT_OPEN = 13;
    public static final int CLIMBER_HOOK_FRONT_CLOSE = 12;

    /** launcher constants **/
    public static final int LAUNCHER_MAIN_ROLLER_LEADER_ID = 31;
    public static final int LAUNCHER_MAIN_ROLLER_FOLLOWER_ID = 30;
    public static final int LAUNCHER_HOOD_ID = 32;
    public static final int LAUNCHER_TOP_ROLLER_ID = 33;


    private static enum robotID {
        COMPETITION, PRACTICE, SWERVETEST
    }
    
    private static final robotID robotSelector = robotID.COMPETITION;

    public static double DRIVETRAIN_TRACKWIDTH_METERS;
    public static double DRIVETRAIN_WHEELBASE_METERS;


    public static int FRONT_LEFT_MODULE_DRIVE_MOTOR;
    public static int FRONT_LEFT_MODULE_STEER_MOTOR;
    public static int FRONT_LEFT_MODULE_STEER_ENCODER;
    public static double FRONT_LEFT_MODULE_STEER_OFFSET;

    public static int FRONT_RIGHT_MODULE_DRIVE_MOTOR;
    public static int FRONT_RIGHT_MODULE_STEER_MOTOR;
    public static int FRONT_RIGHT_MODULE_STEER_ENCODER;
    public static double FRONT_RIGHT_MODULE_STEER_OFFSET;

    public static int BACK_LEFT_MODULE_DRIVE_MOTOR;
    public static int BACK_LEFT_MODULE_STEER_MOTOR;
    public static int BACK_LEFT_MODULE_STEER_ENCODER;
    public static double BACK_LEFT_MODULE_STEER_OFFSET;

    public static int BACK_RIGHT_MODULE_DRIVE_MOTOR;
    public static int BACK_RIGHT_MODULE_STEER_MOTOR;
    public static int BACK_RIGHT_MODULE_STEER_ENCODER;
    public static double BACK_RIGHT_MODULE_STEER_OFFSET;

    public static int PIGEON_ID;
    
    public static void setConstants(){
        switch(robotSelector){
            case COMPETITION:
                DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21.5);
                DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

                FRONT_LEFT_MODULE_DRIVE_MOTOR = 10; // front left module drive motor ID
                FRONT_LEFT_MODULE_STEER_MOTOR = 11; // front left module steer motor ID
                FRONT_LEFT_MODULE_STEER_ENCODER = 18; // front left steer encoder ID
                FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-279.5+180); // front left steer offset

                FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12; // front right drive motor ID
                FRONT_RIGHT_MODULE_STEER_MOTOR = 13; // front right steer motor ID
                FRONT_RIGHT_MODULE_STEER_ENCODER = 19; // front right steer encoder ID
                FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-83.93+180); // front right steer offset

                BACK_LEFT_MODULE_DRIVE_MOTOR = 14; // back left drive motor ID
                BACK_LEFT_MODULE_STEER_MOTOR = 15; // back left steer motor ID
                BACK_LEFT_MODULE_STEER_ENCODER = 20; // back left steer encoder ID
                BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-70.75+180); // back left steer offset

                BACK_RIGHT_MODULE_DRIVE_MOTOR = 16; // back right drive motor ID
                BACK_RIGHT_MODULE_STEER_MOTOR = 17; // back right steer motor ID
                BACK_RIGHT_MODULE_STEER_ENCODER = 21; // back right steer encoder ID
                BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-160.1+180); // back right steer offset

                PIGEON_ID = 9;
                break;
            case PRACTICE:
                DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21.5);
                DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

                FRONT_LEFT_MODULE_DRIVE_MOTOR = 10; // front left module drive motor ID
                FRONT_LEFT_MODULE_STEER_MOTOR = 11; // front left module steer motor ID
                FRONT_LEFT_MODULE_STEER_ENCODER = 18; // front left steer encoder ID
                FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-292.939+180); // front left steer offset

                FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12; // front right drive motor ID
                FRONT_RIGHT_MODULE_STEER_MOTOR = 13; // front right steer motor ID
                FRONT_RIGHT_MODULE_STEER_ENCODER = 19; // front right steer encoder ID
                FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-131.572+180); // front right steer offset

                BACK_LEFT_MODULE_DRIVE_MOTOR = 14; // back left drive motor ID
                BACK_LEFT_MODULE_STEER_MOTOR = 15; // back left steer motor ID
                BACK_LEFT_MODULE_STEER_ENCODER = 20; // back left steer encoder ID
                BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-208.839+180); // back left steer offset

                BACK_RIGHT_MODULE_DRIVE_MOTOR = 16; // back right drive motor ID
                BACK_RIGHT_MODULE_STEER_MOTOR = 17; // back right steer motor ID
                BACK_RIGHT_MODULE_STEER_ENCODER = 21; // back right steer encoder ID
                BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-102.744+180); // back right steer offset

                PIGEON_ID = 9;
                break;
            case SWERVETEST: 
                DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.25);
                DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.125);

                FRONT_LEFT_MODULE_DRIVE_MOTOR = 17; // front left module drive motor ID
                FRONT_LEFT_MODULE_STEER_MOTOR = 18; // front left module steer motor ID
                FRONT_LEFT_MODULE_STEER_ENCODER = 34; // front left steer encoder ID
                FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(55.195); // front left steer offset

                FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11; // front right drive motor ID
                FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // front right steer motor ID
                FRONT_RIGHT_MODULE_STEER_ENCODER = 31; // front right steer encoder ID
                FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(67.939); // front right steer offset

                BACK_LEFT_MODULE_DRIVE_MOTOR = 15; // back left drive motor ID
                BACK_LEFT_MODULE_STEER_MOTOR = 16; // back left steer motor ID
                BACK_LEFT_MODULE_STEER_ENCODER = 33; // back left steer encoder ID
                BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(123.75); // back left steer offset

                BACK_RIGHT_MODULE_DRIVE_MOTOR = 13; // back right drive motor ID
                BACK_RIGHT_MODULE_STEER_MOTOR = 14; // back right steer motor ID
                BACK_RIGHT_MODULE_STEER_ENCODER = 32; // back right steer encoder ID
                BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-127.969); // back right steer offset
                break;
            }

    }
}
