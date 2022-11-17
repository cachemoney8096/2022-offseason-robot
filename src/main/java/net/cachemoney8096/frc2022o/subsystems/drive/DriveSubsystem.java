package net.cachemoney8096.frc2022o.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import net.cachemoney8096.frc2022o.libs_3005.swerve.SwerveDrive;
import net.cachemoney8096.frc2022o.libs_3005.swerve.SwerveModule;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.ThroughBoreEncoder;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.libs.SendablePigeon;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.Limelight;

public class DriveSubsystem extends SwerveDrive {
  private final Limelight limelight;

  public Command trajectoryFollowerCommand(PathPlannerTrajectory trajectory) {
    Calibrations.Drivetrain.PATH_THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    return trajectoryFollowerCommand(
        trajectory,
        Calibrations.Drivetrain.PATH_X_CONTROLLER,
        Calibrations.Drivetrain.PATH_Y_CONTROLLER,
        Calibrations.Drivetrain.PATH_THETA_CONTROLLER);
  }

  static final SwerveModule frontLeft =
      new NickSwerveModule(
          RobotMap.DRIVE_FRONT_LEFT_ID,
          RobotMap.STEER_FRONT_LEFT_ID,
          new ThroughBoreEncoder(
              RobotMap.SWERVE_FRONT_LEFT_DIO,
              Constants.FRONT_LEFT_STEER_OFFSET_RAD,
              2 * Math.PI,
              true));

  static final SwerveModule frontRight =
      new NickSwerveModule(
          RobotMap.DRIVE_FRONT_RIGHT_ID,
          RobotMap.STEER_FRONT_RIGHT_ID,
          new ThroughBoreEncoder(
              RobotMap.SWERVE_FRONT_RIGHT_DIO,
              Constants.FRONT_RIGHT_STEER_OFFSET_RAD,
              2 * Math.PI,
              true));

  static final SwerveModule rearLeft =
      new NickSwerveModule(
          RobotMap.DRIVE_BACK_LEFT_ID,
          RobotMap.STEER_BACK_LEFT_ID,
          new ThroughBoreEncoder(
              RobotMap.SWERVE_BACK_LEFT_DIO,
              Constants.BACK_LEFT_STEER_OFFSET_RAD,
              2 * Math.PI,
              true));

  static final SwerveModule rearRight =
      new NickSwerveModule(
          RobotMap.DRIVE_BACK_RIGHT_ID,
          RobotMap.STEER_BACK_RIGHT_ID,
          new ThroughBoreEncoder(
              RobotMap.SWERVE_BACK_RIGHT_DIO,
              Constants.BACK_RIGHT_STEER_OFFSET_RAD,
              2 * Math.PI,
              true));

  public DriveSubsystem(SendablePigeon gyro, Limelight limelightIn) {
    super(
        frontLeft,
        frontRight,
        rearLeft,
        rearRight,
        Constants.Drivetrain.SWERVE_KINEMATICS,
        gyro,
        Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
    limelight = limelightIn;
    // Logger.tag("Swerve Drive").warn("Reset calibration time back to longer for comp");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("X Controller", Calibrations.Drivetrain.PATH_X_CONTROLLER);
    addChild("Y Controller", Calibrations.Drivetrain.PATH_Y_CONTROLLER);
    addChild("Theta Controller", Calibrations.Drivetrain.PATH_THETA_CONTROLLER);
    addChild("Rotate to target controller", Calibrations.Drivetrain.ROTATE_TO_TARGET_PID_CONTROLLER);
    builder.addBooleanProperty("Aligned to Target", this::alignedToTarget, null);
  }

  /**
   * Rotates the robot towards the target while following some desired translation
   *
   * @param xSpeed Speed of the robot in the x direction (forward) in [0,1].
   * @param ySpeed Speed of the robot in the y direction (sideways) in [0,1].
   * @param rotSpeedIn Rotation of the robot (CCW) in [0,1]. Only applies if target is not observed.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void rotateToShoot(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    if (limelight.isValidTarget()) {
      // Get target angle relative to robot
      double targetRelativeAngleDegrees = limelight.getOffSetX(); // flipping so left is positive

      // Get desired rotation (in [0,1])
      double desiredRotation =
          Calibrations.Drivetrain.ROTATE_TO_TARGET_PID_CONTROLLER.calculate(
              targetRelativeAngleDegrees, 0.0) +
              Math.signum(targetRelativeAngleDegrees) * Calibrations.Drivetrain.ROTATE_TO_SHOOT_FF;

      drive(xSpeed, ySpeed, desiredRotation, fieldRelative);
    } else {
      drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
    }
  }

  public void keepHeading(double x, double y, double rot, boolean fieldRelative){
    double targetHeading = super.getLastHeading();
    double currentHeading = super.getHeading();
    double offsetHeading = MathUtil.inputModulus(currentHeading - targetHeading, -180, 180);

    double desiredRotation = Calibrations.Drivetrain.ROTATE_TO_TARGET_PID_CONTROLLER.calculate(offsetHeading, 0.0) + Math.signum(offsetHeading) * Calibrations.Drivetrain.ROTATE_TO_SHOOT_FF;
    double deadbandDesiredRotation = MathUtil.applyDeadband(desiredRotation, 3);

    drive(x, y, deadbandDesiredRotation, fieldRelative);
  }

  public void choose(double x, double y, double rot, boolean fieldRelative){
    if (rot != 0){
      drive(x, y, rot, fieldRelative);
    } else {
      keepHeading(x, y, rot, fieldRelative);
    }
  }



  public boolean alignedToTarget() {
    double targetRelativeAngleDegrees = -limelight.getOffSetX(); // flipping so left is positive
    return Math.abs(targetRelativeAngleDegrees)
        < Calibrations.SHOOTER_TARGET_ALIGNMENT_TOLERANCE_DEG;
  }

}
