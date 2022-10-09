package net.cachemoney8096.frc2022o.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
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

  public DriveSubsystem(SendablePigeon gyro, Limelight limelight) {
    super(
        frontLeft,
        frontRight,
        rearLeft,
        rearRight,
        Constants.Drivetrain.SWERVE_KINEMATICS,
        gyro,
        Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
    this.limelight = limelight;
    // Logger.tag("Swerve Drive").warn("Reset calibration time back to longer for comp");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("X Controller", Calibrations.Drivetrain.PATH_X_CONTROLLER);
    addChild("Y Controller", Calibrations.Drivetrain.PATH_Y_CONTROLLER);
    addChild("Theta Controller", Calibrations.Drivetrain.PATH_THETA_CONTROLLER);
  }

  public void rotateToShoot(){
      
  }
}
