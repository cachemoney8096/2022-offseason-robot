package net.cachemoney8096.frc2022o.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoDriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SwerveModuleFactory;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.cachemoney8096.frc2022o.libs.CoussensModuleConfiguration;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.FeatureFlags;
import net.cachemoney8096.frc2022o.libs.AbsoluteEncoderFromDioConfiguration;
import net.cachemoney8096.frc2022o.libs.AbsoluteEncoderFromDioFactoryBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drivetrain implements Subsystem {

  private class SwerveModuleStates {
    public SwerveModuleState frontLeftSwerveModuleState;
    public SwerveModuleState frontRightSwerveModuleState;
    public SwerveModuleState backLeftSwerveModuleState;
    public SwerveModuleState backRightSwerveModuleState;
  }

  // Constants for swerve
  private final double wheelDiametersInches = 3.0;
  private final double wheelDiameterMeters = Units.inchesToMeters(wheelDiametersInches);
  private final double driveReduction = 4.8;
  private final boolean driveInverted = false;
  private final double steerReduction = 11.25;
  private final boolean steerInverted = false;
  public final double MAX_VELOCITY_METERS_PER_SECOND =
      6380.0 / 60.0 * driveReduction * wheelDiameterMeters * Math.PI;
  public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 3 * Math.PI;

  // Swerve Modules and Kinematics
  private final ModuleConfiguration moduleConfiguration =
      new ModuleConfiguration(
          wheelDiameterMeters, driveReduction, driveInverted, steerReduction, steerInverted);
  private final CoussensModuleConfiguration coussensModuleConfiguration =
      new CoussensModuleConfiguration();
  private final SwerveModule frontLeftModule =
      new SwerveModuleFactory<>(
              moduleConfiguration,
              new NeoDriveControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                  .build(),
              new NeoSteerControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withPidConstants(1.0, 0.0, 0.1)
                  .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                  .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
          .create(
              // container, maybe want to add shuffleboard here
              RobotMap.DRIVE_FRONT_LEFT_ID,
              new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                  RobotMap.STEER_FRONT_LEFT_ID,
                  new AbsoluteEncoderFromDioConfiguration(
                      RobotMap.SWERVE_FRONT_LEFT_DIO, Constants.FRONT_LEFT_STEER_OFFSET_RAD)));
  private final SwerveModule frontRightModule =
      new SwerveModuleFactory<>(
              moduleConfiguration,
              new NeoDriveControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                  .build(),
              new NeoSteerControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withPidConstants(1.0, 0.0, 0.1)
                  .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                  .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
          .create(
              // container, maybe want to add shuffleboard here
              RobotMap.DRIVE_FRONT_RIGHT_ID,
              new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                  RobotMap.STEER_FRONT_RIGHT_ID,
                  new AbsoluteEncoderFromDioConfiguration(
                      RobotMap.SWERVE_FRONT_RIGHT_DIO, Constants.FRONT_RIGHT_STEER_OFFSET_RAD)));
  private final SwerveModule backLeftModule =
      new SwerveModuleFactory<>(
              moduleConfiguration,
              new NeoDriveControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                  .build(),
              new NeoSteerControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withPidConstants(1.0, 0.0, 0.1)
                  .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                  .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
          .create(
              // container, maybe want to add shuffleboard here
              RobotMap.DRIVE_BACK_LEFT_ID,
              new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                  RobotMap.STEER_BACK_LEFT_ID,
                  new AbsoluteEncoderFromDioConfiguration(
                      RobotMap.SWERVE_BACK_LEFT_DIO, Constants.BACK_LEFT_STEER_OFFSET_RAD)));
  private final SwerveModule backRightModule =
      new SwerveModuleFactory<>(
              moduleConfiguration,
              new NeoDriveControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                  .build(),
              new NeoSteerControllerFactoryBuilder()
                  .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                  .withPidConstants(1.0, 0.0, 0.1)
                  .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                  .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
          .create(
              // container, maybe want to add shuffleboard here
              RobotMap.DRIVE_BACK_RIGHT_ID,
              new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                  RobotMap.STEER_BACK_RIGHT_ID,
                  new AbsoluteEncoderFromDioConfiguration(
                      RobotMap.SWERVE_BACK_RIGHT_DIO, Constants.BACK_RIGHT_STEER_OFFSET_RAD)));
  private final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(Constants.TRACK_WIDTH_METERS / 2.0, Constants.WHEEL_BASE_METERS / 2.0),
          // Front right
          new Translation2d(Constants.TRACK_WIDTH_METERS / 2.0, -Constants.WHEEL_BASE_METERS / 2.0),
          // Back left
          new Translation2d(-Constants.TRACK_WIDTH_METERS / 2.0, Constants.WHEEL_BASE_METERS / 2.0),
          // Back right
          new Translation2d(
              -Constants.TRACK_WIDTH_METERS / 2.0, -Constants.WHEEL_BASE_METERS / 2.0));

  // Things for odometry
  private final PigeonIMU pigeonImu = new PigeonIMU(RobotMap.PIGEON_IMU_ID);
  private double gyroOffsetRad = 0;

  public Drivetrain() {
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
    // update odometry?
  }

  // TODO we need to set this based on starting position so we can do
  // field-relative
  public void initAutonPosition() {
    // public void initAutonPosition(PathPlannerTrajectory.PathPlannerState state){
    zeroGyroscope();
    // gyroOffset = state.holonomicRotation.getDegrees();
    // odometry.resetPosition(new Pose2d(state.poseMeters.getTranslation(),
    // state.holonomicRotation),getGyroscopeRotation());
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(
        pigeonImu.getFusedHeading() + Units.radiansToDegrees(gyroOffsetRad));
  }

  public void zeroGyroscope() {
    pigeonImu.setYaw(0.0);
    gyroOffsetRad = 0;
  }

  // Sets the commands to the swerve modules.
  // xCommand, yCommand, and rotationCommand are expected to be in [-1, 1].
  // If fieldRelative is false, then the command is considered to be
  // robot-relative (x is forward).
  public void drive(
      double xCommand, double yCommand, double rotationCommand, boolean fieldRelative) {
    // Check whether to keep current heading (translation-only)
    boolean heading_lock = headingLock(xCommand, yCommand, rotationCommand);

    // Convert to real velocities
    double xVelocityMetersPerSec = xCommand / MAX_VELOCITY_METERS_PER_SECOND;
    double yVelocityMetersPerSec = yCommand / MAX_VELOCITY_METERS_PER_SECOND;
    double rotVelocityRadPerSec = rotationCommand / MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    if (heading_lock) {
      rotVelocityRadPerSec = 0.0;
    }

    // Convert to commands to the swerve modules
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocityMetersPerSec,
                yVelocityMetersPerSec,
                rotVelocityRadPerSec,
                getGyroscopeRotation())
            : new ChassisSpeeds(xVelocityMetersPerSec, yVelocityMetersPerSec, rotVelocityRadPerSec);
    SwerveModuleState[] states_array = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    assert states_array.length == 4;
    SwerveDriveKinematics.desaturateWheelSpeeds(states_array, MAX_VELOCITY_METERS_PER_SECOND);

    // Convert to our desired format for commanding the modules, and command!
    SwerveModuleStates states = new SwerveModuleStates();
    states.frontLeftSwerveModuleState = states_array[0];
    states.frontRightSwerveModuleState = states_array[1];
    states.backLeftSwerveModuleState = states_array[2];
    states.backRightSwerveModuleState = states_array[3];
    setModuleStates(states);
  }

  // Sets the desired modules states to the modules
  private void setModuleStates(SwerveModuleStates states) {
    frontLeftModule.set(
        states.frontLeftSwerveModuleState.speedMetersPerSecond
            / MAX_VELOCITY_METERS_PER_SECOND
            * coussensModuleConfiguration.getNominalVoltage(),
        states.frontLeftSwerveModuleState.angle.getRadians());
    frontRightModule.set(
        states.frontRightSwerveModuleState.speedMetersPerSecond
            / MAX_VELOCITY_METERS_PER_SECOND
            * coussensModuleConfiguration.getNominalVoltage(),
        states.frontRightSwerveModuleState.angle.getRadians());
    backLeftModule.set(
        states.backLeftSwerveModuleState.speedMetersPerSecond
            / MAX_VELOCITY_METERS_PER_SECOND
            * coussensModuleConfiguration.getNominalVoltage(),
        states.backLeftSwerveModuleState.angle.getRadians());
    backRightModule.set(
        states.backRightSwerveModuleState.speedMetersPerSecond
            / MAX_VELOCITY_METERS_PER_SECOND
            * coussensModuleConfiguration.getNominalVoltage(),
        states.backRightSwerveModuleState.angle.getRadians());
  }

  // Uses input speeds and rotation commands to tell whether the current heading
  // should be locked. All inputs are expected to be in [-1,1]
  private boolean headingLock(double xCommand, double yCommand, double rotationCommand) {
    if (!FeatureFlags.ENABLE_HEADING_LOCK_TELEOP) {
      return false;
    }
    if (Math.abs(rotationCommand) > Calibrations.HEADING_LOCK_ROTATION_COMMAND_THRESHOLD) {
      return false;
    }
    if (Math.abs(xCommand) > Calibrations.HEADING_LOCK_TRANSLATION_COMMAND_THRESHOLD
        || Math.abs(yCommand) > Calibrations.HEADING_LOCK_TRANSLATION_COMMAND_THRESHOLD) {
      return false;
    }
    return true;
  }
}
