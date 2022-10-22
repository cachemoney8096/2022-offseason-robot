// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o.libs_3005.swerve;

import org.tinylog.Logger;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.libs.SendablePigeon;

public abstract class SwerveDrive extends SubsystemBase {
  public enum ModuleLocation {
    frontLeft(0),
    frontRight(1),
    rearLeft(2),
    rearRight(3);

    public final int value;
    private static final ModuleLocation[] m_mapping =
        new ModuleLocation[] {frontLeft, frontRight, rearLeft, rearRight};

    private ModuleLocation(int v) {
      this.value = v;
    }

    public static ModuleLocation fromInt(int v) {
      return m_mapping[v];
    }
  }
  // Robot swerve modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_rearRight;

  /** The gyro sensor */
  private final SendablePigeon pigeon;

  private final SwerveDriveKinematics m_kinematics;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();

  private final double m_maxSpeed;

  // Position based odometry updates, thanks @TripleHelix
  private final Timer m_loopTimer;
  private double[] m_lastDistances;
  private double m_lastLoopTime;

  /**
   * Create a Swerve Drive module
   *
   * @param frontLeft Swerve Module
   * @param frontRight Swerve Module
   * @param rearLeft Swerve Module
   * @param rearRight Swerve Module
   * @param kinematics Swerve drive kinematics
   * @param pigeonIn used for odometry and field centric driving
   * @param maxSpeed of the wheels used to normalize wheel speeds
   */
  public SwerveDrive(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight,
      SwerveDriveKinematics kinematics,
      SendablePigeon pigeonIn,
      double maxSpeed) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_rearLeft = rearLeft;
    m_rearRight = rearRight;
    pigeon = pigeonIn;
    m_kinematics = kinematics;
    m_maxSpeed = maxSpeed;
    m_odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d());

    m_loopTimer = new Timer();
    m_loopTimer.reset();
    m_loopTimer.start();
    m_lastLoopTime = 0;
    m_lastDistances =
        new double[] {
          m_frontLeft.getDriveDistanceMeters(),
          m_frontRight.getDriveDistanceMeters(),
          m_rearLeft.getDriveDistanceMeters(),
          m_rearRight.getDriveDistanceMeters(),
        };
  }

  private ChassisSpeeds m_chassisSpeed = new ChassisSpeeds();

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity vx", () -> m_chassisSpeed.vxMetersPerSecond, null);
    builder.addDoubleProperty("Velocity vy", () -> m_chassisSpeed.vyMetersPerSecond, null);
    builder.addDoubleProperty("Velocity omega", () -> m_chassisSpeed.omegaRadiansPerSecond, null);
    addChild("Pigeon", pigeon);
    addChild("frontLeft", m_frontLeft);
    addChild("frontRight", m_frontRight);
    addChild("rearLeft", m_rearLeft);
    addChild("rearRight", m_rearRight);
    addChild("Field 2d", m_field);
  }

  @Override
  public void periodic() {
    // Run swerve modules if they require it
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    m_chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());

    double[] distances =
        new double[] {
          m_frontLeft.getDriveDistanceMeters(),
          m_frontRight.getDriveDistanceMeters(),
          m_rearLeft.getDriveDistanceMeters(),
          m_rearRight.getDriveDistanceMeters(),
        };

    double time = m_loopTimer.get();
    double dt = time - m_lastLoopTime;
    m_lastLoopTime = time;

    if (dt == 0) {
      return;
    }

    // Update the odometry in the periodic block
    m_odometry.updateWithTime(
        time,
        pigeon.getRotation2d(),
        new SwerveModuleState(
            (distances[0] - m_lastDistances[0]) / dt, m_frontLeft.getState().angle),
        new SwerveModuleState(
            (distances[1] - m_lastDistances[1]) / dt, m_frontRight.getState().angle),
        new SwerveModuleState(
            (distances[2] - m_lastDistances[2]) / dt, m_rearLeft.getState().angle),
        new SwerveModuleState(
            (distances[3] - m_lastDistances[3]) / dt, m_rearRight.getState().angle));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_lastDistances = distances;
  }

  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_rearRight.simulationPeriodic();
    m_rearLeft.simulationPeriodic();
  }

  /**
   * Return the current velocity of the chassis as a ChassisSpeeds object.
   *
   * @return velocity of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeed;
  }

  /** Checks whether the chassis is moving appreciably or not */
  public boolean isMoving() {
    final double MOVING_THRESHOLD_METERS_PER_SECOND = 0.1;
    final double ROTATING_THRESHOLD_RADIANS_PER_SECOND = 0.1;
    if (m_chassisSpeed.vxMetersPerSecond > MOVING_THRESHOLD_METERS_PER_SECOND
        || m_chassisSpeed.vyMetersPerSecond > MOVING_THRESHOLD_METERS_PER_SECOND
        || m_chassisSpeed.omegaRadiansPerSecond > ROTATING_THRESHOLD_RADIANS_PER_SECOND) {
      return true;
    }
    return false;
  }

  /**
   * Predict the motion between the current position and a future state.
   *
   * @param lookahead Time in seconds to predict ahead.
   * @return twist2d represnting the change in pose over the lookahead time.
   */
  public Twist2d getPredictedMotion(double lookahead) {
    ChassisSpeeds chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());
    return new Twist2d(
        chassisSpeed.vxMetersPerSecond * lookahead,
        chassisSpeed.vyMetersPerSecond * lookahead,
        chassisSpeed.omegaRadiansPerSecond * lookahead);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, pigeon.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward) in [0,1].
   * @param ySpeed Speed of the robot in the y direction (sideways) in [0,1].
   * @param rot Angular rate of the robot in [0,1], where positive is CCW.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // If nothing is commanded, hold the same position
    if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
      holdAllModulesRotation();
      return;
    }

    ySpeed = ySpeed * m_maxSpeed;
    xSpeed = xSpeed * m_maxSpeed;
    rot = rot * m_maxSpeed;

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private void holdModuleRotation(SwerveModule m) {
    var state = m.getDesiredState();
    state.speedMetersPerSecond = 0;
    m.setDesiredState(state);
  }

  private void holdAllModulesRotation() {
    holdModuleRotation(m_frontLeft);
    holdModuleRotation(m_frontRight);
    holdModuleRotation(m_rearLeft);
    holdModuleRotation(m_rearRight);
  }

  /** Set the swerve drive into an X which is not drivable but should help prevent being pushed. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Set an individual module state independently of all other modules. This should only be used for
   * testing/tuning.
   */
  public void testPeriodic() {
    m_frontLeft.testPeriodic();
    m_frontRight.testPeriodic();
    m_rearLeft.testPeriodic();
    m_rearRight.testPeriodic();
  }

  public SwerveModuleState[] getModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon.setYaw(0.0);
  }

  /** Calibrate the gyro. Requirements for the device being still depend on the gyro being used. */
  public void calibrateGyro() {
    // Pigeon does not require calibration, auto-calibrates on boot
    // pigeon.calibrate();
  }

  public void setHeading(double degreesCCWPositive) {
    Logger.tag("Swerve Drive").trace("Setting heading to {} degrees", degreesCCWPositive);
    pigeon.setYaw(degreesCCWPositive);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return pigeon.getRotation2d().getDegrees();
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  public Command stopCommand() {
    return new RunCommand(this::stop, this).withName("Swerve Stop");
  }

  /**
   * Create a trajectory following command. Note that the beginning and end states of the command
   * are not necessarily 0 speed.
   *
   * @param trajectory PathPlanner trajectory
   * @param xController PID Controller for the X direction (left/right)
   * @param yController PID Contorller for the Y direction (forward/back)
   * @param thetaController Turning PID Controller for rotation (CCW Positive)
   * @return Command to be scheduled
   */
  public Command trajectoryFollowerCommand(
      PathPlannerTrajectory trajectory,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController) {
    Command swCommand =
        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            m_kinematics,
            xController,
            yController,
            thetaController,
            (states) -> setModuleStates(states),
            this);
    return new InstantCommand(() -> m_field.getObject("Trajectory").setTrajectory(trajectory))
        .alongWith(swCommand);
  }
}
