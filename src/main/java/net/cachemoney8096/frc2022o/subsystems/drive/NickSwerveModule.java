package net.cachemoney8096.frc2022o.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.libs_3005.controller.Controller;
import net.cachemoney8096.frc2022o.libs_3005.electromechanical.Encoder;
import net.cachemoney8096.frc2022o.libs_3005.swerve.SwerveModule;
import net.cachemoney8096.frc2022o.libs_3005.util.SendableHelper;
import net.cachemoney8096.frc2022o.libs_3005.vendor.motorcontroller.SparkMax;
import net.cachemoney8096.frc2022o.libs_3005.vendor.motorcontroller.SparkMaxUtils;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.ThroughBoreEncoder;
import org.tinylog.Logger;

public class NickSwerveModule implements SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final ProfiledPIDController m_turningController;
  private final Controller m_driveController;

  private final Encoder m_turningEncoder;
  private final ThroughBoreEncoder m_turningAbsoluteEncoder;
  private final Encoder m_driveEncoder;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_turningFeedforward;

  private boolean m_testEnable = false;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private static Boolean driveMotorConfig(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    RelativeEncoder enc = sparkMax.getEncoder();

    // Convert 'rotations' to 'meters'
    // enc.setPositionConversionFactor(Constants.Drivetrain.kDriveEncoderPositionFactor);
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(Constants.Drivetrain.DRIVE_ENCODER_POSITION_FACTOR));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(Constants.Drivetrain.DRIVE_ENCODER_VELOCITY_FACTOR));

    // Set inversion
    sparkMax.setInverted(Constants.Drivetrain.DRIVE_INVERTED);

    errors += SparkMaxUtils.check(sparkMax.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kCoast));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSmartCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT_AMPS));

    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);

    return errors == 0;
  }

  private static Boolean turningMotorConfig(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    RelativeEncoder enc = sparkMax.getEncoder();

    // Convert 'rotations' to 'meters'
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(Constants.Drivetrain.STEERING_ENCODER_POSITION_FACTOR));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(Constants.Drivetrain.STEERING_ENCODER_VELOCITY_FACTOR));

    // Set inversion
    sparkMax.setInverted(Constants.Drivetrain.STEER_INVERTED);

    errors += SparkMaxUtils.check(sparkMax.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    // errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
    // sparkMax.setSmartCurrentLimit(Constants.Drivetrain.kTurningMotorCurrentLimit);
    // sparkMax.getEncoder().setPosition(0.0);

    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);

    return errors == 0;
  }

  public NickSwerveModule(int driveCanId, int turningCanId, ThroughBoreEncoder turningEncoder) {
    m_driveMotor =
        new SparkMax(driveCanId, MotorType.kBrushless)
            .withInitializer(NickSwerveModule::driveMotorConfig);
    m_turningMotor =
        new SparkMax(turningCanId, MotorType.kBrushless)
            .withInitializer(NickSwerveModule::turningMotorConfig);

    m_turningEncoder = m_turningMotor.builtinEncoder();
    m_driveEncoder = m_driveMotor.builtinEncoder();
    m_driveEncoder.setPosition(0.0);
    m_turningAbsoluteEncoder = turningEncoder;

    m_driveFeedforward = Calibrations.Drivetrain.DRIVE_FEEDFORWARD;
    m_turningFeedforward = Calibrations.Drivetrain.STEER_FEEDFORWARD;

    m_driveController = m_driveMotor.velocityController(Calibrations.Drivetrain.DRIVE_PID_GAINS);

    m_turningController =
        new ProfiledPIDController(
          Calibrations.Drivetrain.STEER_PID_GAINS.P,
          Calibrations.Drivetrain.STEER_PID_GAINS.I,
          Calibrations.Drivetrain.STEER_PID_GAINS.D,
          Calibrations.Drivetrain.STEER_TRAPEZOID_CONSTRAINTS);

    m_turningController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the position to the absolute values
    resetEncoders();

    // Reset the angle to current angle
    m_desiredState.angle = new Rotation2d(m_turningAbsoluteEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.setActuator(true);
    SendableHelper.addChild(builder, this, m_driveController, "DriveMotor");
    SendableHelper.addChild(builder, this, m_turningController, "TurningMotor");
    SendableHelper.addChild(builder, this, m_turningEncoder, "TurningEncoder");
    SendableHelper.addChild(builder, this, m_driveEncoder, "DriveEncoder");
    if (m_turningAbsoluteEncoder != null) {
      SendableHelper.addChild(builder, this, m_turningAbsoluteEncoder, "TurningAbsolute");
    }
    SendableHelper.addChild(builder, this, m_turningEncoder, "TurningEncoder");
    builder.addDoubleProperty(
        "Velocity Setpoint",
        () -> m_desiredState.speedMetersPerSecond,
        val -> setDesiredState(new SwerveModuleState(val, m_desiredState.angle)));
    builder.addDoubleProperty(
        "Turning Setpoint",
        () -> m_desiredState.angle.getRadians(),
        (val) ->
            setDesiredState(
                new SwerveModuleState(m_desiredState.speedMetersPerSecond, new Rotation2d(val))));
    builder.addBooleanProperty("Testmode Enable", () -> m_testEnable, (val) -> m_testEnable = val);
    builder.addDoubleProperty("Drive Current", () -> m_driveMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Turning Current", () -> m_turningMotor.getOutputCurrent(), null);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningAbsoluteEncoder.getPosition()));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Module range must be [0, PI)
    // desiredState.angle = new Rotation2d(MathUtil.inputModulus(desiredState.angle.getRadians(),
    // 0.0, 2.0 * Math.PI));
    desiredState.angle = new Rotation2d(desiredState.angle.getRadians() % (2.0 * Math.PI));
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(
            desiredState, new Rotation2d(m_turningAbsoluteEncoder.getPosition()));

    // Module range must be [0, PI)
    desiredState.angle = new Rotation2d(desiredState.angle.getRadians() % (2.0 * Math.PI));

    m_desiredState = state;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Override
  public void periodic() {
    // Calculate the turning motor output from the turning PID controller.
    m_driveController.setReference(
        m_desiredState.speedMetersPerSecond,
        m_driveEncoder.getVelocity(),
        m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond));
    m_turningController.setGoal(m_desiredState.angle.getRadians());
    double demand = m_turningController.calculate(m_turningAbsoluteEncoder.getPosition());
    demand += m_turningFeedforward.calculate(m_turningController.getSetpoint().velocity);
    m_turningMotor.setVoltage(demand);
  }

  /** Run the controllers */
  @Override
  public void testPeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (!m_testEnable) {
      m_turningMotor.setVoltage(0.0);
      return;
    }

    m_driveController.setReference(
        m_desiredState.speedMetersPerSecond,
        m_driveEncoder.getVelocity(),
        m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond));
    m_turningController.setGoal(m_desiredState.angle.getRadians());

    double demand = m_turningController.calculate(m_turningAbsoluteEncoder.getPosition());
    demand += m_turningFeedforward.calculate(m_turningController.getSetpoint().velocity);
    m_turningMotor.setVoltage(demand);
  }

  @Override
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    if (m_turningAbsoluteEncoder != null) {
      m_turningEncoder.setPosition(m_turningAbsoluteEncoder.getPosition());
    } else {
      Logger.tag("Swerve Module").warn("Unable to reset, no sensor to reset on");
    }
  }

  @Override
  public double getDriveDistanceMeters() {
    return m_driveEncoder.getPosition();
  }
}
