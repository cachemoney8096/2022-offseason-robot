package net.cachemoney8096.frc2022o.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.ThroughBoreEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.Limelight;

public class Shooter extends SubsystemBase {

  // Actuators
  private final CANSparkMax shooterMotorLeft;
  private final SparkMaxPIDController shooterController;
  private final CANSparkMax shooterMotorRight;
  private final CANSparkMax hoodMotor;

  // Sensors
  private final RelativeEncoder shooterEncoder;
  private final RelativeEncoder hoodMotorEncoder;
  private final ThroughBoreEncoder hoodAbsoluteEncoder;
  private final Limelight limelight;

  // Members
  private final boolean INVERT_HOOD_ENCODER = true;
  private final SparkMaxPIDController hoodController;
  private double shooterSetpointRpm = 0;
  private double hoodSetpointDeg = 0;

  /** Reading from the hoodAbsoluteEncoder when the hood is at its lowest (retracted) point */
  private final double HOOD_ABSOLUTE_ENCODER_OFFSET_DEG = 10;

  public Shooter(Limelight limelightIn) {
    limelight = limelightIn;

    shooterMotorLeft = new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_ID, MotorType.kBrushless);
    shooterMotorLeft.restoreFactoryDefaults();
    shooterEncoder = shooterMotorLeft.getEncoder();
    shooterEncoder.setVelocityConversionFactor(Constants.SHOOTER_ENCODER_RATIO);
    shooterMotorLeft.setInverted(true);

    shooterController = shooterMotorLeft.getPIDController();
    shooterController.setP(Calibrations.SHOOTER_kP);
    shooterController.setI(Calibrations.SHOOTER_kI);
    shooterController.setD(Calibrations.SHOOTER_kD);
    shooterController.setFF(Calibrations.SHOOTER_kF);

    shooterMotorRight = new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT_ID, MotorType.kBrushless);
    shooterMotorRight.restoreFactoryDefaults();
    final boolean INVERT_FOLLOW = true;
    shooterMotorRight.follow(shooterMotorLeft, INVERT_FOLLOW);

    hoodMotor = new CANSparkMax(RobotMap.HOOD_MOTOR_ID, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    final int HOOD_MOTOR_CURRENT_LIMIT = 15; //20
    hoodMotor.setSmartCurrentLimit(HOOD_MOTOR_CURRENT_LIMIT);
    hoodMotor.setInverted(true);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 11.0f);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 36.0f);

    hoodAbsoluteEncoder =
        new ThroughBoreEncoder(
            RobotMap.HOOD_ENCODER_DIO,
            HOOD_ABSOLUTE_ENCODER_OFFSET_DEG,
            Constants.HOOD_EXTERNAL_ENCODER_SCALAR,
            INVERT_HOOD_ENCODER);
    hoodMotorEncoder = hoodMotor.getEncoder();
    hoodMotorEncoder.setPositionConversionFactor(Constants.HOOD_MOTOR_ENCODER_SCALAR);
    hoodMotorEncoder.setVelocityConversionFactor(Constants.HOOD_MOTOR_ENCODER_VELOCITY_SCALAR);

    hoodController = hoodMotor.getPIDController();
    hoodController.setP(Calibrations.HOOD_kP);
    hoodController.setI(Calibrations.HOOD_kI);
    hoodController.setD(Calibrations.HOOD_kD);
    hoodController.setFF(Calibrations.HOOD_kF);
    hoodController.setIZone(Calibrations.HOOD_IZone);
  }

  /** Call for initialization at least a couple seconds after construction */
  public void initialize() {
    // This allows for the hood to not be fully retracted when we turn on the robot
    // The absolute encoder minus offset gets us the real current position
    // From then on we can use the hoodMotorEncoder position, which is relative to start position
    hoodMotorEncoder.setPosition(
        hoodAbsoluteEncoder.getPosition());
  }

  /** Get hood position in actual degrees of hood movement from start */
  public double getHoodPositionDeg() {
    return hoodMotorEncoder.getPosition();
  }

  public double getHoodAbsolutePosition(){
    return hoodAbsoluteEncoder.getPosition();
  }

  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setHoodPosition(double positionDeg) {
    hoodSetpointDeg = positionDeg;
    hoodController.setReference(hoodSetpointDeg, ControlType.kPosition);
  }

  /**
   * Sets the velocity set point for the shooter wheel. There's special handling for 0 RPM, to
   * simply leave the motor unpowered instead of having the controller force the shooter wheel to
   * zero.
   *
   * @param velocityRpm Desired velocity for shooter wheel in rev / minute.
   */
  public void setShooterVelocity(double velocityRpm) {
    shooterSetpointRpm = velocityRpm;
    if (velocityRpm == 0.0) {
      // Unpowered slow down
      shooterController.setReference(0.0, ControlType.kVoltage);
    } else {
      shooterController.setReference(velocityRpm, ControlType.kVelocity);
    }
  }

  private boolean hoodPositionReady() {
    return Math.abs(getHoodPositionDeg() - hoodSetpointDeg)
        < Calibrations.HOOD_POSITION_TOLERANCE_DEG;
  }

  private boolean shooterSpeedReady() {
    return Math.abs(getShooterVelocity() - shooterSetpointRpm) < Calibrations.SHOOTER_RANGE_RPM;
  }

  public boolean checkShootReady() {
    return hoodPositionReady() && shooterSpeedReady();
  }

  public void dontShoot() {
    setShooterVelocity(0.0);
    hoodController.setReference(0.0, ControlType.kVoltage);
  }

  public void shoot() {
    if (limelight.isValidTarget()) {
      double distanceFromTargetMeters = limelight.getDistanceFromTargetMeters();

      // Send shooter setpoint to shooter controller (which runs an internal PID)
      setShooterVelocity(Calibrations.SHOOTER_TABLE.get(distanceFromTargetMeters));

      // Send hood setpoint to hood controller (which also runs an internal PID)
      setHoodPosition(Calibrations.HOOD_TABLE.get(distanceFromTargetMeters));
    } else {
      setShooterVelocity(2000);
      setHoodPosition(25);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Hood Setpoint Position (deg)",
        () -> {
          return hoodSetpointDeg;
        },
        this::setHoodPosition);
    builder.addDoubleProperty("Absolute Hood Position", this::getHoodAbsolutePosition, null);
    builder.addDoubleProperty("Motor Hood Position", this::getHoodPositionDeg, null);
    builder.addDoubleProperty(
        "Shooter Setpoint Speed (RPM)",
        () -> {
          return shooterSetpointRpm;
        },
        this::setShooterVelocity);
    builder.addDoubleProperty("Shooter Velocity", this::getShooterVelocity, null);
    builder.addBooleanProperty("Hood Position Ready", this::hoodPositionReady, null);
    builder.addBooleanProperty("Shooter Speed Ready", this::shooterSpeedReady, null);
    builder.addDoubleProperty("Hood kP", hoodController::getP, hoodController::setP);
    builder.addDoubleProperty("Hood kI", hoodController::getI, hoodController::setI);
    builder.addDoubleProperty("Hood kD", hoodController::getD, hoodController::setD);
    builder.addDoubleProperty("Hood kFF", hoodController::getFF, hoodController::setFF);
    builder.addDoubleProperty("Shooter kP", shooterController::getP, shooterController::setP);
    builder.addDoubleProperty("Shooter kI", shooterController::getI, shooterController::setI);
    builder.addDoubleProperty("Shooter kD", shooterController::getD, shooterController::setD);
    builder.addDoubleProperty("Shooter kFF", shooterController::getFF, shooterController::setFF);
    addChild("Hood Absolute Encoder", hoodAbsoluteEncoder);
  }
}
