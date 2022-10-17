package net.cachemoney8096.frc2022o.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.ThroughBoreEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.Limelight;
import edu.wpi.first.math.MathUtil;

public class Shooter extends SubsystemBase {

  // Actuators
  private final CANSparkMax shooterMotorLeft;
  private final SparkMaxPIDController shooterController;
  private final CANSparkMax shooterMotorRight;
  private final CANSparkMax hoodMotor;

  // Sensors
  private final RelativeEncoder shooterEncoder;
  private final ThroughBoreEncoder hoodAbsoluteEncoder;
  private final Limelight limelight;

  // Members
  private final boolean INVERT_HOOD_ENCODER = false; // TODO placeholder
  private final PIDController hoodController;
  private double shooterSetpointRpm = 0;
  private double hoodSetpointDeg = 0;

  public Shooter(Limelight limelightIn) {
    limelight = limelightIn;

    shooterMotorLeft = new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_ID, MotorType.kBrushless);
    shooterMotorLeft.restoreFactoryDefaults();
    shooterEncoder = shooterMotorLeft.getEncoder();
    shooterEncoder.setVelocityConversionFactor(Constants.SHOOTER_ENCODER_RATIO);
    shooterMotorLeft.setInverted(
        false);

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
    final int HOOD_MOTOR_CURRENT_LIMIT = 20;
    hoodMotor.setSmartCurrentLimit(HOOD_MOTOR_CURRENT_LIMIT);
    hoodMotor.setInverted(
        true);

    hoodAbsoluteEncoder =
        new ThroughBoreEncoder(
            RobotMap.HOOD_ENCODER_DIO, 0.0, Constants.HOOD_ENCODER_SCALAR, INVERT_HOOD_ENCODER);
    hoodAbsoluteEncoder.setPositionOffsetToCurrentPosition();

    hoodController =
        new PIDController(Calibrations.HOOD_kP, Calibrations.HOOD_kI, Calibrations.HOOD_kD);
    hoodController.setIntegratorRange(
        -Calibrations.HOOD_MAX_INTEGRAL_VALUE, Calibrations.HOOD_MAX_INTEGRAL_VALUE);
    hoodController.setTolerance(
        Calibrations.HOOD_POSITION_TOLERANCE_DEG, Calibrations.HOOD_VELOCITY_TOLERANCE_DEG_PER_SEC);
  }

  @Override
  public void periodic() {
    // TODO consider moving this to a function that gets called after
    // there's an opportunity to change hoodSetpointDeg
    // Recall: periodic runs first, so this will lag by a whole cycle
    hoodMotor.set(calculateHoodControl());
  }

  private double calculateHoodControl() {
    return MathUtil.clamp(
        hoodController.calculate(getHoodPositionDeg(), hoodSetpointDeg) + Calibrations.HOOD_kF,
        -1.0,
        1.0);
  }

  /** Get hood position in actual degrees of hood movement from start */
  public double getHoodPositionDeg() {
    return hoodAbsoluteEncoder.getPosition();
  }

  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setHoodPosition(double positionDeg) {
    hoodSetpointDeg = positionDeg;
  }

  public void setShooterVelocity(double velocityRpm) {
    shooterController.setReference(velocityRpm, ControlType.kVelocity);
    shooterSetpointRpm = velocityRpm;
  }

  public boolean checkShootReady() {
    if (hoodController.atSetpoint()
        && Math.abs(getShooterVelocity() - shooterSetpointRpm) < Calibrations.SHOOTER_RANGE_RPM) {
      return true; // ready
    } else {
      return false; // not ready
    }
  }

  public void dontShoot() {
    setShooterVelocity(0);
  }

  public void aimHood() {
    if (limelight.isValidTarget()) {
      setHoodPosition(Calibrations.HOOD_TABLE.get(limelight.getDistanceFromTargetMeters()));
    }
  }

  public void shoot() {
    if (checkShootReady() && limelight.isValidTarget()) {
      setShooterVelocity(Calibrations.SHOOTER_TABLE.get(limelight.getDistanceFromTargetMeters()));
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Hood Position (deg)",
        () -> {
          return hoodSetpointDeg;
        },
        this::setHoodPosition);
    builder.addDoubleProperty(
        "Shooter Speed (RPM)",
        () -> {
          return shooterSetpointRpm;
        },
        this::setShooterVelocity);
    addChild("Hood PID", hoodController);
    builder.addDoubleProperty("Shooter kP", shooterController::getP, shooterController::setP);
    builder.addDoubleProperty("Shooter kI", shooterController::getI, shooterController::setI);
    builder.addDoubleProperty("Shooter kD", shooterController::getD, shooterController::setD);
    builder.addDoubleProperty("Shooter kFF", shooterController::getFF, shooterController::setFF);
    addChild("Hood Encoder", hoodAbsoluteEncoder);
  }
}
