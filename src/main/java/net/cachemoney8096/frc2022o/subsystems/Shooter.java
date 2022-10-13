package net.cachemoney8096.frc2022o.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import net.cachemoney8096.frc2022o.RobotMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.Limelight;


public class Shooter extends SubsystemBase {

  // Actuators
  private final CANSparkMax shooterMotorOne;
  private final SparkMaxPIDController shooterPID;
  private final SparkMaxPIDController hoodPID;
  private final CANSparkMax shooterMotorTwo;
  private final CANSparkMax hoodMotor;

  // Sensors
  private final RelativeEncoder shooterEncoder;
  private final DutyCycleEncoder hoodAbsoluteEncoder;

  private double shooterSetpointRPM = 0;
  private double hoodSetpointDeg = 0;

  private final Limelight limelight;

  public Shooter(Limelight limelight) {
    shooterMotorOne = new CANSparkMax(RobotMap.SHOOTER_MOTOR_ONE_ID, MotorType.kBrushless);
    shooterMotorOne.restoreFactoryDefaults();
    shooterEncoder = shooterMotorOne.getEncoder();
    shooterEncoder.setVelocityConversionFactor(Constants.SHOOTER_ENCODER_RATIO);

    shooterPID = shooterMotorOne.getPIDController();
    shooterPID.setP(Calibrations.SHOOTER_kP);
    shooterPID.setI(Calibrations.SHOOTER_kI);
    shooterPID.setD(Calibrations.SHOOTER_kD);
    shooterPID.setFF(Calibrations.SHOOTER_kF);

    shooterMotorTwo = new CANSparkMax(RobotMap.SHOOTER_MOTOR_TWO_ID, MotorType.kBrushless);
    shooterMotorTwo.restoreFactoryDefaults();
    final boolean INVERT_FOLLOW = true;
    shooterMotorTwo.follow(shooterMotorOne, INVERT_FOLLOW);

    hoodMotor = new CANSparkMax(RobotMap.HOOD_MOTOR_ID, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    final int HOOD_MOTOR_CURRENT_LIMIT = 40;
    hoodMotor.setSmartCurrentLimit(HOOD_MOTOR_CURRENT_LIMIT);
    hoodAbsoluteEncoder = new DutyCycleEncoder(RobotMap.HOOD_ENCODER_DIO);
    hoodAbsoluteEncoder.setDistancePerRotation(360 / Constants.HOOD_ENCODER_RATIO);

    hoodPID = hoodMotor.getPIDController();
    hoodPID.setP(Calibrations.HOOD_kP);
    hoodPID.setI(Calibrations.HOOD_kI);
    hoodPID.setD(Calibrations.HOOD_kD);
    hoodPID.setFF(Calibrations.HOOD_kF);

    this.limelight = limelight;
  }

  public double getHoodPosition() {
    return hoodAbsoluteEncoder.getDistance();
  }

  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setHoodPosition(double positionDeg) {
    hoodPID.setReference(positionDeg, ControlType.kPosition);
    hoodSetpointDeg = positionDeg;
  }

  public void setShooterVelocity(double velocityRpm) {
    shooterPID.setReference(velocityRpm, ControlType.kVelocity);
    shooterSetpointRPM = velocityRpm;
  }

  public boolean checkShootReady() {

    if (Math.abs(getHoodPosition() - hoodSetpointDeg) < Calibrations.HOOD_RANGE_DEG
        && Math.abs(getShooterVelocity() - shooterSetpointRPM) < Calibrations.SHOOTER_RANGE_RPM) {
      return true; // ready
    } else {
      return false; // not ready
    }
  }

  public void shoot() {
    // dummy function because the actual code only exists in another branch 😐
  }

  public void dontShoot() {
    setShooterVelocity(0);
  }

  public void aimHood() {
    if (limelight.isValidTarget()){
      setHoodPosition(Calibrations.HOOD_TABLE.get(limelight.getDistanceFromTargetMeters()));
    }
  }

  public void shoot(){
    if (checkShootReady() && limelight.isValidTarget()){
      shooterMotorOne.getPIDController().setReference(Calibrations.SHOOTER_TABLE.get(limelight.getDistanceFromTargetMeters()), ControlType.kVelocity);
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
          return shooterSetpointRPM;
        },
        this::setShooterVelocity);
  }
}
