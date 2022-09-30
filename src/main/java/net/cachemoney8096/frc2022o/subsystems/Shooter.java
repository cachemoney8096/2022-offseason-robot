package net.cachemoney8096.frc2022o.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import net.cachemoney8096.frc2022o.RobotMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.Constants;

import java.lang.Math;

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

  private double ShooterSetpointRPM = 0;
  private double HoodSetpointDeg = 0;

  public Shooter() {
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
    hoodAbsoluteEncoder.setDistancePerRotation(360 * Constants.HOOD_ENCODER_RATIO);

    hoodPID = hoodMotor.getPIDController();
    hoodPID.setP(Calibrations.HOOD_kP);
    hoodPID.setI(Calibrations.HOOD_kI);
    hoodPID.setD(Calibrations.HOOD_kD);
    hoodPID.setFF(Calibrations.HOOD_kF);
  }

  public double getHoodPosition() {
    return hoodAbsoluteEncoder.getDistance();
  }

  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setHoodPosition(double position_deg) {
    hoodPID.setReference(position_deg, ControlType.kPosition);
    HoodSetpointDeg = position_deg;
  }

  public void setShooterVelocity(double velocity_rpm) {
    shooterPID.setReference(velocity_rpm, ControlType.kVelocity);
    ShooterSetpointRPM = velocity_rpm;
  }

  public boolean checkShootReady(){

    if (Math.abs(getHoodPosition() - HoodSetpointDeg) > Calibrations.HOOD_RANGE && Math.abs(getShooterVelocity() - ShooterSetpointRPM) > Calibrations.SHOOTER_RANGE){
      return true;
    } else {
      return false;
    }
  }
}
