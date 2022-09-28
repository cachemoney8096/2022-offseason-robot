package net.cachemoney8096.frc2022o.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import net.cachemoney8096.frc2022o.RobotMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  // Actuators
  private final CANSparkMax shooterMotorOne;
  private final SparkMaxPIDController shooterPID;
  private final CANSparkMax shooterMotorTwo;
  private final CANSparkMax hoodMotor;

  // Sensors
  private final RelativeEncoder shooterEncoder;
  private final DutyCycleEncoder hoodAbsoluteEncoder;

  public Shooter() {
    shooterMotorOne = new CANSparkMax(RobotMap.SHOOTER_MOTOR_ONE_ID, MotorType.kBrushless);
    shooterMotorOne.restoreFactoryDefaults();
    shooterEncoder = shooterMotorOne.getEncoder();
    // shooterEncoder.setVelocityConversionFactor(???);
    shooterPID = shooterMotorOne.getPIDController();
    // shooterPID.setP();
    // shooterPID.setI();
    // shooterPID.setD();
    // shooterPID.setFF();

    shooterMotorTwo = new CANSparkMax(RobotMap.SHOOTER_MOTOR_TWO_ID, MotorType.kBrushless);
    shooterMotorTwo.restoreFactoryDefaults();
    final boolean INVERT_FOLLOW = true;
    shooterMotorTwo.follow(shooterMotorOne, INVERT_FOLLOW);

    hoodMotor = new CANSparkMax(RobotMap.HOOD_MOTOR_ID, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    final int HOOD_MOTOR_CURRENT_LIMIT = 40;
    hoodMotor.setSmartCurrentLimit(HOOD_MOTOR_CURRENT_LIMIT);
    hoodAbsoluteEncoder = new DutyCycleEncoder(RobotMap.HOOD_ENCODER_DIO);
    // hoodAbsoluteEncoder.setDistancePerRotation(???);
    // TODO use wpilib PID to do position control on the roboRIO
  }

  public double getHoodPosition() {
    return hoodAbsoluteEncoder.getDistance();
  }

  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setHoodPosition(double position_deg) {
    // hoodPID.setReference(position_deg, ControlType.kPosition);
  }

  public void setShooterVelocity() {}
}
