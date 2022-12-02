package net.cachemoney8096.frc2022o.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import net.cachemoney8096.frc2022o.RobotMap;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

public class Climber extends SubsystemBase {
  private final CANSparkMax climbMotorRight;
  private final CANSparkMax climbMotorLeft;
  private final RelativeEncoder climbMotorLeftEncoder;
  private final RelativeEncoder climbMotorRightEncoder;
  private final boolean INVERSION_PLACEHOLDER = false;

  public Climber() {
    climbMotorRight = new CANSparkMax(RobotMap.CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);
    climbMotorLeft = new CANSparkMax(RobotMap.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);
    climbMotorLeftEncoder = climbMotorLeft.getEncoder();
    climbMotorRightEncoder = climbMotorRight.getEncoder();
    climbMotorLeftEncoder.setPosition(0);
    climbMotorRightEncoder.setPosition(0);

    climbMotorRight.restoreFactoryDefaults();
    climbMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climbMotorRight.setInverted(
        !INVERSION_PLACEHOLDER); // TODO see which way motors are facing and invert such that
    // positive = in

    climbMotorLeft.restoreFactoryDefaults();
    climbMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climbMotorLeft.setInverted(
        INVERSION_PLACEHOLDER); // TODO see which way motors are facing and invert such that
    // positive = in
    
    climbMotorLeft.setSoftLimit(SoftLimitDirection.kForward, -110.0f);
    climbMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, -110.0f);
    climbMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climbMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    climbMotorRight.setSoftLimit(SoftLimitDirection.kForward, -110.0f);
    climbMotorRight.setSoftLimit(SoftLimitDirection.kReverse, -110.0f);
    climbMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climbMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
    // climbMotorLeft.setSmartCurrentLimit(80);
    // climbMotorRight.setSmartCurrentLimit(80);
  }

  public void rightMotorDown() {
    climbMotorRight.set(1.0);
  }

  public void rightMotorUp() {
    climbMotorRight.set(-0.5);
  }

  public void leftMotorDown() {
    climbMotorLeft.set(1.0);
  }

  public void leftMotorUp() {
    climbMotorLeft.set(-0.5);
  }

  public void holdClimb(){
    climbMotorLeft.set(0);
    climbMotorRight.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Left Climber Power",
        () -> {
          return climbMotorLeft.get();
        },
        null);
    builder.addDoubleProperty(
        "Right Climber Power",
        () -> {
          return climbMotorRight.get();
        },
        null);
    builder.addDoubleProperty("Left Climber Position", () -> {
      return climbMotorLeftEncoder.getPosition();
    }, null);
  }
}
