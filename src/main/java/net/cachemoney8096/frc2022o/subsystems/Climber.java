package net.cachemoney8096.frc2022o.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import net.cachemoney8096.frc2022o.RobotMap;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

public class Climber extends SubsystemBase {
  private final CANSparkMax climbMotor;
  private final RelativeEncoder climbMotorEncoder;
  private final boolean INVERSION_PLACEHOLDER = false;

  public Climber() {
    climbMotor = new CANSparkMax(RobotMap.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);
    climbMotorEncoder = climbMotor.getEncoder();
    climbMotorEncoder.setPosition(0);
    climbMotor.restoreFactoryDefaults();
    climbMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climbMotor.setInverted(
        INVERSION_PLACEHOLDER); // TODO see which way motors are facing and invert such that
    // positive = in

    climbMotor.setSoftLimit(SoftLimitDirection.kForward, -110.0f);
    climbMotor.setSoftLimit(SoftLimitDirection.kReverse, -110.0f);
    climbMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // climbMotorLeft.setSmartCurrentLimit(80);
  }

  public void climbMotorDown() {
    climbMotor.set(1.0);
  }

  public void climbMotorUp() {
    climbMotor.set(-0.5);
  }

  public void holdClimb() {
    climbMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Climber Power",
        () -> {
          return climbMotor.get();
        },
        null);
    builder.addDoubleProperty(
        "Climber Position",
        () -> {
          return climbMotorEncoder.getPosition();
        },
        null);
  }
}
