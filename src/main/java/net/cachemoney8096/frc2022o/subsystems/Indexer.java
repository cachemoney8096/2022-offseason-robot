package net.cachemoney8096.frc2022o.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.RobotMap;

public class Indexer extends SubsystemBase {

  // Actuators
  private final CANSparkMax indexerMotorOne;
  private final CANSparkMax indexerMotorTwo;
  private final CANSparkMax indexerMotorThree;

  // Sensors
  private final DigitalInput cargoSensor;

  public Indexer() {
    indexerMotorOne = new CANSparkMax(RobotMap.INDEXER_MOTOR_ONE_ID, MotorType.kBrushless);
    indexerMotorOne.restoreFactoryDefaults();
    indexerMotorOne.setIdleMode(CANSparkMax.IdleMode.kCoast);
    indexerMotorOne.setInverted(false);  // TODO see which way motors are facing and invert such that positive = in
    // indexerMotorOne.setSmartCurrentLimit(25);
    // following line used to reduce CAN utilization?
    // indexerMotorOne.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);

    indexerMotorTwo = new CANSparkMax(RobotMap.INDEXER_MOTOR_TWO_ID, MotorType.kBrushless);
    indexerMotorTwo.restoreFactoryDefaults();
    indexerMotorTwo.setIdleMode(CANSparkMax.IdleMode.kCoast);
    indexerMotorTwo.follow(indexerMotorOne);

    indexerMotorThree = new CANSparkMax(RobotMap.INDEXER_MOTOR_THREE_ID, MotorType.kBrushless);
    indexerMotorThree.restoreFactoryDefaults();
    indexerMotorThree.setIdleMode(CANSparkMax.IdleMode.kCoast);

    cargoSensor = new DigitalInput(RobotMap.INDEXER_CARGO_DIO);
  }

  @Override
  public void periodic() {
    // check for a ball
  }

  public boolean hasCargo() {
    return cargoSensor.get();
  }

  public void indexBall() {
    // run one/two forward
    // don't run three, unless there's a wrong-colored ball in which case run backwards for a second
  }

  public void ejectBall() {
    // run one/two forward
    // run three backwards
  }

  public void feedShooter() {
    // run all three forwards
  }
}
