package net.cachemoney8096.frc2022o.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class Indexer extends SubsystemBase {
  /** For instructions on what to do from the intake with current or next cargo */
  static public enum IndexerInstruction {
    HOLD,
    INDEX,
    EJECT,
  }

  // Actuators
  private final CANSparkMax indexerMotorOne;
  private final CANSparkMax indexerMotorTwo;
  private final CANSparkMax indexerMotorThree;

  // Sensors
  private final DigitalInput cargoSensor;

  // Members
  private IndexerInstruction instructionFromIntake = IndexerInstruction.HOLD;
  private Optional<Timer> ejectTimer = Optional.empty();

  public Indexer() {
    indexerMotorOne = new CANSparkMax(RobotMap.INDEXER_MOTOR_ONE_ID, MotorType.kBrushless);
    indexerMotorOne.restoreFactoryDefaults();
    indexerMotorOne.setIdleMode(CANSparkMax.IdleMode.kCoast);
    indexerMotorOne.setInverted(
        false); // TODO see which way motors are facing and invert such that positive = in
    // indexerMotorOne.setSmartCurrentLimit(25);
    // following line used to reduce CAN utilization?
    // indexerMotorOne.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);

    indexerMotorTwo = new CANSparkMax(RobotMap.INDEXER_MOTOR_TWO_ID, MotorType.kBrushless);
    indexerMotorTwo.restoreFactoryDefaults();
    indexerMotorTwo.setIdleMode(CANSparkMax.IdleMode.kCoast);
    indexerMotorTwo.follow(indexerMotorOne);

    indexerMotorThree = new CANSparkMax(RobotMap.INDEXER_MOTOR_THREE_ID, MotorType.kBrushless);
    indexerMotorThree.restoreFactoryDefaults();
    indexerMotorThree.setIdleMode(CANSparkMax.IdleMode.kBrake); // to keep balls from leaving 

    cargoSensor = new DigitalInput(RobotMap.INDEXER_CARGO_DIO);
  }

  @Override
  public void periodic() {
  }

  /** Instructions coming from the intake and the overall cargo state manager. Must be set each cycle! */
  public void setInstruction(IndexerInstruction instructionFromIntakeIn) {
    instructionFromIntake = instructionFromIntakeIn;
  }

  /** Returns true if the cargo sensor see a cargo */
  public boolean seeCargo() {
    // Sensor is false if there's a ball
    return !cargoSensor.get();
  }

  /** Default command if we're not shooting, finishes ejections then follows intake instructions */
  public void indexBall() {
    if (ejectTimer.isPresent()) {
      if (ejectTimer.get().hasElapsed(Calibrations.EJECT_CARGO_BACK_SECONDS)) {
        // Ejection done! Continue.
        ejectTimer = Optional.empty();
      }
      else {
        // Still ejecting
        indexerMotorOne.set(Calibrations.INDEXER_ONE_POWER); // 2 follows 1
        indexerMotorThree.set(Calibrations.INDEXER_EJECT_POWER);
        return;
      }
    }

    if (instructionFromIntake == IndexerInstruction.HOLD) {
      // Hold => all motors do nothing
      indexerMotorOne.set(0); // 2 follows 1
      indexerMotorThree.set(0);
    }
    else if (instructionFromIntake == IndexerInstruction.EJECT) {
      // Should eject => set new ejection timer and start ejecting!
      ejectTimer = Optional.of(new Timer());
      indexerMotorOne.set(Calibrations.INDEXER_ONE_POWER); // 2 follows 1
      indexerMotorThree.set(Calibrations.INDEXER_EJECT_POWER);
    }
    else {
      // Index => Only run 1/2, 3 stays steady to not let the ball go into
      // the shooter or get ejected
      indexerMotorOne.set(Calibrations.INDEXER_ONE_POWER); // 2 follows 1
      indexerMotorThree.set(0);
    }
  }

  /** Command if we're shooting, finishes ejections then sends cargo to the shooter */
  public void feedShooter() {
    if (ejectTimer.isPresent()) {
      if (ejectTimer.get().hasElapsed(Calibrations.EJECT_CARGO_BACK_SECONDS)) {
        // Ejection done! Continue.
        ejectTimer = Optional.empty();
      }
      else {
        // Still ejecting
        indexerMotorOne.set(Calibrations.INDEXER_ONE_POWER); // 2 follows 1
        indexerMotorThree.set(Calibrations.INDEXER_EJECT_POWER);
        return;
      }
    }

    if (instructionFromIntake == IndexerInstruction.HOLD ||
        instructionFromIntake == IndexerInstruction.INDEX) {
      // "Hold" or "index" while shooting means send everything in
      indexerMotorOne.set(Calibrations.INDEXER_ONE_POWER); // 2 follows 1
      indexerMotorThree.set(Calibrations.INDEXER_THREE_POWER);
    }
    else {
      // Should eject => set new ejection timer and start ejecting!
      ejectTimer = Optional.of(new Timer());
      indexerMotorOne.set(Calibrations.INDEXER_ONE_POWER); // 2 follows 1
      indexerMotorThree.set(Calibrations.INDEXER_EJECT_POWER);
    }
  }
}
