package net.cachemoney8096.frc2022o.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.libs.PicoColorSensor;
import net.cachemoney8096.frc2022o.libs.CargoColor;
import net.cachemoney8096.frc2022o.libs.CargoColorDifferentiator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import net.cachemoney8096.frc2022o.libs.CargoStateManager;

public class Intake extends SubsystemBase {

  // Actuators
  private final CANSparkMax intakeMotorOne;
  private final CANSparkMax intakeMotorTwo;
  private final CANSparkMax intakeMotorThree;
  private final Compressor compressor;
  private final Solenoid intakeSolenoidLeft;
  private final Solenoid intakeSolenoidRight;

  // Sensors
  private final DigitalInput cargoSensor;
  private PicoColorSensor colorSensor;

  // Members
  private CargoColorDifferentiator cargoColorDifferentiator = new CargoColorDifferentiator();
  private Optional<Timer> ejectTimer;
  /** Indexer stored just to see sensor state */
  private Indexer indexer;

  private CargoStateManager cargoStateManager = new CargoStateManager();
  private CargoStateManager.IntakeState prevIntakeState =
      CargoStateManager.IntakeState.NOT_ACTUATING;

  public Intake(Indexer indexerIn) {
    super();
    indexer = indexerIn;

    intakeMotorOne = new CANSparkMax(RobotMap.INTAKE_MOTOR_ONE_ID, MotorType.kBrushless);
    intakeMotorOne.restoreFactoryDefaults();
    intakeMotorOne.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorOne.setInverted(
        false); // TODO see which way motors are facing and invert such that positive = in

    intakeMotorTwo = new CANSparkMax(RobotMap.INTAKE_MOTOR_TWO_ID, MotorType.kBrushless);
    intakeMotorTwo.restoreFactoryDefaults();
    intakeMotorTwo.setIdleMode(CANSparkMax.IdleMode.kCoast);

    intakeMotorThree = new CANSparkMax(RobotMap.INTAKE_MOTOR_THREE_ID, MotorType.kBrushless);
    intakeMotorThree.restoreFactoryDefaults();
    intakeMotorThree.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorThree.follow(intakeMotorTwo);

    compressor = new Compressor(RobotMap.COMPRESSOR_MODULE_ID, PneumaticsModuleType.CTREPCM);
    intakeSolenoidLeft =
        new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.LEFT_INTAKE_SOLENOID_CHANNEL);
    intakeSolenoidRight =
        new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_INTAKE_SOLENOID_CHANNEL);

    cargoSensor = new DigitalInput(RobotMap.INTAKE_CARGO_DIO);
    colorSensor = new PicoColorSensor();
  }

  @Override
  public void periodic() {
    // if all three colors return 0, reinstantiate the color sensor
    // based on https://www.chiefdelphi.com/t/rev-color-sensor-stops-outputting/405153/3
    PicoColorSensor.RawColor sensorColor = colorSensor.getRawColor0();
    if (sensorColor.red == 0 && sensorColor.green == 0 && sensorColor.blue == 0) {
      colorSensor = new PicoColorSensor();
    }

    // Check color sensor
    CargoColor lastColorSeen = CargoColor.UNKNOWN;
    if (colorSensor.getProximity0() < Calibrations.COLOR_SENSOR_PROXIMITY_THRESHOLD) {
      sensorColor = new PicoColorSensor.RawColor();
      colorSensor.getRawColor0(sensorColor);
      lastColorSeen = cargoColorDifferentiator.whatColor(sensorColor);
    }

    // Check cargo sensors
    boolean indexerSeeCargo = indexer.seeCargo();
    boolean intakeSeeCargo = seeCargo();

    // Update cargo states
    CargoStateManager.InputState inputState =
        new CargoStateManager.InputState(
            lastColorSeen, intakeSeeCargo, indexerSeeCargo, prevIntakeState);
    CargoStateManager.RobotCargoState robotCargoState =
        cargoStateManager.updateCargoState(inputState);

    // Decide what the intake should try to do
    // We default to intaking so let's just see if we need to eject
    if (lastColorSeen == CargoColor.THEIRS
        || (robotCargoState.intakeCurrentCargo.isPresent()
            && robotCargoState.intakeCurrentCargo.get() == CargoColor.THEIRS)) {
      // If we saw a wrong-colored cargo or if we're holding a wrong-colored cargo, we should eject
      // it
      if ((robotCargoState.indexerCurrentCargo.isPresent()
              && robotCargoState.indexerCurrentCargo.get() != CargoColor.THEIRS)
          || (robotCargoState.intakeCargoPassedToIndexer.isPresent()
              && robotCargoState.intakeCargoPassedToIndexer.get() != CargoColor.THEIRS)
          || (robotCargoState.intakeCurrentCargo.isPresent()
              && robotCargoState.intakeCurrentCargo.get() != CargoColor.THEIRS)) {
        // There's a cargo we want to keep past the wrong-colored cargo, so we need to eject to the
        // front
        ejectTimer = Optional.of(new Timer());
      }
    }

    // Update Indexer on what to do
    if (robotCargoState.indexerCurrentCargo.isPresent()) {
      // Indexer already has a cargo
      if (robotCargoState.indexerCurrentCargo.get() == CargoColor.THEIRS) {
        // Their cargo, get rid of it
        indexer.setInstruction(Indexer.IndexerInstruction.EJECT);
      } else {
        // Our cargo or an unknown cargo, keep it
        indexer.setInstruction(Indexer.IndexerInstruction.HOLD);
      }
    } else {
      // Indexer does not already have a cargo
      if (robotCargoState.intakeCargoPassedToIndexer.isPresent()) {
        // There is a cargo potentially coming
        if (robotCargoState.intakeCargoPassedToIndexer.get() == CargoColor.THEIRS) {
          // Their cargo, get rid of it
          indexer.setInstruction(Indexer.IndexerInstruction.EJECT);
        } else {
          // Our cargo or an unknown cargo, bring it in
          indexer.setInstruction(Indexer.IndexerInstruction.INDEX);
        }
      }
    }
  }

  /** Returns true if the cargo sensor see a cargo */
  public boolean seeCargo() {
    // Sensor is false if there's a ball
    return !cargoSensor.get();
  }

  public void updateAllianceColor() {
    cargoColorDifferentiator.updateAllianceColor();
  }

  /**
   * Function to call in the event of not trying to intake. Either this or intakeCargo must be
   * called!
   */
  public void dontIntakeCargo() {
    // Stop running and bring the intake in
    // Ignore the ejection timer, the intake must be brought in ASAP
    intakeMotorOne.set(0);
    intakeMotorTwo.set(0);
    prevIntakeState = CargoStateManager.IntakeState.NOT_ACTUATING;
    retractIntake(); // 3 follows 2
  }

  /**
   * Function to call in the event of trying to intake. Either this or DontIntakeCargo must be
   * called!
   */
  public void intakeCargo() {
    extendIntake();
    intakeMotorOne.set(Calibrations.INTAKE_ONE_POWER);
    if (ejectTimer.isEmpty()) {
      // Nothing to eject
      intakeMotorTwo.set(Calibrations.INTAKE_TWO_POWER); // 3 follows 2
      prevIntakeState = CargoStateManager.IntakeState.INTAKING;
    } else if (ejectTimer.get().hasElapsed(Calibrations.EJECT_CARGO_FRONT_SECONDS)) // done ejecting
    {
      // Done ejecting, intake and also clear the timer
      intakeMotorTwo.set(Calibrations.INTAKE_TWO_POWER); // 3 follows 2
      prevIntakeState = CargoStateManager.IntakeState.INTAKING;
      ejectTimer = Optional.empty();
    } else {
      // Eject timer has not finished, keep ejecting
      intakeMotorTwo.set(Calibrations.INTAKE_EJECT_POWER); // 3 follows 2
      prevIntakeState = CargoStateManager.IntakeState.INTAKING;
    }
  }

  private void extendIntake() {
    intakeSolenoidLeft.set(true);
    intakeSolenoidRight.set(true);
  }

  private void retractIntake() {
    intakeSolenoidLeft.set(false);
    intakeSolenoidRight.set(false);
  }
}
