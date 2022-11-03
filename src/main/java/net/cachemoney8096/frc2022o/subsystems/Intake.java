package net.cachemoney8096.frc2022o.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.libs.PicoColorSensor;
import net.cachemoney8096.frc2022o.libs.CargoStateManager.IntakeState;
import net.cachemoney8096.frc2022o.libs.CargoColor;
import net.cachemoney8096.frc2022o.libs.CargoColorDifferentiator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import net.cachemoney8096.frc2022o.libs.CargoStateManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {

  // Actuators
  private final CANSparkMax intakeMotorOne;
  private final CANSparkMax intakeMotorTwo;
  private final CANSparkMax intakeMotorThree;
  private final Compressor compressor;

  
  private final Solenoid intakeSolenoidForward;
  private final Solenoid intakeSolenoidReverse;

  // Sensors
  private final DigitalInput cargoSensor;
  private PicoColorSensor colorSensor;

  // Members
  private CargoColorDifferentiator cargoColorDifferentiator = new CargoColorDifferentiator();
  private Optional<Timer> ejectTimer = Optional.empty();
  private boolean intakeExtended = false;
  /** Indexer stored just to see sensor state */
  private Indexer indexer;

  private CargoStateManager cargoStateManager = new CargoStateManager();
  private CargoStateManager.IntakeState prevIntakeState = CargoStateManager.IntakeState.NOT_ACTUATING;
  private CargoStateManager.IntakeState nextIntakeState = CargoStateManager.IntakeState.NOT_ACTUATING;
  private boolean ejectionShouldBePartial = false;

  public Intake(Indexer indexerIn) {
    super();
    indexer = indexerIn;

    intakeMotorOne = new CANSparkMax(RobotMap.INTAKE_MOTOR_ONE_ID, MotorType.kBrushless);
    intakeMotorOne.restoreFactoryDefaults();
    intakeMotorOne.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorOne.setInverted(true);

    intakeMotorTwo = new CANSparkMax(RobotMap.INTAKE_MOTOR_TWO_ID, MotorType.kBrushless);
    intakeMotorTwo.restoreFactoryDefaults();
    intakeMotorTwo.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorTwo.setInverted(false);

    intakeMotorThree = new CANSparkMax(RobotMap.INTAKE_MOTOR_THREE_ID, MotorType.kBrushless);
    intakeMotorThree.restoreFactoryDefaults();
    intakeMotorThree.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorThree.setInverted(true);

    compressor = new Compressor(RobotMap.COMPRESSOR_MODULE_ID, PneumaticsModuleType.CTREPCM);
    intakeSolenoidForward = new Solenoid(PneumaticsModuleType.CTREPCM,
    RobotMap.LEFT_INTAKE_SOLENOID_CHANNEL_FORWARD);
    intakeSolenoidReverse = new Solenoid(PneumaticsModuleType.CTREPCM,
    RobotMap.LEFT_INTAKE_SOLENOID_CHANNEL_REVERSE);

    cargoSensor = new DigitalInput(RobotMap.INTAKE_CARGO_DIO);
    colorSensor = new PicoColorSensor();
  }

  @Override
  public void periodic() {
    // if all three colors return 0, reinstantiate the color sensor
    // based on
    // https://www.chiefdelphi.com/t/rev-color-sensor-stops-outputting/405153/3
    PicoColorSensor.RawColor sensorColor = colorSensor.getRawColor0();
    if (colorSensor.isSensor0Connected() && sensorColor.red == 0 && sensorColor.green == 0 && sensorColor.blue == 0) {
      if (RobotBase.isReal()) {
        colorSensor = new PicoColorSensor();
      }
    }

    // Check color sensor
    CargoColor lastColorSeen = CargoColor.UNKNOWN;
    if (colorSensor.getProximity0() > Calibrations.COLOR_SENSOR_PROXIMITY_THRESHOLD) {
      sensorColor = new PicoColorSensor.RawColor();
      colorSensor.getRawColor0(sensorColor);
      lastColorSeen = cargoColorDifferentiator.whatColor(sensorColor);
    }

    // Check cargo sensors
    boolean indexerSeeCargo = indexer.seeCargo();
    boolean intakeSeeCargo = seeCargo();

    // Update cargo states
    CargoStateManager.InputState inputState = new CargoStateManager.InputState(
        lastColorSeen, intakeSeeCargo, indexerSeeCargo, prevIntakeState);
    CargoStateManager.RobotCargoState robotCargoState = cargoStateManager.updateCargoState(inputState);

    // Determine what the intake should do
    nextIntakeState = whatShouldIntakeDo(lastColorSeen, robotCargoState);
    
    // If we need to eject, reset the timer
    if (nextIntakeState == IntakeState.EJECTING) {
      ejectTimer = Optional.of(new Timer());
      ejectTimer.get().start();
    }
    
    // If we're set on cargo (already have two of ours), then any ejection should be partial
    // meaning we should keep the cargo in the intake
    ejectionShouldBePartial = nextIntakeState == IntakeState.NOT_ACTUATING;

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
    } else if (robotCargoState.intakeCargoPassedToIndexer.isPresent()) {
      // Indexer does not already have a cargo but there is a cargo potentially coming
      if (robotCargoState.intakeCargoPassedToIndexer.get() == CargoColor.THEIRS) {
        // Their cargo, get rid of it
        indexer.setInstruction(Indexer.IndexerInstruction.EJECT);
      } else {
        // Our cargo or an unknown cargo, bring it in
        indexer.setInstruction(Indexer.IndexerInstruction.INDEX);
      }
    } else {
      // No cargo in indexer and none coming, just hold
      indexer.setInstruction(Indexer.IndexerInstruction.HOLD);
    }
  }

  /**
   * Decide what the intake should try to do based on the cargo
   * This assumes that intaking is being requested.
   */
  private IntakeState whatShouldIntakeDo(CargoColor lastColorSeen, CargoStateManager.RobotCargoState robotCargoState) {
    if ((lastColorSeen == CargoColor.THEIRS) || (robotCargoState.intakeCurrentCargo.isPresent()
        && robotCargoState.intakeCurrentCargo.get() == CargoColor.THEIRS)) {
      // We are seeing or holding a wrong-colored cargo
      if ((robotCargoState.indexerCurrentCargo.isPresent()
          && robotCargoState.indexerCurrentCargo.get() != CargoColor.THEIRS)
          || (robotCargoState.intakeCargoPassedToIndexer.isPresent()
              && robotCargoState.intakeCargoPassedToIndexer.get() != CargoColor.THEIRS)
          || (robotCargoState.intakeCurrentCargo.isPresent()
              && robotCargoState.intakeCurrentCargo.get() != CargoColor.THEIRS)) {
        // There's a cargo we want to keep past the wrong-colored cargo, so we need to
        // eject to the front
        return IntakeState.EJECTING;
      } else {
        // We can just eject out the back
        return IntakeState.INTAKING;
      }
    } else if ((robotCargoState.intakeCurrentCargo.isPresent()
        && robotCargoState.intakeCurrentCargo.get() != CargoColor.THEIRS) &&
        (robotCargoState.indexerCurrentCargo.isPresent()
            && robotCargoState.indexerCurrentCargo.get() != CargoColor.THEIRS)) {
      // Already holding two balls, quit intaking
      return IntakeState.NOT_ACTUATING;
    } else {
      // No issues, just intake!
      return IntakeState.INTAKING;
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
   * Function to call in the event of not trying to intake. Either this or
   * intakeCargo must be called!
   */
  public void dontIntakeCargo() {
    // Stop running and bring the intake in
    // Ignore the ejection timer, the intake must be brought in ASAP
    intakeMotorOne.set(0);
    intakeMotorTwo.set(0);
    intakeMotorThree.set(0);
    prevIntakeState = CargoStateManager.IntakeState.NOT_ACTUATING;
    retractIntake();
  }

  /**
   * Function to call in the event of trying to intake. Either this or
   * DontIntakeCargo must be called!
   */
  public void intakeCargo() {
    extendIntake();
    if (ejectTimer.isPresent()) {
      // Started ejecting at some point
      if (ejectTimer.get().hasElapsed(Calibrations.EJECT_CARGO_FRONT_SECONDS)) {
        // Ejection timer did finish, clear it and continue
        ejectTimer = Optional.empty();
      } else {
        // Eject timer has not finished, keep ejecting
        intakeMotorOne.set(Calibrations.INTAKE_ONE_POWER);
        intakeMotorTwo.set(Calibrations.INTAKE_EJECT_POWER);
        if (ejectionShouldBePartial) {
          intakeMotorThree.set(0.0);
        }
        else {
          intakeMotorThree.set(Calibrations.INTAKE_EJECT_POWER);
        }
        prevIntakeState = CargoStateManager.IntakeState.EJECTING;
        return;
      }
    }

    switch (nextIntakeState) {
      case EJECTING:
        // This really should not happen, but we can use the same logic from above
        intakeMotorOne.set(Calibrations.INTAKE_ONE_POWER);
        intakeMotorTwo.set(Calibrations.INTAKE_EJECT_POWER);
        if (ejectionShouldBePartial) {
          intakeMotorThree.set(0.0);
        }
        else {
          intakeMotorThree.set(Calibrations.INTAKE_EJECT_POWER);
        }
        break;
      case NOT_ACTUATING:
        // All set, no actuation
        intakeMotorOne.set(0);
        intakeMotorTwo.set(0);
        intakeMotorThree.set(0);
        break;
      case INTAKING:
        // Bring in cargo
        intakeMotorOne.set(Calibrations.INTAKE_ONE_POWER);
        intakeMotorTwo.set(Calibrations.INTAKE_TWO_POWER);
        intakeMotorThree.set(Calibrations.INTAKE_TWO_POWER);
        break;
    }
    prevIntakeState = nextIntakeState;
  }

  public void extendIntake() {
    intakeSolenoidForward.set(true);
    intakeSolenoidReverse.set(false);
    intakeExtended = true;
  }

  public void retractIntake() {
    intakeSolenoidForward.set(false);
    intakeSolenoidReverse.set(true);
    intakeExtended = false;
  }

  /** Gives a string description of the eject timer */
  private String ejectTimerStatus() {
    if (ejectTimer.isPresent()) {
      return String.format(
          "Time %f, has elapsed %b",
          ejectTimer.get().get(),
          ejectTimer.get().hasElapsed(Calibrations.EJECT_CARGO_BACK_SECONDS));
    } else {
      return "No eject timer";
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty(
        "Intake Solenoids",
        () -> {
          return intakeExtended;
        },
        null);
    builder.addStringProperty("Intake Eject Timer", this::ejectTimerStatus, null);
    addChild("Cargo State Manager", cargoStateManager);
    builder.addStringProperty("Intake state", () -> prevIntakeState.name(), null);
  }

  public void runAllIntakeBackwardsOverride() {
    intakeMotorOne.set(Calibrations.INTAKE_BACKWARDS_POWER);
    intakeMotorTwo.set(Calibrations.INTAKE_BACKWARDS_POWER);
    intakeMotorThree.set(Calibrations.INTAKE_BACKWARDS_POWER);
  }

  public void runAllIntakeForwardsOverride() {
    intakeMotorOne.set(Calibrations.INTAKE_FORWARDS_POWER);
    intakeMotorTwo.set(Calibrations.INTAKE_FORWARDS_POWER);
    intakeMotorThree.set(Calibrations.INTAKE_FORWARDS_POWER);
  }
}
