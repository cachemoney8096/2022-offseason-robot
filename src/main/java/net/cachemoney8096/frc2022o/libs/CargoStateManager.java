package net.cachemoney8096.frc2022o.libs;

import java.util.Optional;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * This class takes info from the intake and indexer to determine all the cargo states of the robot
 */
public class CargoStateManager implements Sendable {
  /** Whether the intake is bringing cargo in, out, or neither */
  public enum IntakeState {
    INTAKING,
    EJECTING,
    NOT_ACTUATING
  }

  public static String optionalCargoToString(Optional<CargoColor> inputCargo) {
    if (inputCargo.isPresent()) {
      return "none";
    } else {
      return inputCargo.get().name();
    }
  }

  /** Sensor readings */
  public static class InputState {
    /** Whether the intake color sensor sees a color (unknown for no color) */
    public CargoColor intakeLastColorSeen;

    /** Whether the intake ball sensor sees a cargo */
    public boolean intakeSeeCargo;

    /** Whether the indexer ball sensor sees a cargo */
    public boolean indexerSeeCargo;

    /** In the previous cycle, which way the intake was going */
    public IntakeState intakeState;

    public InputState(
        CargoColor intakeLastColorSeenIn,
        boolean intakeSeeCargoIn,
        boolean indexerSeeCargoIn,
        IntakeState intakeStateIn) {
      intakeLastColorSeen = intakeLastColorSeenIn;
      intakeSeeCargo = intakeSeeCargoIn;
      indexerSeeCargo = indexerSeeCargoIn;
      intakeState = intakeStateIn;
    }
  }

  /** Class containing the state of all cargo in the robot */
  public class RobotCargoState {
    /** Last color seen by the color sensor, indicating what the intake might see */
    public Optional<CargoColor> intakeLastColorSeen = Optional.empty();

    /** The color of cargo currently held by the intake */
    public Optional<CargoColor> intakeCurrentCargo = Optional.empty();

    /**
     * The color of a cargo sent along by the intake, indicating what the indexer might see. This
     * may be a cargo still seen by the intake!
     */
    public Optional<CargoColor> intakeCargoPassedToIndexer = Optional.empty();

    /** The color of cargo currently held by the indexer */
    public Optional<CargoColor> indexerCurrentCargo = Optional.empty();

    public RobotCargoState() {}

    public RobotCargoState(
        Optional<CargoColor> intakeLastColorSeenIn,
        Optional<CargoColor> intakeCurrentCargoIn,
        Optional<CargoColor> intakeCargoPassedToIndexerIn,
        Optional<CargoColor> indexerCurrentCargoIn) {
      intakeLastColorSeen = intakeLastColorSeenIn;
      intakeCurrentCargo = intakeCurrentCargoIn;
      intakeCargoPassedToIndexer = intakeCargoPassedToIndexerIn;
      indexerCurrentCargo = indexerCurrentCargoIn;
    }
  }

  private RobotCargoState robotCargoState = new RobotCargoState();

  /** Override for robot cargo state, mostly for initialization purposes */
  public void overrideCargoState(CargoColor intakeCargoColorIn, CargoColor indexerCargoColorIn) {
    robotCargoState.intakeLastColorSeen = Optional.empty();
    robotCargoState.intakeCurrentCargo = Optional.of(intakeCargoColorIn);
    robotCargoState.intakeCargoPassedToIndexer = Optional.empty();
    robotCargoState.indexerCurrentCargo = Optional.of(indexerCargoColorIn);
  }

  /**
   * Figure out the new state of the robot's cargo, working back-to-front starting with the indexer
   */
  public RobotCargoState updateCargoState(InputState inputState) {
    // First, update the indexer
    // Only change anything if the new observation does not match the old state
    if (inputState.indexerSeeCargo ^ robotCargoState.indexerCurrentCargo.isPresent()) {
      if (inputState.indexerSeeCargo) {
        // We see a cargo we didn't see before.
        if (robotCargoState.intakeCargoPassedToIndexer.isPresent()) {
          // A cargo was passed from the intake, that's probably the indexer's cargo
          robotCargoState.indexerCurrentCargo = robotCargoState.intakeCargoPassedToIndexer;
          robotCargoState.intakeCargoPassedToIndexer = Optional.empty();
        } else {
          // No cargo was passed from the intake, weird..
          robotCargoState.indexerCurrentCargo = Optional.of(CargoColor.UNKNOWN);
        }
      } else {
        // A cargo we previously saw has disappeared
        robotCargoState.indexerCurrentCargo = Optional.empty();
      }
    }

    // Next, update the "cargo passed to indexer"
    // We only update if:
    // (a) we are trying to pass a cargo to the indexer and
    // (b) we have a cargo to pass and
    // (c) we have not already passed a cargo (don't update until the forward cargo
    // has reached the
    // indexer)
    if (inputState.intakeState == IntakeState.INTAKING
        && robotCargoState.intakeCurrentCargo.isPresent()
        && robotCargoState.intakeCargoPassedToIndexer.isEmpty()) {
      robotCargoState.intakeCargoPassedToIndexer = robotCargoState.intakeCurrentCargo;
    }

    // Next, update the Intake cargo
    // Similar to Indexer, we update only if things have changed
    if (inputState.intakeSeeCargo ^ robotCargoState.intakeCurrentCargo.isPresent()) {
      if (inputState.intakeSeeCargo) {
        // We see a cargo we didn't see before.
        if (robotCargoState.intakeLastColorSeen.isPresent())
          // We saw this cargo, that's probably what's here now
          robotCargoState.intakeCurrentCargo = robotCargoState.intakeLastColorSeen;
        robotCargoState.intakeLastColorSeen = Optional.empty();
      } else {
        // We didn't see the cargo that was passed in, weird..
        robotCargoState.intakeCurrentCargo = Optional.of(CargoColor.UNKNOWN);
      }
    } else {
      // A cargo we previously saw has disappeared
      robotCargoState.intakeCurrentCargo = Optional.empty();
    }

    // Finally, we update the "last color seen" from the color sensor
    if (inputState.intakeLastColorSeen != CargoColor.UNKNOWN) {
      // We actually saw a color, let's update
      robotCargoState.intakeLastColorSeen = Optional.of(inputState.intakeLastColorSeen);
    }

    return robotCargoState;
  }

  private Optional<CargoColor> convertStringToCargoColor(String cargoString) {
    String lowercaseCargoString = cargoString.toLowerCase();
    if (lowercaseCargoString == "ours") {
      return Optional.of(CargoColor.OURS);
    } else if (lowercaseCargoString == "theirs") {
      return Optional.of(CargoColor.OURS);
    } else if (lowercaseCargoString == "unknown") {
      return Optional.of(CargoColor.UNKNOWN);
    } else if (lowercaseCargoString == "none") {
      return Optional.empty();
    } else {
      return Optional.empty();
    }
  }

  private void overrideIntakeCargo(String intakeCargo) {
    robotCargoState.intakeCurrentCargo = convertStringToCargoColor(intakeCargo);
  }

  private void overrideIndexerCargo(String indexerCargo) {
    robotCargoState.indexerCurrentCargo = convertStringToCargoColor(indexerCargo);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "Intake Last Color Seen",
        () -> optionalCargoToString(robotCargoState.intakeLastColorSeen),
        null);
    builder.addStringProperty(
        "Intake Current Cargo",
        () -> optionalCargoToString(robotCargoState.intakeCurrentCargo),
        this::overrideIntakeCargo);
    builder.addStringProperty(
        "Cargo passed intake to indexer",
        () -> optionalCargoToString(robotCargoState.intakeCargoPassedToIndexer),
        null);
    builder.addStringProperty(
        "Indexer Current Cargo",
        () -> optionalCargoToString(robotCargoState.indexerCurrentCargo),
        this::overrideIndexerCargo);
  }
}
