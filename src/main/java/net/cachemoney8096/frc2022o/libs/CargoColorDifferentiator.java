package net.cachemoney8096.frc2022o.libs;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

public class CargoColorDifferentiator {
  public enum Color {
    BLUE,
    RED
  }

  private Optional<Color> ourAllianceColor = Optional.empty();

  // Added to whichever is our color, which is in 0-255.
  // This results in a bias towards thinking cargo are ours.
  private final int COLOR_OFFSET = 0;

  // If ratio is above this, consider the ball known
  private final double COLOR_RATIO_THRESHOLD = 1.5;

  // Updates the alliance color if available.
  // This should only be run in disabled periodic.
  public void updateAllianceColor() {
    ourAllianceColor = getAllianceColor();
  }

  public CargoColor whatColor(PicoColorSensor.RawColor inputColor) {
    if (ourAllianceColor.isEmpty()) {
      // If we're not sure what color we are, just use all colors.
      return CargoColor.OURS;
    }

    // Adjust colors to alliances
    int ourColor = ourAllianceColor.get() == Color.BLUE ? inputColor.blue + COLOR_OFFSET
        : inputColor.red + COLOR_OFFSET;
    int theirColor = ourAllianceColor.get() == Color.BLUE ? inputColor.red : inputColor.blue;

    // Apply ratio threshold
    if (ourColor / theirColor > COLOR_RATIO_THRESHOLD)
      return CargoColor.OURS;
    else if (theirColor / ourColor > COLOR_RATIO_THRESHOLD)
      return CargoColor.THEIRS;
    else
      return CargoColor.UNKNOWN;
  }

  // Gets the alliance color if available.
  // This should only be run in disabled periodic.
  private static Optional<Color> getAllianceColor() {
    if (DriverStation.isDSAttached()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        return Optional.of(Color.RED);
      } else if (DriverStation.getAlliance() == Alliance.Blue) {
        return Optional.of(Color.BLUE);
      }
    }
    return Optional.empty();
  }
}