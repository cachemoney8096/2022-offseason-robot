package net.cachemoney8096.frc2022o.libs_3005.electromechanical;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface AbsoluteEncoder extends Sendable {
  public double getPosition();

  public void setPositionOffset(double position);

  public default boolean isConnected() {
    return true;
  }

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position", () -> this.getPosition(), null);
  }
}
