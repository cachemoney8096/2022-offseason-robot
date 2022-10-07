package net.cachemoney8096.frc2022o.libs_3005.electromechanical;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface Encoder extends Sendable {
  public double getVelocity();

  public double getPosition();

  public void setPosition(double position);

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position", () -> this.getPosition(), null);
    builder.addDoubleProperty("Velocity", () -> this.getVelocity(), null);
  }
}
