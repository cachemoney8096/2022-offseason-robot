package net.cachemoney8096.frc2022o.libs_3005.vendor.sensor;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface ColorDistanceSensor extends Sendable, DistanceSensor, ColorSensor {

  @Override
  public default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("R", () -> getColor().red, null);
    builder.addDoubleProperty("G", () -> getColor().green, null);
    builder.addDoubleProperty("B", () -> getColor().blue, null);
    builder.addDoubleProperty("Distance mm", () -> getDistanceMillimeters(), null);
  }

  @Override
  default boolean isConnected() {
    return DistanceSensor.super.isConnected() && ColorSensor.super.isConnected();
  }
}
