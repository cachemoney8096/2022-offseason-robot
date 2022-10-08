package net.cachemoney8096.frc2022o.libs_3005.vendor.sensor;

import net.cachemoney8096.frc2022o.libs_3005.monitor.IsConnected;

public interface DistanceSensor extends IsConnected {
  public default double getDistanceInches() {
    return getDistanceMillimeters() * 25.4;
  }

  public double getDistanceMillimeters();
}
