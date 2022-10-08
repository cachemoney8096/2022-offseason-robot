package net.cachemoney8096.frc2022o.libs_3005.vendor.sensor;

import edu.wpi.first.wpilibj.util.Color;
import net.cachemoney8096.frc2022o.libs_3005.monitor.IsConnected;

public interface ColorSensor extends IsConnected {
  public Color getColor();
}
