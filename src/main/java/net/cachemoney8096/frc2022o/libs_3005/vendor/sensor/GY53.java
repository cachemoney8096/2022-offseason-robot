package net.cachemoney8096.frc2022o.libs_3005.vendor.sensor;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Counter.Mode;

public class GY53 implements DistanceSensor, Sendable {
  private final Counter m_counter;

  public GY53(int channel) {
    m_counter = new Counter(Mode.kSemiperiod);
    m_counter.setUpSource(channel);
    m_counter.setSemiPeriodMode(true);

    // Datasheet max period is 22ms, give plenty of margin. This is used to detect if the sensor is
    // present at all.
    m_counter.setMaxPeriod(0.25);
  }

  @Override
  public double getDistanceMillimeters() {
    // http://wiki.sunfounder.cc/index.php?title=GY-53_VL53L0X_Laser_ToF_Flight_Time_Range_Sensor_Module_Serial_PWM_Output
    // 0.1mm per microsecond, getPeriod() returns seconds
    return m_counter.getPeriod() * 1e5;
  }

  @Override
  public boolean isConnected() {
    return !m_counter.getStopped();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Distance mm", this::getDistanceMillimeters, null);
    builder.addBooleanProperty("Connected", this::isConnected, null);
  }
}
