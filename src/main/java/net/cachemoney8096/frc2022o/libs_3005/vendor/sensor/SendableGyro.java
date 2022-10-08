package net.cachemoney8096.frc2022o.libs_3005.vendor.sensor;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public interface SendableGyro extends Gyro, Sendable {
  public void setAngle(double angleDegreesCCWPositive);
}
