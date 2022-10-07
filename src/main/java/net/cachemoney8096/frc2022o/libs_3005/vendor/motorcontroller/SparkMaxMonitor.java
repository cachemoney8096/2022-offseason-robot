// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o.libs_3005.vendor.motorcontroller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.tinylog.Logger;

/** This is a basic monitor class separate from the HealthMonitor setup. */
public class SparkMaxMonitor extends SubsystemBase {
  private HashMap<SparkMax, Short> m_sparkMaxs = new HashMap<>();
  private int m_runCount = 0;

  /** Creates a new SparkMaxMonitor. */
  public SparkMaxMonitor() {}

  public boolean add(SparkMax sparkMax) {
    if (m_sparkMaxs.containsKey(sparkMax)) {
      return false;
    }
    m_sparkMaxs.put(sparkMax, (short) 0);
    return true;
  }

  @Override
  public void periodic() {
    // Run at 1 second
    if (m_runCount++ < 50) {
      return;
    }
    m_runCount = 0;

    m_sparkMaxs.forEach(
        (sparkMax, prevFault) -> {
          short faults = sparkMax.getStickyFaults();
          if (faults != prevFault.shortValue()) {
            Logger.tag("Spark Max Monitor")
                .warn(
                    "Spark Max ID {} faults: {}",
                    sparkMax.getDeviceId(),
                    SparkMaxUtils.faultWordToString(faults));
          }
          m_sparkMaxs.put(sparkMax, faults);
        });
  }
}
