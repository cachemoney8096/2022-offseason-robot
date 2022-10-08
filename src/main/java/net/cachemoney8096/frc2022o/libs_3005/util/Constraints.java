package net.cachemoney8096.frc2022o.libs_3005.util;

public class Constraints {
  public double maxVelocity;

  public double maxAcceleration;

  /**
   * Construct constraints for a TrapezoidProfile.
   *
   * @param maxVelocity maximum velocity
   * @param maxAcceleration maximum acceleration
   */
  public Constraints(double maxVelocity, double maxAcceleration) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
  }
}
