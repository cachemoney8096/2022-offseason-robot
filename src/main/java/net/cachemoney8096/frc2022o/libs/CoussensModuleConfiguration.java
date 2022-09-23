package net.cachemoney8096.frc2022o.libs;

import java.util.Objects;

/**
 * Additional swerve module configuration parameters.
 *
 * <p>Almost exactly like the Mk3ModuleConfiguration. The configuration parameters here are used to
 * customize the behavior of the swerve module. Each setting is initialized to a default that should
 * be adequate for most use cases.
 */
public class CoussensModuleConfiguration {
  private double nominalVoltage = 12.0;
  private double driveCurrentLimit = 80.0;
  private double steerCurrentLimit = 20.0;

  public double getNominalVoltage() {
    return nominalVoltage;
  }

  public void setNominalVoltage(double nominalVoltage) {
    this.nominalVoltage = nominalVoltage;
  }

  public double getDriveCurrentLimit() {
    return driveCurrentLimit;
  }

  public void setDriveCurrentLimit(double driveCurrentLimit) {
    this.driveCurrentLimit = driveCurrentLimit;
  }

  public double getSteerCurrentLimit() {
    return steerCurrentLimit;
  }

  public void setSteerCurrentLimit(double steerCurrentLimit) {
    this.steerCurrentLimit = steerCurrentLimit;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    CoussensModuleConfiguration that = (CoussensModuleConfiguration) o;
    return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0
        && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0
        && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
  }

  @Override
  public int hashCode() {
    return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
  }

  @Override
  public String toString() {
    return "CoussensModuleConfiguration{"
        + "nominalVoltage="
        + nominalVoltage
        + ", driveCurrentLimit="
        + driveCurrentLimit
        + ", steerCurrentLimit="
        + steerCurrentLimit
        + '}';
  }
}
