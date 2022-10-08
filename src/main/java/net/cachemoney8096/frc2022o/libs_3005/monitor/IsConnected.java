package net.cachemoney8096.frc2022o.libs_3005.monitor;

public interface IsConnected {

  /**
   * Return true if the device is connected. Some components may not have this capability, so a
   * default of returning true is used.
   *
   * @return true if the device is connected.
   */
  public default boolean isConnected() {
    return true;
  }
}
