package net.cachemoney8096.frc2022o.libs_3005.util;

import org.tinylog.Logger;

public class ThreadUtils {
  public static boolean sleep(long millis) {
    try {
      Thread.sleep(millis);
    } catch (Exception e) {
      Logger.debug(e);
      return false;
    }
    return true;
  }
}
