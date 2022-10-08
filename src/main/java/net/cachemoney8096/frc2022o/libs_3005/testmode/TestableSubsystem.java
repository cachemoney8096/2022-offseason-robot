package net.cachemoney8096.frc2022o.libs_3005.testmode;

public interface TestableSubsystem {
  public void testModePeriodic(boolean globalEnable);

  public boolean selfTest();
}
