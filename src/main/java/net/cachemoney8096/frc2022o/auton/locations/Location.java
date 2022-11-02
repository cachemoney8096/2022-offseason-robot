package net.cachemoney8096.frc2022o.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class Location {
  public abstract Pose2d get();

  public String getName() {
    String name = this.getClass().getSimpleName();
    return name.substring(name.lastIndexOf('.') + 1);
  }

  public static RightStart RightStart() {
    return RightStart.instance;
  }

  public static CenterStart CenterStart() {
    return CenterStart.instance;
  }

  public static LeftStart LeftStart() {
    return LeftStart.instance;
  }
}
