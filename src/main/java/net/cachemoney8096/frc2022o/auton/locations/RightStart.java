package net.cachemoney8096.frc2022o.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RightStart extends Location {
  private RightStart() {}

  @Override
  public Pose2d get() {
    return new Pose2d(5.95, 3.86, Rotation2d.fromDegrees(-90.0));
  }

  public static final RightStart instance = new RightStart();
}
