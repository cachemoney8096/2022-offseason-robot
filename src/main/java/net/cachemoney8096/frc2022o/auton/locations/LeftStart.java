package net.cachemoney8096.frc2022o.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LeftStart extends Location {
  private LeftStart() {}

  @Override
  public Pose2d get() {
    return new Pose2d(5.95, 3.86, Rotation2d.fromDegrees(135.0));
  }

  public static final LeftStart instance = new LeftStart();
}
