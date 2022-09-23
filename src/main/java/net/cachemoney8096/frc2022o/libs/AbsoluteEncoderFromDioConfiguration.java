package net.cachemoney8096.frc2022o.libs;


// Basically a copy of CanCoderAbsoluteConfiguration from:
// src\main\java\com\swervedrivespecialties\swervelib\ctre\CanCoderAbsoluteConfiguration.java
public class AbsoluteEncoderFromDioConfiguration {
  private final int id;
  private final double offset;

  public AbsoluteEncoderFromDioConfiguration(int id, double offset) {
    this.id = id;
    this.offset = offset;
  }

  public int getId() {
    return id;
  }

  public double getOffset() {
    return offset;
  }
}
