package net.cachemoney8096.frc2022o;

// Use this class to store numbers that are arbitrary but can be tuned (such as thresholds, etc.)
public class Calibrations {
  // For translation commands above this threshold (in [0,1]), heading lock will not apply
  public static final double HEADING_LOCK_TRANSLATION_COMMAND_THRESHOLD = 0.1;
  // For rotation commands below this threshold (in [0,1]), heading lock may apply
  public static final double HEADING_LOCK_ROTATION_COMMAND_THRESHOLD = 0.5;
}