package net.cachemoney8096.frc2022o.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tinylog.Logger;

public abstract class AutonCommandBase extends SequentialCommandGroup {
  private double m_autonStartTime = 0;
  // TODO: The below will be moved to auton/locations
  public enum StartPosition {
    Left(0),
    Center(1),
    Right(2),
    OutOfEveryonesWay(3);

    public final int value;

    private final Pose2d[] poses = {
      new Pose2d(6.21, 5.21, Rotation2d.fromDegrees(135.44)),
      new Pose2d(6.70, 2.53, Rotation2d.fromDegrees(-133.63)),
      new Pose2d(7.68, 1.87, Rotation2d.fromDegrees(-88.99)),
      new Pose2d(5.95, 3.86, Rotation2d.fromDegrees(-178.79))
    };

    StartPosition(int v) {
      value = v;
    }

    public Pose2d getPose() {
      return poses[value];
    }
  }

  public AutonCommandBase() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);

    AutonChooser.addAuton(this, name);
  }

  public AutonCommandBase(String name) {
    AutonChooser.addAuton(this, name);
  }

  public void addCommandsWithLog(String tag, Command... commands) {
    // Add logging to each command

    // Log start of group
    super.addCommands(
        new InstantCommand(
            () -> {
              m_autonStartTime = Timer.getFPGATimestamp();
              Logger.tag(tag).trace("Auton goup Starting at {}", m_autonStartTime);
            }));

    // Log start and end of each command
    for (var cmd : commands) {
      super.addCommands(
          new InstantCommand(
              () ->
                  Logger.tag(tag)
                      .trace(
                          "Starting command step {}, at {}",
                          cmd.getName(),
                          Timer.getFPGATimestamp())),
          cmd,
          new InstantCommand(
              () ->
                  Logger.tag(tag)
                      .trace(
                          "Ending command step {}, at {}",
                          cmd.getName(),
                          Timer.getFPGATimestamp())));
    }

    // Log end of auton
    super.addCommands(
        new InstantCommand(
            () -> {
              Logger.tag(tag)
                  .trace(
                      "Auton group complete after {} seconds",
                      Timer.getFPGATimestamp() - m_autonStartTime);
            }));
  }
}
