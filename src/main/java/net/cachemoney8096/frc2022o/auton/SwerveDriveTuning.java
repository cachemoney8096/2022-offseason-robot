package net.cachemoney8096.frc2022o.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.cachemoney8096.frc2022o.auton.locations.Location;
import net.cachemoney8096.frc2022o.commands.ShootCommand;
import net.cachemoney8096.frc2022o.subsystems.Climber;
import net.cachemoney8096.frc2022o.subsystems.Shooter;
import net.cachemoney8096.frc2022o.subsystems.Indexer;
import net.cachemoney8096.frc2022o.subsystems.Intake;
import net.cachemoney8096.frc2022o.subsystems.drive.DriveSubsystem;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveDriveTuning extends AutonCommandBase {
   public SwerveDriveTuning(DriveSubsystem swerve) {
  //   PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("Drive2M", 2.0, 6.0);
  //   // spotless:off
  //   addCommands(
  //       new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose()), swerve)
  //           .withName("Reset Odometry"),
  //       swerve.trajectoryFollowerCommand(
  //           trajectory,
  //           Constants.Drivetrain.kXController,
  //           Constants.Drivetrain.kYController,
  //           new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0))),
  //       new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve)
  //   );
  //   // spotless:on
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
