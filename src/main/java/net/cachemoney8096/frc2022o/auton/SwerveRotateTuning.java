package net.cachemoney8096.frc2022o.auton;

import net.cachemoney8096.frc2022o.subsystems.drive.DriveSubsystem;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveRotateTuning extends AutonCommandBase {
  public SwerveRotateTuning(DriveSubsystem swerve) {
    // PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("CCW90", 0.10, 0.50);
    // // spotless:off
    // addCommands(
    //     new InstantCommand(() ->
    // swerve.setHeading(trajectory.getInitialPose().getRotation().getDegrees())),
    //     new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose()), swerve)
    //         .withName("Reset Odometry"),
    //     swerve.trajectoryFollowerCommand(
    //         trajectory,
    //         new PIDController(0.0, 0.0, 0.0),
    //         new PIDController(0.0, 0.0, 0.0),
    //         Constants.Drivetrain.kThetaController),
    //     new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve).withName("Stop After
    // Auto")
    // );
    // // spotless:on
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
