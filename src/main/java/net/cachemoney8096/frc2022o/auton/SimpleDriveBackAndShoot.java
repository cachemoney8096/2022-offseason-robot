package net.cachemoney8096.frc2022o.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.cachemoney8096.frc2022o.auton.locations.Location;
import net.cachemoney8096.frc2022o.commands.ShootCommand;
import net.cachemoney8096.frc2022o.subsystems.Climber;
import net.cachemoney8096.frc2022o.subsystems.Shooter;
import net.cachemoney8096.frc2022o.subsystems.Indexer;
import net.cachemoney8096.frc2022o.subsystems.Intake;
import net.cachemoney8096.frc2022o.subsystems.drive.DriveSubsystem;

public class SimpleDriveBackAndShoot extends AutonCommandBase {
  public SimpleDriveBackAndShoot(
      DriveSubsystem drivetrain,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Climber climber,
      Location startLocation) {

    super(startLocation.getName().replace("Start", "") + " Drive Back SIMPLE");

    double DRIVE_TIME_SECONDS = 1.7;
    double INTAKE_TIME_SECONDS = 3.0;
    double SHOOT_TIME_SECONDS = 4.0;
    double WAIT_FOR_BALL_TIME_SECONDS = 0.25;
    double RETRACT_INTAKE_TIME_SECONDS = 1.0;

    double DRIVE_SPEED_PERCENT = 0.15;

    /* Uses no trajectory, just super simple drive back for time, and shoot */
    // spotless:off
    addCommandsWithLog(
        "Simple Drive Back and Shoot" + startLocation.getName(),
        // House keeping commands for pose and gyro
        new InstantCommand(
                () -> drivetrain.setHeading(startLocation.get().getRotation().getDegrees()))
            .withName("Set Heading"),
        new InstantCommand(() -> drivetrain.resetOdometry(startLocation.get()))
            .withName("Reset Odometry"),

        // Drive back while intaking
        new RunCommand(intake::runAllIntakeForwardsOverride, intake)
            .withName("Deploy and Run Intake"),
        new RunCommand(shooter::shoot, shooter).withName("Spinning up shooter"),
        new RunCommand(() -> drivetrain.drive(DRIVE_SPEED_PERCENT, 0.0, 0.0, false), drivetrain)
            .withTimeout(DRIVE_TIME_SECONDS)
            .withName("Drive Back Time")
            .andThen(() -> drivetrain.stop())
            .withName("Drive x Speed for Y Time")
            .withTimeout(INTAKE_TIME_SECONDS),

        // Stop and bring the intake in before shooting to be more stable
        new InstantCommand(() -> drivetrain.stop()).withName("Stop Drive"),
        new WaitCommand(WAIT_FOR_BALL_TIME_SECONDS).withName("Wait for Ball"),
        new InstantCommand(intake::dontIntakeCargo, intake).withName("Retract Intake"),
        new WaitCommand(RETRACT_INTAKE_TIME_SECONDS).withName("Wait for intake"),
        new ShootCommand(drivetrain, indexer, shooter, 0.0, 0.0, 0.0, true)
            .withName("Trying to shoot")
            .withTimeout(SHOOT_TIME_SECONDS));
  }
}
