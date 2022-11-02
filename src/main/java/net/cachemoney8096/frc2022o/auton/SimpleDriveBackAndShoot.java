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

public class SimpleDriveBackAndShoot extends AutonCommandBase {
  public SimpleDriveBackAndShoot(
      DriveSubsystem drivetrain,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Climber climber,
      Location startLocation) {

    super(startLocation.getName().replace("Start", "") + "Drive Back SIMPLE");

    double kDriveTime = 1.7;
    double kIntakeTimeout = 3.0;

    // Percent
    double kDriveSpeed = 0.15;

    /* Uses no trajectory, just super simple drive back for time, and shoot */
    // spotless:off
    addCommandsWithLog("Simple Drive Back and Shoot" + startLocation.getName(),
        // House keeping commands for pose and gyro
        new InstantCommand(() -> drivetrain.setHeading(startLocation.get().getRotation().getDegrees())).withName("Set Heading"),
        new InstantCommand(() -> drivetrain.resetOdometry(startLocation.get())).withName("Reset Odometry"),

        // Enable tracking, turn on shooter and hood
        //visionCommands.enableTracking(),
        //hood.enableCommand(),

        // Drive back while intaking
        
        new InstantCommand(intake::runAllIntakeForwardsOverride, intake)
          .withName("Deploy and Run Intake"),
        new InstantCommand(shooter::shoot, shooter)
          .withName("Spinning up shooter"),

        new RunCommand(() -> drivetrain.drive(kDriveSpeed, 0.0, 0.0, false), drivetrain)
              .withTimeout(kDriveTime)
              .withName("Drive Back Time")
              .andThen(() -> drivetrain.stop()).withName("Drive x Speed for Y Time").withTimeout(kIntakeTimeout),


        //new InstantCommand(() -> Logger.tag("Simple Drive Back and Shoot").info("Finished drive back"))
      //   visionCommands.waitOnTarget(2.0),

        // Stop and bring the intake in before shooting to be more stable
        new InstantCommand(() -> drivetrain.stop()).withName("Stop Drive"),
        new InstantCommand(intake::dontIntakeCargo, intake)
          .withName("Retract Intake"),
        new WaitCommand(1.0).withName("Wait for intake"),
        new ShootCommand(
          drivetrain,
          indexer,
          shooter,
          0.0,
          0.0,
          0.0,
          true)
          .withName("Trying to shoot").withTimeout(4.0)

        
      //   // Disable tracking and reset hood and turret
      //   visionCommands.disableTracking(),
      //   turret.setDegreesCommand(0.0),
      //   hood.disableCommand()
    );
    // spotless:on
  }
}
