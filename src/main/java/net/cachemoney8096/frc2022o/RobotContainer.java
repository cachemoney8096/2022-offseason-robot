// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o;

import net.cachemoney8096.frc2022o.subsystems.Climber;
import net.cachemoney8096.frc2022o.subsystems.Intake;
import net.cachemoney8096.frc2022o.subsystems.Indexer;
import net.cachemoney8096.frc2022o.subsystems.Shooter;
import net.cachemoney8096.frc2022o.subsystems.drive.DriveSubsystem;
import net.cachemoney8096.frc2022o.commands.ShootCommand;
import net.cachemoney8096.frc2022o.libs.SendablePigeon;
import net.cachemoney8096.frc2022o.libs.XboxController;
import net.cachemoney8096.frc2022o.libs_3005.util.JoystickUtil;
import net.cachemoney8096.frc2022o.libs_3005.vendor.sensor.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import net.cachemoney8096.frc2022o.auton.AutonChooser;
import net.cachemoney8096.frc2022o.auton.SimpleDriveBackAndShoot;
import net.cachemoney8096.frc2022o.auton.locations.Location;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer INSTANCE;

  public final XboxController driverController;
  public final XboxController operatorController;

  private final SendablePigeon pigeon;
  private final PowerDistribution powerDistribution;
  public final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final DriveSubsystem drivetrain;
  private final Climber climber;
  private final Limelight limelight;

  private SendableChooser<Command> autonChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    INSTANCE = this;

    powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    powerDistribution.clearStickyFaults();

    driverController = new XboxController(RobotMap.DRIVER_CONTROLLER_INDEX);
    operatorController = new XboxController(RobotMap.OPERATOR_CONTROLLER_INDEX);

    limelight =
        new Limelight(
            Constants.LIMELIGHT_PITCH_DEGREES,
            Constants.LIMELIGHT_HEIGHT_METERS,
            Constants.TARGET_HEIGHT_METERS);
    pigeon = new SendablePigeon(RobotMap.PIGEON_IMU_ID);
    indexer = new Indexer();
    intake = new Intake(indexer);
    shooter = new Shooter(limelight);
    drivetrain = new DriveSubsystem(pigeon, limelight);
    climber = new Climber();
    

    Shuffleboard.getTab("Subsystems").add(drivetrain.getName(), drivetrain);
    Shuffleboard.getTab("Subsystems").add(intake.getName(), intake);
    Shuffleboard.getTab("Subsystems").add(indexer.getName(), indexer);
    Shuffleboard.getTab("Subsystems").add(shooter.getName(), shooter);
    Shuffleboard.getTab("Subsystems").add(climber.getName(), climber);

    configureButtonBindings();
    configureAuton();
  }

  /** Call for initialization at least a couple seconds after construction */
  public void initialize() {
    shooter.initialize();
    //autons
    new SimpleDriveBackAndShoot(drivetrain, shooter, indexer, intake, climber, Location.LeftStart());
    AutonChooser.setDefaultAuton(
        new SimpleDriveBackAndShoot(drivetrain, shooter, indexer, intake, climber, Location.LeftStart()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive controls
    drivetrain.setDefaultCommand(
        new RunCommand(
                () ->
                    drivetrain.drive(
                        MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                        MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                        JoystickUtil.squareAxis(
                            MathUtil.applyDeadband(-driverController.getRightX(), 0.1)),
                        driverController.getLeftBumper()), // default to robot-relative for now
                drivetrain)
            .withName("Manual Drive"));

    // Set up intake controls
    intake.setDefaultCommand(
        new RunCommand(intake::dontIntakeCargo, intake).withName("Not Intaking"));
    driverController
        .TriggerLeft()
        .whileActiveContinuous(
            new InstantCommand(intake::intakeCargo, intake).withName("Intaking"));

    // Set up shooter controls for the indexer and for the shooter
    indexer.setDefaultCommand(
        new RunCommand(indexer::indexBall, indexer).withName("Not Feeding Shooter"));
    final boolean SHOOTING_NOT_INTERRUPTIBLE = false;
    driverController
        .TriggerRight()
        .whileActiveContinuous(
            new ShootCommand(
                    drivetrain,
                    indexer,
                    shooter,
                    MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                    MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                    JoystickUtil.squareAxis(
                        MathUtil.applyDeadband(-driverController.getRightX(), 0.1)),
                    driverController.getLeftBumper())
                .withName("Trying to shoot"),
            SHOOTING_NOT_INTERRUPTIBLE);
    driverController
        .BumperRight()
        .whileActiveContinuous(
            new InstantCommand(shooter::shoot, shooter).withName("Spinning up shooter"));
    // driverController
    //     .TriggerRight()
    //     .whenInactive(
    //         new InstantCommand(shooter::dontShoot, shooter).withName("Stopping shooter (trigger)"));
    driverController
        .BumperRight()
        .whenInactive(
            new InstantCommand(shooter::dontShoot, shooter).withName("Stopping shooter (bumper)"));
    driverController
        .A()
        .whenPressed(
            new InstantCommand(shooter::setHoodFromLimelight, shooter).withName("Setting hood from limelight"));

    // Set up climber controls
    operatorController
        .A()
        .whileHeld(
            new InstantCommand(climber::rightMotorDown, climber).withName("Right Climber Down"));
    operatorController
        .B()
        .whileHeld(new InstantCommand(climber::rightMotorUp, climber).withName("Right Climber Up"));
    operatorController
        .X()
        .whileHeld(
            new InstantCommand(climber::leftMotorDown, climber).withName("Left Climber Down"));
    operatorController
        .Y()
        .whileHeld(new InstantCommand(climber::leftMotorUp, climber).withName("Left Climber Up"));

    operatorController
        .BumperLeft()
        .whileHeld(
            new InstantCommand(intake::runAllIntakeBackwardsOverride, intake)
                .withName("Run All Intake Backwards"));

    operatorController
        .BumperLeft()
        .whileHeld(
            new InstantCommand(indexer::runAllIndexerBackwardsOverride, indexer)
                .withName("Run All Indexer Backwards"));

    operatorController
        .BumperRight()
        .whileHeld(
            new InstantCommand(intake::runAllIntakeForwardsOverride, intake)
                .withName("Run All Intake Forwards"));

    operatorController
        .BumperRight()
        .whileHeld(
            new InstantCommand(indexer::runOneAndTwoIndexerForwardsOverride, indexer)
                .withName("Run One and Two Indexer Forwards"));

    operatorController
        .TriggerRight()
        .whileActiveContinuous(
            new InstantCommand(indexer::runAllIndexerForwardsOverride, indexer)
                .withName("Run All Indexer Forwards"));

    operatorController
        .TriggerLeft()
        .whenActive(new InstantCommand(intake::extendIntake, intake).withName("Extending Intake"));

    operatorController
        .TriggerLeft()
        .whenInactive(
            new InstantCommand(intake::retractIntake, intake).withName("Retracting Intake"));
  }

  private void configureAuton() {
    autonChooser = new SendableChooser<>();
    // autonChooser.setDefaultOption("Do Nothing", new InstantCommand(() ->
    // System.out.println("Doing nothing...")));
    // autonChooser.addOption("3 Ball 1678", Trajectory.threeBall1678(launcher,
    // hopper,
    // cargoManager, drivetrain));
    // autonChooser.setDefaultOption("5 Ball 1678",
    // Trajectory.fiveBall1678(launcher, hopper,
    // cargoManager, drivetrain));

  }

  public static synchronized RobotContainer getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotContainer();
    }
    return INSTANCE;
  }

  public SendableChooser<Command> getAutonChooser() {
    return autonChooser;
  }
}
