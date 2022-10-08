// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o;

import net.cachemoney8096.frc2022o.subsystems.Climber;
import net.cachemoney8096.frc2022o.subsystems.Drivetrain;
import net.cachemoney8096.frc2022o.subsystems.Intake;
import net.cachemoney8096.frc2022o.subsystems.Indexer;
import net.cachemoney8096.frc2022o.subsystems.Shooter;
import net.cachemoney8096.frc2022o.libs.XboxController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;

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

  private final PowerDistribution powerDistribution;
  public final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Drivetrain drivetrain;
  private final Climber climber;

  private SendableChooser<Command> autonChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    INSTANCE = this;

    powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    powerDistribution.clearStickyFaults();

    driverController = new XboxController(RobotMap.DRIVER_CONTROLLER_INDEX);
    operatorController = new XboxController(RobotMap.OPERATOR_CONTROLLER_INDEX);

    indexer = new Indexer();
    intake = new Intake(indexer);
    shooter = new Shooter();
    drivetrain = new Drivetrain();
    climber = new Climber();

    configureButtonBindings();
    configureAuton();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    operatorController.B().whileHeld(new InstantCommand(climber::rightMotorDown, climber));
    operatorController.Y().whileHeld(new InstantCommand(climber::rightMotorUp, climber));
    operatorController.A().whileHeld(new InstantCommand(climber::leftMotorDown, climber));
    operatorController.X().whileHeld(new InstantCommand(climber::leftMotorUp, climber));
  }

  private void configureAuton() {
    autonChooser = new SendableChooser<>();
    // autonChooser.setDefaultOption("Do Nothing", new InstantCommand(() ->
    // System.out.println("Doing nothing...")));
    // autonChooser.addOption("3 Ball 1678", Trajectory.threeBall1678(launcher, hopper,
    // cargoManager, drivetrain));
    // autonChooser.setDefaultOption("5 Ball 1678", Trajectory.fiveBall1678(launcher, hopper,
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
