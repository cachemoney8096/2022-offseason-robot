// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o;

import net.cachemoney8096.frc2022o.subsystems.Indexer;
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

  private final PowerDistribution powerDistribution;
  private final Indexer indexer;

  private SendableChooser<Command> autonChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    INSTANCE = this;

    powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    powerDistribution.clearStickyFaults();

    indexer = new Indexer();

    configureButtonBindings();
    configureAuton();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

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
