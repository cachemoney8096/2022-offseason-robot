// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import net.cachemoney8096.frc2022o.commands.CargoManagerWantedStateCommand;
import net.cachemoney8096.frc2022o.commands.PathPlannerCommand;
import net.cachemoney8096.frc2022o.config.Trajectory;
import net.cachemoney8096.frc2022o.loops.SubsystemManager;
import net.cachemoney8096.frc2022o.subsystems.*;
import net.cachemoney8096.frc2022o.util.XboxController;

import java.nio.file.Path;


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

	public final PowerDistribution powerDistribution;
	public final PneumaticHub pneumaticsHub;

	private final SubsystemManager manager;

    private final Drivetrain drivetrain;
	public final Launcher launcher;
	private final Hopper hopper;
	private final CargoManager cargoManager;
	public final LED led;
	private final Limelight limelight;
    private final Intake intake;
	private final Climber climber;
	private final ColorSensor colorSensor;
	private SendableChooser<Command> autonChooser;

	private final Compressor compressor;

	public enum EnableState {
		DISABLED,
		AUTON,
		TELEOP
	}

	public EnableState enableState = EnableState.DISABLED;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		INSTANCE = this;
		driverController = new XboxController(0);
		operatorController = new XboxController(1);
		// Configure the button bindings
		LiveWindow.disableAllTelemetry();
		LiveWindow.setEnabled(false);

		compressor = new Compressor(Constants.PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH);
		compressor.enableAnalog(80, 120);

		manager = new SubsystemManager(0.02);
        Constants.setConstants();

		powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
		pneumaticsHub = new PneumaticHub(Constants.PNEUMATIC_HUB_CAN_ID);
		powerDistribution.clearStickyFaults();
		pneumaticsHub.clearStickyFaults();

		hopper = new Hopper(compressor);
		led = new LED();
		limelight = new Limelight();
		launcher = new Launcher(limelight, hopper);
        intake = new Intake(hopper);
		climber = new Climber();
		colorSensor = new ColorSensor();
		cargoManager = new CargoManager(hopper, launcher, intake, colorSensor);
		drivetrain = new Drivetrain(cargoManager, driverController, limelight, launcher);

		manager.setSubsystems(drivetrain, climber, hopper, launcher, cargoManager, intake, limelight, led, colorSensor);

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
		operatorController.TriggerLeft()
				.whenActive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.INDEX)))
				.whenInactive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.IDLE)));

		/*
		driverController.BumperLeft()
				.whenActive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.INDEX)))
				.whenInactive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.IDLE)));
		*/


		driverController.X().whenActive(new InstantCommand(drivetrain::resetOdometry));

		operatorController.TriggerRight()
				.whenActive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.SHOOT)))
				.whenInactive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.IDLE)));
		/*
		driverController.A()
				.whenActive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.SHOOT)))
				.whenInactive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.IDLE)));
		*/
		operatorController.StickLeft()
				.whenPressed(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.EXHAUST)))
				.whenReleased(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.IDLE)));

		driverController.TriggerRight()
				.whenActive(new ParallelCommandGroup(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.GOAL_TRACKING)),
						new InstantCommand(() -> hopper.setShotSpacingTime(0.1))))
				.whileActiveContinuous(new InstantCommand(launcher::setLimelightShot))
				.whenInactive(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL)));

		driverController.BumperRight()
				.whenActive(new ParallelCommandGroup(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MOVE_AND_SHOOT)),
						new InstantCommand(() -> hopper.setShotSpacingTime(0.1))))
				.whileActiveContinuous(new InstantCommand(launcher::setMovingLimelightShot))
				.whenInactive(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL)));

		operatorController.Y().whenPressed(
				new InstantCommand(launcher::setFenderShot)
						.alongWith(new InstantCommand(() -> hopper.setShotSpacingTime(0.5))));

		operatorController.X().whenPressed(new ParallelCommandGroup(new InstantCommand(launcher::setFenderShotLow),
				new InstantCommand(() -> hopper.setShotSpacingTime(0.0))));

		operatorController.BumperRight().whenPressed(
				new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.SPINUP)));

		operatorController.North().whenPressed(new InstantCommand(climber::advanceStateManual));
		operatorController.West().whenPressed(new InstantCommand(() -> climber.setWantedState(Climber.WantedState.DEPLOY)));
		operatorController.East().whenPressed(new InstantCommand(() -> climber.setWantedState(Climber.WantedState.LEVEL_3)));
		operatorController.South().whenPressed(new InstantCommand(() -> climber.setWantedState(Climber.WantedState.LEVEL_4)));

		operatorController.A().whenPressed(new ParallelCommandGroup(new InstantCommand(launcher::setTarmacLineShot),
			new InstantCommand(() -> hopper.setShotSpacingTime(0.0))));

		operatorController.Start().whenPressed(new InstantCommand(() -> compressor.enableAnalog(119, 120)))
				.whenReleased(new InstantCommand(() -> compressor.enableAnalog(80, 120)));

		operatorController.Back().whenPressed(new InstantCommand(()-> launcher.zeroHood()));

		operatorController.B()
		.whenActive(new InstantCommand(() -> launcher.setManualOverride(true)))
		.whenInactive(new InstantCommand(() -> launcher.setManualOverride(false)));

//		operatorController.B().whenActive(new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.OPPOSITE_COLOR))
//				.whenInactive(new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE));


		Trigger firstBall = new Trigger(hopper::hasFirstBall);
		Trigger secondBall = new Trigger(hopper::hasSecondBall);
		Trigger llTracking = new Trigger(drivetrain::isGoalTracking);
		Trigger llLocked = new Trigger(drivetrain::isGoalTracked);
		Trigger spinUp = new Trigger(launcher::isSpinningUp);
		Trigger shot = new Trigger(launcher::readyToLaunch);
		Trigger ballShot = new Trigger(hopper::hasShot);
		Trigger lowPressure = new Trigger(() -> compressor.getPressure() < 60);
		Trigger oppositeColorBall = new Trigger(() -> colorSensor.determineIndexedBallColorState().equals(ColorSensor.indexedBallState.OPPOSITE_COLOR));

		/* Controller Vibration Commands */
		//lowPressure.whenActive(new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, .5)));
		//lowPressure.whenInactive(new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)));
		oppositeColorBall.whenActive(new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, .5)));
		oppositeColorBall.whenInactive(new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0)));
		// Driver
		/*
		secondBall.whenActive(new SequentialCommandGroup(
				new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.5)),
				new WaitCommand(0.5),
				new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0))));

		ballShot.whenActive(new SequentialCommandGroup(
				new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 1.0)),
				new WaitCommand(0.1),
				new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0))));
*/
		// Operator
		/*
		shot.whileActiveContinuous(new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 0.5)))
				.whenInactive(new InstantCommand(() -> operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0)));*/

		Command command = new InstantCommand(() -> {
			led.setWantedState(LED.WantedState.ONE_BALL);
			SmartDashboard.putNumber("hopper/cargoCount", 1);
		});

		firstBall.and(secondBall.negate()).and(shot.negate()).whenActive(new InstantCommand(() -> {
			led.setWantedState(LED.WantedState.ONE_BALL);
			SmartDashboard.putNumber("hopper/cargoCount", 1);
		})).whenInactive(new InstantCommand(() -> led.setWantedState(LED.WantedState.IDLE)));

		firstBall.and(secondBall).whenActive(new InstantCommand(() -> {
			led.setWantedState(LED.WantedState.TWO_BALL);
			SmartDashboard.putNumber("hopper/cargoCount", 2);
		})).whenInactive(new InstantCommand(() -> led.setWantedState(LED.WantedState.IDLE)));

		firstBall.negate().and(secondBall.negate()).whenActive(new InstantCommand(() -> {
			SmartDashboard.putNumber("hopper/cargoCount", 0);
		}));


		
		llTracking.and(llLocked.negate()).whileActiveContinuous(()-> led.setWantedState(LED.WantedState.LL_LOCKING)).whenInactive(new InstantCommand(() -> led.setWantedState(LED.WantedState.IDLE)));
		llLocked.and(spinUp.negate()).whileActiveContinuous(()-> led.setWantedState(LED.WantedState.LL_LOCKED)).whenInactive(new InstantCommand(() -> led.setWantedState(LED.WantedState.IDLE)));
		llLocked.and(spinUp).whileActiveContinuous(() -> led.setWantedState(LED.WantedState.SPEED_UP)).whenInactive(new InstantCommand(() -> led.setWantedState(LED.WantedState.IDLE)));
		shot.whileActiveContinuous(new InstantCommand(() -> led.setWantedState(LED.WantedState.READY_TO_FIRE))).whenInactive(new InstantCommand(() -> led.setWantedState(LED.WantedState.IDLE)));

		//firstBall.and(secondBall).whenActive(new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.SPINUP)));

	}

	private void configureAuton() {
		autonChooser = new SendableChooser<>();
		//autonChooser.setDefaultOption("Do Nothing", new InstantCommand(() -> System.out.println("Doing nothing...")));
		autonChooser.addOption("3 Ball 1678", Trajectory.threeBall1678(launcher, hopper, cargoManager, drivetrain));
		autonChooser.setDefaultOption("5 Ball 1678", Trajectory.fiveBall1678(launcher, hopper, cargoManager, drivetrain));
		autonChooser.addOption("1 Ball and Back Up", Trajectory.shootAndBackUp(launcher, hopper, cargoManager, drivetrain));
		//autonChooser.addOption("StationaryLLTest", Trajectory.StationaryLLTest(cargoManager, launcher, drivetrain, hopper));
		autonChooser.addOption("2 Ball Left Tarmac With Limelight", Trajectory.twoBallLeftTarmacWithLimelight(launcher, hopper, cargoManager, drivetrain));
		autonChooser.addOption("2 + 2 Ball", Trajectory.twoPlusTwoBall(launcher, hopper, cargoManager, drivetrain));
		autonChooser.addOption("Drive Then Shoot w/ LL", Trajectory.moveThenShootLL(launcher, hopper, cargoManager, drivetrain));
		autonChooser.addOption("2 + 1 Ball", Trajectory.twoPlusOneBall(launcher, hopper, cargoManager, drivetrain));
		autonChooser.addOption("5 Ball, LL 4/5", Trajectory.fiveBallWithLimelight(launcher, hopper, cargoManager, drivetrain));
		SmartDashboard.putData("auton/chooser",autonChooser);

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

	public void startSubsystemThreads(){
		manager.start();
	}

	public void stopSubsystemThreads(){
		manager.stop();
	}

	public void checkSubsystems() {
		manager.checkSubsystems();
	}

	public void stopSubsystems() {
		manager.stopSubsystems();
	}

}
