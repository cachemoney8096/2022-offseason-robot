// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o.config;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.cachemoney8096.frc2022o.commands.*;
import net.cachemoney8096.frc2022o.subsystems.CargoManager;
import net.cachemoney8096.frc2022o.subsystems.Drivetrain;
import net.cachemoney8096.frc2022o.subsystems.Hopper;
import net.cachemoney8096.frc2022o.subsystems.Launcher;

import java.nio.file.Path;

/** Add your docs here. */
public class Trajectory {

    private static final PathPlannerTrajectory threeBallTrajectory = PathPlanner.loadPath("2022_01_p1", 2.0, 6.0);
    private static final PathPlannerTrajectory hpTrajectory = PathPlanner.loadPath("2022_01_p2", 2.5, 6.0);
    private static final PathPlannerTrajectory hp2Traectory = PathPlanner.loadPath("2022_01_p3", 2.5, 6.0);
    private static final PathPlannerTrajectory threeBall1678Trajectory = PathPlanner.loadPath("1678_ThreeBall", 2.0, 3.0);
    private static final PathPlannerTrajectory fiveBallHP1678Trajectory = PathPlanner.loadPath("1678_FiveBallHP", 2.5, 3.0);
    private static final PathPlannerTrajectory fiveBallReturn1678Trajectory = PathPlanner.loadPath("1678_FiveBallReturn", 2.5, 3.0);
    private static final PathPlannerTrajectory twoBallLeftTarmac = PathPlanner.loadPath("2Ball_LeftTarmac", 2.0, 3.0);
    private static final PathPlannerTrajectory backUpTwoMeters = PathPlanner.loadPath("BackUp2m", 2.0, 3.0);
    private static final PathPlannerTrajectory twoBallThief = PathPlanner.loadPath("2Ball_Thief", 2.0, 3.0);
    private static final PathPlannerTrajectory twoPlusTwoBallOne = PathPlanner.loadPath("2Plus2Ball_1", 2.0, 3.0);
    private static final PathPlannerTrajectory twoPlusTwoBallTwo = PathPlanner.loadPath("2Plus2Ball_2", 2.5, 5.0);
    private static final PathPlannerTrajectory twoPlusOneBallTwo = PathPlanner.loadPath("2Plus1Ball_2", 2.5, 5.0);
    private static final PathPlannerTrajectory twoPlusOneBallThree = PathPlanner.loadPath("2Plus1Ball_3", 2.5, 5.0);
    private static final PathPlannerTrajectory fiveBallReturnLLTrajectory = PathPlanner.loadPath("5BallReturn_LLHandoff", 3.0, 5.0);

    public static Command threeBall1678(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain) {

        final PathPlannerCommand threeBall1678 = new PathPlannerCommand(threeBall1678Trajectory, drivetrain, true);

        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0, true)),
                new ShootAtDistanceCommand(launcher, 1.57),
                new ConfigShotSpacingCommand(hopper, 0.1),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(0.75),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                threeBall1678.alongWith(
                        new SequentialCommandGroup(
                                new ShootAtDistanceCommand(launcher, 2.1),
                                new WaitCommand(3.0),
                                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE))),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP),
                new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(3.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE)
        );
    }

    public static Command fiveBall1678(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        final PathPlannerCommand threeBall1678 = new PathPlannerCommand(threeBall1678Trajectory, drivetrain, true);
        final PathPlannerCommand fiveBallHP1678 = new PathPlannerCommand(fiveBallHP1678Trajectory, drivetrain, false);
        final PathPlannerCommand fiveBallReturn1678 = new PathPlannerCommand(fiveBallReturn1678Trajectory, drivetrain, false);

        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0, true)),
                new ShootAtDistanceCommand(launcher, 1.57),
                new ConfigShotSpacingCommand(hopper, 0.1),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(0.75),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                threeBall1678.alongWith(new SequentialCommandGroup(
                        new ShootAtDistanceCommand(launcher, 2.1),
                        new WaitCommand(3.0),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE))),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP),
                new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(1.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                fiveBallHP1678,
                new WaitCommand(0.1),
                fiveBallReturn1678.alongWith(new SequentialCommandGroup(
                        new ShootAtDistanceCommand(launcher, 2.8),
                        //new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.IDLE)),
                        //new WaitCommand(0.25),
                        //new InstantCommand(() -> cargoManager.setWantedState(CargoManager.WantedState.INDEX)),
                        new WaitCommand(1.0),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP))),
                //new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(5.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE)
        );
    }

    public static Command threeBallFender(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        final PathPlannerCommand threeBallCommand = new PathPlannerCommand(threeBallTrajectory, drivetrain, true);

        return new SequentialCommandGroup(
                new InstantCommand(launcher::setFenderShot),
                new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(1.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                threeBallCommand.alongWith(
                                new WaitCommand(3.5).andThen(
                                    new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE))),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP),
                new WaitCommand(0.1),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(2.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE)
        );
    }

    public static Command fiveBallFender(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        final PathPlannerCommand threeBallCommand = new PathPlannerCommand(threeBallTrajectory, drivetrain, true);
        final PathPlannerCommand toHpCommand = new PathPlannerCommand(hpTrajectory, drivetrain, false);
        final PathPlannerCommand fromHpCommand = new PathPlannerCommand(hp2Traectory, drivetrain, false);

        return new SequentialCommandGroup(
                new InstantCommand(launcher::setFenderShot),
                new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(1.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                threeBallCommand.alongWith(
                                new WaitCommand(3.5).andThen(
                                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE))),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP),
                new WaitCommand(0.1),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(2.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE),
                toHpCommand.alongWith(new SequentialCommandGroup(
                        new WaitCommand(0.25),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX))),
                new WaitCommand(1.0),
                fromHpCommand.alongWith(new WaitCommand(0.25).andThen(
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE))),
                new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP),
                new WaitCommand(0.1),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(2.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE)
        );
    }

    public static Command twoBallLeftTarmac(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        final PathPlannerCommand twoBallCommand = new PathPlannerCommand(twoBallLeftTarmac, drivetrain, true);

        return new SequentialCommandGroup(
                new ShootAtDistanceCommand(launcher, 2.0),
                new ConfigShotSpacingCommand(hopper, 0.5),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                twoBallCommand.alongWith(new SequentialCommandGroup(new WaitCommand(2.0),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX))),
                new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP),
                new WaitCommand(0.1),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(2.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE)
        );
    }

    public static Command shootAndBackUp(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain) {
        final PathPlannerCommand backUpTwoMetersCommand = new PathPlannerCommand(backUpTwoMeters, drivetrain, true);

        return new SequentialCommandGroup(
                new ShootAtDistanceCommand(launcher, 1.875),
                new ConfigShotSpacingCommand(hopper, 0.5),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(0.65),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE),
                backUpTwoMetersCommand
        );
    }

    public static Command StationaryLLTest(CargoManager cargoManager, Launcher launcher, Drivetrain drivetrain, Hopper hopper){
        return new SequentialCommandGroup(
                new StationaryShotWithLimelightCommand(cargoManager, launcher, drivetrain, hopper, 2),
                new WaitCommand(2.0),
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
            new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL))
        );
    }

    public static Command twoBallLeftTarmacWithLimelight(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        final PathPlannerCommand twoBallCommand = new PathPlannerCommand(twoBallLeftTarmac, drivetrain, true);

        return new SequentialCommandGroup(
                new ConfigShotSpacingCommand(hopper, 0.5),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                twoBallCommand.alongWith(new SequentialCommandGroup(new WaitCommand(2.0),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX))),
                new WaitCommand(0.25),
                new StationaryShotWithLimelightCommand(cargoManager, launcher, drivetrain, hopper, 2),
                new WaitCommand(5.0),
                new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL))
        );
    }
/*
    public static Command twoPlusTowBallOld(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        final PathPlannerCommand ourTwoBallCommand = new PathPlannerCommand(twoBallLeftTarmac, drivetrain, true);
        final PathPlannerCommand theirTwoBallCommand = new PathPlannerCommand(twoBallThief, drivetrain, false);

        return new SequentialCommandGroup(
                new ConfigShotSpacingCommand(hopper, 0.5),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                ourTwoBallCommand.alongWith(new SequentialCommandGroup(new WaitCommand(2.0),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX))),
                new WaitCommand(0.25),
                new StationaryShotWithLimelightCommand(cargoManager, launcher, drivetrain, hopper, 2),
                new WaitCommand(2.0),
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.INDEX),
                theirTwoBallCommand,
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                new InstantCommand(() -> launcher.setFenderShotLow()),
                new WaitCommand(0.5),
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.SHOOT),
                new WaitCommand(2.0),
                new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL))
        );*/

    public static Command twoPlusTwoBall(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain) {
        final PathPlannerCommand twoPlusTwoCommandOne = new PathPlannerCommand(twoPlusTwoBallOne, drivetrain, true);
        final PathPlannerCommand twoPlusTwoCommandTwo = new PathPlannerCommand(twoPlusTwoBallTwo, drivetrain, false);

        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0, true)),
                new ShootAtDistanceCommand(launcher, 1.5),
                new ConfigShotSpacingCommand(hopper, 0.0),
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.SHOOT),
                new WaitCommand(0.75),
                twoPlusTwoCommandOne.alongWith(new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.INDEX)),
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                new WaitCommand(0.25),
                new ConfigShotSpacingCommand(hopper, 3.0),
                new StationaryShotWithLimelightCommand(cargoManager, launcher, drivetrain, hopper, 1),
                new WaitCommand(0.7),
                //new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                //new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.INDEX),
                new InstantCommand(launcher::setOppositeColorFarEject),
                new ConfigShotSpacingCommand(hopper, 0.0),
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.INDEX),
                twoPlusTwoCommandTwo.alongWith(new SequentialCommandGroup(
                        new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.INDEX),
                        new WaitCommand(2.0),
                        new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                        new WaitCommand(1.5),
                        new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.SHOOT),
                        new WaitCommand(2.0))),
                new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL))
        );
    }

        public static Command twoPlusOneBall(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
            final PathPlannerCommand twoPlusOneCommandOne = new PathPlannerCommand(twoPlusTwoBallOne, drivetrain, true);
            final PathPlannerCommand twoPlusOneCommandTwo = new PathPlannerCommand(twoPlusOneBallTwo, drivetrain, false);
            final PathPlannerCommand twoPlusOneCommandThree = new PathPlannerCommand(twoPlusOneBallThree, drivetrain, false);

            return new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0, true)),
                    new ShootAtDistanceCommand(launcher, 1.5),
                    new ConfigShotSpacingCommand(hopper, 0.0),
                    new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                    new WaitCommand(0.75),
                    twoPlusOneCommandOne.alongWith(new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX)),
                    new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                    new WaitCommand(0.25),
                    new ConfigShotSpacingCommand(hopper, 3.0),
                    new StationaryShotWithLimelightCommand(cargoManager, launcher, drivetrain, hopper, 1),
                    new WaitCommand(0.7),
                    //new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                    //new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.INDEX),
                    new InstantCommand(launcher::setOppositeColorFarEject),
                    new ConfigShotSpacingCommand(hopper, 0.0),
                    twoPlusOneCommandTwo,
                    new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.SHOOT),
                    new WaitCommand(1.5),
                    new CargoManagerWantedStateCommand(cargoManager, CargoManager.WantedState.IDLE),
                    twoPlusOneCommandThree,
                    new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.MANUAL_CONTROL))
            );
    }

    public static Command moveThenShootLL(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        return new SequentialCommandGroup(
                new MoveThenShootWithLimelightCommand(cargoManager, launcher, drivetrain, hopper, 1, 2.0, 0.0, 2.0)
        );
    }

    public static Command fiveBallWithLimelight(Launcher launcher, Hopper hopper, CargoManager cargoManager, Drivetrain drivetrain){
        final PathPlannerCommand threeBall1678 = new PathPlannerCommand(threeBall1678Trajectory, drivetrain, true);
        final PathPlannerCommand fiveBallHP1678 = new PathPlannerCommand(fiveBallHP1678Trajectory, drivetrain, false);
        final PathPlannerCommandDistanceEnd fiveBallReturnToLL = new PathPlannerCommandDistanceEnd(fiveBallReturnLLTrajectory, drivetrain, false, 4.0);

        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0, true)),
                new ShootAtDistanceCommand(launcher, 1.57),
                new ConfigShotSpacingCommand(hopper, 0.1),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(0.75),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                threeBall1678.alongWith(new SequentialCommandGroup(
                        new ShootAtDistanceCommand(launcher, 2.3),
                        new WaitCommand(3.0),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE))),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP),
                new WaitCommand(0.25),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SHOOT),
                new WaitCommand(1.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.INDEX),
                fiveBallHP1678,
                fiveBallReturnToLL.alongWith(new SequentialCommandGroup(
                        new WaitCommand(0.1),
                        new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.SPINUP))),
                new MoveThenShootWithLimelightCommand(cargoManager, launcher, drivetrain, hopper, 4, 2.5, 0.0, 1.0),
                new WaitCommand(5.0),
                new CargoManagerWantedStateCommand(cargoManager,CargoManager.WantedState.IDLE)
        );
    }
}
