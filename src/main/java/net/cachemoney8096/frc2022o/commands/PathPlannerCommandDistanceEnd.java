// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.cachemoney8096.frc2022o.subsystems.Drivetrain;

//Borrowing code from Team 2337's 2020 code: TrajectoryCommand.java

public class PathPlannerCommandDistanceEnd extends PPSwerveControllerCommand {
  /** Creates a new PathPlannerSwerveTrajectoryCommand. */
  private PathPlannerTrajectory trajectory;
  private Drivetrain drivetrain;
  private final boolean resetOdometry;
  private final double endDistance;

    //TODO: relocate to constants file
    private static final double translateKp = 5.5;
    private static final double translateKi = 0;
    private static final double translateKd = 0;
    private static final double rotateKp = 4;
    private static final double rotateKi = 0;
    private static final double rotateKd = 0;

  private static ProfiledPIDController thetaController =  new ProfiledPIDController(rotateKp, rotateKi, rotateKd, new TrapezoidProfile.Constraints(
    Units.degreesToRadians(3600), //max angular velocity
    Units.degreesToRadians(10800) //max angular acceleration
  ));

  public PathPlannerCommandDistanceEnd(PathPlannerTrajectory trajectory, Drivetrain drivetrain){
      this(trajectory, drivetrain, true, 1.0);
  }

  public PathPlannerCommandDistanceEnd(PathPlannerTrajectory trajectory, Drivetrain drivetrain, boolean resetOdometry, double endDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      trajectory,
      drivetrain::getPose,
      drivetrain.getKinematics(),
      new PIDController(translateKp, translateKi, translateKd), //X direction controller
      new PIDController(translateKp, translateKi, translateKd), //Y direction controller
      //body angle controller
      thetaController,
      drivetrain::setModuleStatesFromTrajectory, 
      drivetrain
      );

      this.trajectory = trajectory;
      this.drivetrain = drivetrain;
      this.resetOdometry = resetOdometry;
      this.endDistance = endDistance;
      thetaController.enableContinuousInput(-Math.PI, Math.PI); //TODO check if pigeon needs this
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    this.drivetrain.setWantedState(Drivetrain.WantedState.TRAJECTORY_FOLLOWING);

    if(resetOdometry){
        drivetrain.initAutonPosition(trajectory.getInitialState());
        SmartDashboard.putString("auton/pose",trajectory.getInitialPose().toString());
        SmartDashboard.putString("auton/angle",trajectory.getInitialState().holonomicRotation.toString());    
    }
  }

  @Override
  public void end(boolean interrupted){
    super.end(interrupted);
    this.drivetrain.setWantedState(Drivetrain.WantedState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return drivetrain.getPose().getX() > endDistance;
  }
}
