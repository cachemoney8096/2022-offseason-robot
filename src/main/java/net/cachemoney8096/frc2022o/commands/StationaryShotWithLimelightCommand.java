package net.cachemoney8096.frc2022o.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.cachemoney8096.frc2022o.subsystems.*;


public class StationaryShotWithLimelightCommand extends CommandBase {

    private final CargoManager cargoManager;
    private final Launcher launcher;
    private final Drivetrain drivetrain;
    private final Hopper hopper;
    private int ballCount;
    private final boolean shotSensor_Prev = false;
    private final Debouncer shotSensorDebounce = new Debouncer(0.04, Debouncer.DebounceType.kRising);
    private double startTime;

    public StationaryShotWithLimelightCommand(CargoManager cargoManager, Launcher launcher, Drivetrain drivetrain, Hopper hopper, int ballCount){
        this.cargoManager = cargoManager;
        this.launcher = launcher;
        this.drivetrain = drivetrain;
        this.hopper = hopper;
        this.ballCount = ballCount;
    }

    @Override
    public void initialize(){
        super.initialize();
        //drivetrain.setWantedState(Drivetrain.WantedState.GOAL_TRACKING);
        //launcher.setWantedState(Launcher.WantedState.SPINUP);

        //hopper.setShotSpacingTime(0.25);
        launcher.setAutonShot(2.0);
        //cargoManager.setWantedState(CargoManager.WantedState.SPINUP);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        drivetrain.setWantedState(Drivetrain.WantedState.GOAL_TRACKING);
        //hopper.setShotSpacingTime(0.25);
        if (Timer.getFPGATimestamp() - startTime > 0.5) {
            if (drivetrain.isGoalTracked()) {
                launcher.setLimelightShot();
                cargoManager.setWantedState(CargoManager.WantedState.SHOOT);
            } else if (drivetrain.isGoalTracking()) {
                launcher.setLimelightShot();
                cargoManager.setWantedState(CargoManager.WantedState.SPINUP);
            }
        }

        if (shotSensorDebounce.calculate(hopper.getCargoSensorRear()) && !shotSensor_Prev){
            ballCount = ballCount - 1;
        }

        SmartDashboard.putNumber("auton/ballCount", ballCount);

    }

    @Override
    public boolean isFinished() {
        return ballCount <= 0;
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        cargoManager.setWantedState(CargoManager.WantedState.IDLE);
    }




}
