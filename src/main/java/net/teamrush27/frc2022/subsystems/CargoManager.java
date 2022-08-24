package net.teamrush27.frc2022.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2022.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;

public class CargoManager implements Subsystem {

	private enum SystemState {
		IDLE,
		SHOOT,
		INDEX,
		EXHAUST,
		SPINUP,
		OPPOSITE_COLOR
	}

	public enum WantedState {
		IDLE,
		SHOOT,
		INDEX,
		EXHAUST,
		SPINUP,
		OPPOSITE_COLOR
	}

	private SystemState currentState = SystemState.IDLE;
	private WantedState wantedState = WantedState.IDLE;

	private final Hopper hopper;
	private final Launcher launcher;
	private final Intake intake;
	private final ColorSensor colorSensor;

	private double currentStateStartTime;

	public CargoManager(Hopper hopper, Launcher launcher, Intake intake, ColorSensor colorSensor){
		this.hopper = hopper;
		this.launcher = launcher;
		this.intake = intake;
		this.colorSensor = colorSensor;
	}

	@Override
	public void readPeriodicInputs(double timestamp) {

	}

	@Override
	public void processLoop(double timestamp) {
		SystemState newState;
		switch (currentState){
			default:
			case IDLE:
				newState = handleIdle();
				break;
			case SHOOT:
				newState = handleShoot();
				break;
			case INDEX:
				newState = handleIndex();
				break;
			case EXHAUST:
				newState = handleExhaust();
				break;
			case SPINUP:
				newState = handleSpinup();
				break;
			case OPPOSITE_COLOR:
				newState = handleOppositeColor(timestamp);
				break;
		}
		if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
	}

	private SystemState handleIdle() {
		launcher.setWantedState(Launcher.WantedState.IDLE);

		hopper.setWantedState(Hopper.WantedState.IDLE);
		intake.setWantedState(Intake.WantedState.RETRACT);

		return defaultStateChange();
	}

	private SystemState handleShoot() {
		launcher.setWantedState(Launcher.WantedState.RUN);
		if(launcher.readyToLaunch()){
			hopper.setWantedState(Hopper.WantedState.SHOOT);
		} else {
			hopper.setWantedState(Hopper.WantedState.SPINUP);
		}
		intake.setWantedState(Intake.WantedState.RETRACT);

		return defaultStateChange();
	}

	private SystemState handleIndex() {
		hopper.setWantedState(Hopper.WantedState.INDEX);
		launcher.setWantedState(Launcher.WantedState.IDLE);
		intake.setWantedState(Intake.WantedState.INTAKE);
		if (colorSensor.determineIndexedBallColorState().equals(ColorSensor.indexedBallState.OPPOSITE_COLOR)){
			//hopper.setShotSpacingTime(2.0);
		}

		return defaultStateChange();
	}

	private SystemState handleExhaust() {
		hopper.setWantedState(Hopper.WantedState.EXHAUST);
		launcher.setWantedState(Launcher.WantedState.IDLE);
		intake.setWantedState(Intake.WantedState.EXHAUST);

		return defaultStateChange();
	}

	private SystemState handleSpinup() {
		if (hopper.getCargoSensorRear()){
			hopper.setWantedState(Hopper.WantedState.SPINUP);
			launcher.setWantedState(Launcher.WantedState.IDLE);
		} else if (intake.getSystemState().equals(Intake.SystemState.RETRACTED)){
			hopper.setWantedState(Hopper.WantedState.SPINUP);
			launcher.setWantedState(Launcher.WantedState.SPINUP);
		} else {
			hopper.setWantedState(Hopper.WantedState.IDLE);
			launcher.setWantedState(Launcher.WantedState.IDLE);
		}
		
		intake.setWantedState(Intake.WantedState.RETRACT);

		return defaultStateChange();
	}

	private SystemState handleOppositeColor(double timestamp){
		hopper.setShotSpacingTime(1.0);
		if(colorSensor.determineIndexedBallColorState().equals(ColorSensor.indexedBallState.OPPOSITE_COLOR)){
			launcher.setOppositeColorShot();
			launcher.setWantedState(Launcher.WantedState.RUN);
			if(launcher.readyToLaunch()){
				hopper.setWantedState(Hopper.WantedState.SHOOT);
			}
		} else {
			hopper.setWantedState(Hopper.WantedState.IDLE);
			launcher.setWantedState(Launcher.WantedState.IDLE);
		}

		intake.setWantedState(Intake.WantedState.RETRACT);
		return defaultStateChange();
	}

	private SystemState defaultStateChange() {
		switch (wantedState){
			default:
			case IDLE:
				return SystemState.IDLE;
			case SHOOT:
				return SystemState.SHOOT;
			case INDEX:
				return SystemState.INDEX;
			case EXHAUST:
				return SystemState.EXHAUST;
			case SPINUP:
				return SystemState.SPINUP;
			case OPPOSITE_COLOR:
				return SystemState.OPPOSITE_COLOR;
		}
	}

	@Override
	public void writePeriodicOutputs(double timestamp) {

        SmartDashboard.putString("cargoManager/currentState", currentState.toString());
        SmartDashboard.putString("cargoManager/wantedState", wantedState.toString());
	}

	@Override
	public void outputTelemetry(double timestamp) {

	}

	@Override
	public void stop() {
	}

	@Override
	public boolean checkSystem() {
		return true;
	}

	@Override
	public void zeroSensors() {

	}

	@Override
	public String getId() {
		return "CargoManager";
	}

	public void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}

    public WantedState getWantedState() {
		return this.wantedState;
	}
}
