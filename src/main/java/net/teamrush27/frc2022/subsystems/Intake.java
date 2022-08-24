package net.cachemoney8096.frc2022o.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.util.CtreConversions;
import static net.cachemoney8096.frc2022o.calibrations.IntakeCals.*;



public class Intake implements Subsystem {
	private TalonFX motor;
	private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
	private Solenoid solenoidExtend;
	private Solenoid solenoidRetract;
	private Hopper hopper;

	public Intake(Hopper hopper){
		motor = new TalonFX(Constants.INTAKE_FALCON_CAN_ID);
		motor.setInverted(true);
		motor.setNeutralMode(NeutralMode.Coast);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
		motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 1.0));
		solenoidExtend = new Solenoid(Constants.PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_EXTEND);
		solenoidRetract = new Solenoid(Constants.PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_RETRACT);

		motor.configAllSettings(motorConfig);
		motor.config_kF(0, INTAKE_KF);
		motor.config_kP(0, INTAKE_KP);
		motor.config_kI(0, INTAKE_KI);
		motor.config_kD(0, INTAKE_KD);


		this.hopper = hopper;
    
	}

	public enum SystemState {
		INTAKE,
		EXHAUST,
		EXTEND,
		RETRACT,
		RETRACTED
	}

	public enum WantedState {
		INTAKE,
		EXHAUST,
		RETRACT
	}

	private static class PeriodicIO {
		// INPUTS
		double timestamp;

		double falconSpeed;
		double falconTemperature;
		double falconSupplyCurrent;
		double falconStatorCurrent;

		boolean currentSolenoidValue;
		boolean currentRetractSolenoidValue;

		// OUTPUTS
		double outputValue;
		boolean solenoidValue;
		boolean solenoidRetractValue;
		double intakePercentOutput;
	}

	private SystemState currentState = SystemState.RETRACTED;
	private WantedState wantedState = WantedState.RETRACT;

	private final PeriodicIO periodicIO = new PeriodicIO();

	private double currentStateStartTime;

	@Override
	public void readPeriodicInputs(double timestamp) {

		periodicIO.falconSpeed = CtreConversions.ticksToRpm(motor.getSelectedSensorVelocity());
		periodicIO.falconTemperature = motor.getTemperature();
		periodicIO.falconSupplyCurrent = motor.getSupplyCurrent();
		periodicIO.falconStatorCurrent = motor.getStatorCurrent();


		periodicIO.currentSolenoidValue = solenoidExtend.get();
		periodicIO.currentRetractSolenoidValue = solenoidRetract.get();
	}

	@Override
	public void processLoop(double timestamp) {
		SystemState newState;
		switch (currentState) {
			default:
			case RETRACTED:
				newState = handleRetracted();
				break;
			case RETRACT:
				newState = handleRetract(timestamp);
				break;
			case EXTEND:
				newState = handleExtend(timestamp);
				break;
			case INTAKE:
				newState = handleIntake();
				break;
			case EXHAUST:
				newState = handleExhaust();
				break;
		}
		if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
	}

	private SystemState handleRetracted() {
		if(hopper.getCargoSensorFront()){
			periodicIO.outputValue = INTAKE_STIR_SPEED_TGT;
		} else {
			periodicIO.outputValue = 0.0;
		}
		periodicIO.solenoidValue = false;
		periodicIO.solenoidRetractValue = true;
		periodicIO.intakePercentOutput = 0.0;

		switch(wantedState) {
			case INTAKE:
			case EXHAUST:
				return SystemState.EXTEND;
		}
		return SystemState.RETRACTED;
	}

	private SystemState handleRetract(double timestamp) {
		periodicIO.solenoidValue = false;
		periodicIO.solenoidRetractValue = true;
		periodicIO.outputValue = INTAKE_RETRACT_SPEED_TGT;
		periodicIO.intakePercentOutput = 0.0;
		if(timestamp - currentStateStartTime > .5){
			if (WantedState.RETRACT.equals(wantedState)) {
				return SystemState.RETRACTED;
			}
		}
		return defaultStateChange();
	}

	private SystemState handleExtend(double timestamp) {
		periodicIO.solenoidValue = true;
		periodicIO.solenoidRetractValue = false;
		periodicIO.outputValue = 0;
		periodicIO.intakePercentOutput = 0.0;

		if(timestamp - currentStateStartTime > .25){
			switch(wantedState) {
			case INTAKE:
				return SystemState.INTAKE;
			case EXHAUST:
				return SystemState.EXHAUST;
			case RETRACT:
				return defaultStateChange();
			}
		}
		return defaultStateChange();
	}

	private SystemState handleIntake() {
		periodicIO.outputValue = INTAKE_SPEED_TGT;
		periodicIO.intakePercentOutput = INTAKE_PERCENT;

		return defaultStateChange();
	}

	private SystemState handleExhaust() {
		periodicIO.outputValue = INTAKE_EXHAUST_SPEED_TGT;
		periodicIO.intakePercentOutput = EXHAUST_PERCENT;

		return defaultStateChange();
	}

	private SystemState defaultStateChange() {
		switch(wantedState) {
			case INTAKE:
			case EXHAUST:
				if(SystemState.INTAKE.equals(currentState)){
					return SystemState.INTAKE;
				}
				if(SystemState.EXHAUST.equals(currentState)){
					return SystemState.EXHAUST;
				}
				return SystemState.EXTEND;
			default:
			case RETRACT:
				if(SystemState.RETRACTED.equals(currentState)) {
					return SystemState.RETRACTED;
				}
				return SystemState.RETRACT;
		}
	}

	@Override
	public void writePeriodicOutputs(double timestamp) {
		solenoidExtend.set(periodicIO.solenoidValue);
		solenoidRetract.set(periodicIO.solenoidRetractValue);
        //if (currentState == SystemState.RETRACT || currentState == SystemState.RETRACTED){
		//motor.set(TalonFXControlMode.Velocity, CtreConversions.rpmToTicks(periodicIO.outputValue));

		if (currentState == SystemState.RETRACT){
			motor.neutralOutput();
		} else if (currentState == SystemState.RETRACTED && periodicIO.outputValue < -100.0){
			motor.set(TalonFXControlMode.Velocity, CtreConversions.rpmToTicks(periodicIO.outputValue));
		} else {
			motor.set(TalonFXControlMode.PercentOutput, periodicIO.intakePercentOutput);
		}

	}

	@Override
	public void outputTelemetry(double timestamp) {
		SmartDashboard.putString("intake/currentState", currentState.toString());
		SmartDashboard.putNumber("intake/outputPct", periodicIO.outputValue);
		SmartDashboard.putNumber("intake/supplyCurrent", periodicIO.falconSupplyCurrent);
		SmartDashboard.putNumber("intake/speed", periodicIO.falconSpeed);
	}

	@Override
	public void stop() {
		motor.neutralOutput();
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
		return "Intake";
	}

    public void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}

	public SystemState getSystemState(){
		return currentState;
	}
}
