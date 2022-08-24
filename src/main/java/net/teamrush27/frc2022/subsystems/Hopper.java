package net.cachemoney8096.frc2022o.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.cachemoney8096.frc2022o.Constants;
import static net.cachemoney8096.frc2022o.calibrations.HopperCals.*;

public class Hopper implements Subsystem {

	private final CANSparkMax frontHopperMotor;
	//private final CANSparkMax hopperFollower;
	private final TalonFX rearHopperMotor;
	private final TalonFXConfiguration rearHopperMotorConfig = new TalonFXConfiguration();

	private final CANSparkMax indexLeader;
	private final CANSparkMax indexFollower;

	private final DigitalInput cargoSensorFront;
	private final DigitalInput cargoSensorMid;
	private final DigitalInput cargoSensorRear;
	private final DigitalInput cargoSensorIndexer;

	private final RelativeEncoder frontHopperEncoder;
	private final RelativeEncoder indexLeaderEncoder;

	private final Compressor compressor;
	private final Solenoid solenoid;

	private static class PeriodicIO {

		// INPUTS
		double timestamp;

		double rearHopperSpeed;
		double rearHopperTemperature;
		double rearHopperCurrent;
		double rearHopperVoltage;

		double frontHopperSpeed;
		double frontHopperTemperature;
		double frontHopperCurrent;
		double frontHopperVoltage;

		double hopperFollowerTemperature;
		double hopperFollowerCurrent;
		double hopperRPM;

		boolean cargoSensorFront;
		boolean cargoSensorMid;
		boolean cargoSensorRear;
		boolean cargoSensorIndexer;

		double indexSpeed;
		double indexLeaderTemperature;
		double indexLeaderCurrent;
		double indexFollowerTemperature;
		double indexFollowerCurrent;

		boolean currentSolenoidValue;
		double pressure;

		double frontBallTrackSpeed;

		// OUTPUTS
		double outputValue;
		double rearHopperSpeedOutput;
		double indexOutputValue;
		boolean solenoidValue;
		double shotSpacingStartTime;
		double shotSpacingTime;
		double ballFloorPercent;
	}

	public Hopper(Compressor compressor){

		frontHopperMotor = new CANSparkMax(Constants.HOPPER_SPARK_MAX_LEADER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
		frontHopperMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		frontHopperMotor.setInverted(false);
		//frontHopperMotor.setSmartCurrentLimit(30);
		frontHopperMotor.setSmartCurrentLimit(25);
		frontHopperMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		frontHopperMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
		frontHopperMotor.getPIDController().setFF(HOPPER_KF);
		frontHopperMotor.getPIDController().setP(HOPPER_KP);
		frontHopperMotor.getPIDController().setI(HOPPER_KI);
		frontHopperMotor.getPIDController().setD(HOPPER_KD);
		/*
		hopperFollower = new CANSparkMax(Constants.HOPPER_SPARK_MAX_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
		hopperFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
		hopperFollower.setSmartCurrentLimit(30);
		hopperFollower.follow(hopperLeader, true);
		hopperFollower.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
		hopperFollower.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		hopperFollower.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
*/


		rearHopperMotor = new TalonFX(Constants.REAR_HOPPER_F500_CAN_ID);
		rearHopperMotor.configAllSettings(rearHopperMotorConfig);
		rearHopperMotor.config_kF(0, REAR_HOPPER_F500_KF);
		rearHopperMotor.config_kP(0, REAR_HOPPER_F500_KP);
		rearHopperMotor.config_kI(0, REAR_HOPPER_F500_KI);
		rearHopperMotor.config_kD(0, REAR_HOPPER_F500_KD);
		rearHopperMotor.setNeutralMode(NeutralMode.Brake);
		rearHopperMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20.0, 20, 0));
		rearHopperMotor.configClosedloopRamp(0.25);

		indexLeader = new CANSparkMax(Constants.INDEX_WHEEL_SPARK_MAX_LEADER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
		indexLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
		indexLeader.setInverted(false);
		indexLeader.setSmartCurrentLimit(40);
		indexLeader.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		indexLeader.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
		indexLeader.getPIDController().setFF(INDEX_KF);
		indexLeader.getPIDController().setP(INDEX_KP);
		indexLeader.getPIDController().setI(INDEX_KI);
		indexLeader.getPIDController().setD(INDEX_KD);
		indexFollower = new CANSparkMax(Constants.INDEX_WHEEL_SPARK_MAX_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
		indexFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
		indexFollower.setSmartCurrentLimit(40);
		indexFollower.follow(indexLeader, true);
		indexFollower.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
		indexFollower.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		indexFollower.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);

		frontHopperEncoder = frontHopperMotor.getEncoder();
		indexLeaderEncoder = indexLeader.getEncoder();

		cargoSensorFront = new DigitalInput(Constants.HOPPER_DIO_CARGO_SENSOR_FRONT);
		cargoSensorMid = new DigitalInput(Constants.HOPPER_DIO_CARGO_SENSOR_MID);
		cargoSensorRear = new DigitalInput(Constants.HOPPER_DIO_CARGO_SENSOR_REAR);
		cargoSensorIndexer = new DigitalInput(Constants.HOPPER_DIO_CARGO_SENSOR_INDEXER);

		this.compressor = compressor;
		solenoid = new Solenoid(Constants.PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, Constants.HOPPER_SOLENOID_BRAKE);
		periodicIO.shotSpacingTime = 1.0;
		SmartDashboard.putNumber("launcher/shotSpacing",periodicIO.shotSpacingTime);
		rearHopperMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 20, 1.0));
	}

	private enum SystemState {
		IDLE,
		SHOOT,
		INDEX,
		EXHAUST,
		SHOT_SPACING,
		SPINUP
	}

	public enum WantedState {
		IDLE,
		SHOOT,
		INDEX,
		EXHAUST,
		SPINUP
	}



	private SystemState currentState = SystemState.IDLE;
	private WantedState wantedState = WantedState.IDLE;
	private SystemState previousState = SystemState.IDLE;

	private final PeriodicIO periodicIO = new PeriodicIO();

	private double currentStateStartTime;

	@Override
	public void readPeriodicInputs(double timestamp) {
		periodicIO.timestamp = timestamp;

		periodicIO.frontHopperSpeed = frontHopperEncoder.getVelocity();
		periodicIO.frontHopperTemperature = frontHopperMotor.getMotorTemperature();
		periodicIO.frontHopperCurrent = frontHopperMotor.getOutputCurrent();
		periodicIO.frontHopperVoltage = frontHopperMotor.getAppliedOutput();


		periodicIO.rearHopperSpeed = rearHopperMotor.getSelectedSensorVelocity();
		periodicIO.rearHopperTemperature = rearHopperMotor.getTemperature();
		periodicIO.rearHopperCurrent = rearHopperMotor.getStatorCurrent();
		periodicIO.rearHopperVoltage = rearHopperMotor.getMotorOutputVoltage();

		periodicIO.indexSpeed = indexLeaderEncoder.getVelocity();
		periodicIO.indexLeaderTemperature = indexLeader.getMotorTemperature();
		periodicIO.indexLeaderCurrent = indexLeader.getOutputCurrent();

		periodicIO.indexFollowerTemperature = indexFollower.getMotorTemperature();
		periodicIO.indexFollowerCurrent = indexFollower.getOutputCurrent();

		periodicIO.cargoSensorFront = !cargoSensorFront.get();
		periodicIO.cargoSensorMid = !cargoSensorMid.get();
		periodicIO.cargoSensorRear = !cargoSensorRear.get();
		periodicIO.cargoSensorIndexer = !cargoSensorIndexer.get();

		periodicIO.pressure = compressor.getPressure();
		periodicIO.currentSolenoidValue = solenoid.get();
		//periodicIO.shotSpacingTime = SmartDashboard.getNumber("launcher/shotSpacing", HOPPER_SHOT_SPACING_TIME);
		periodicIO.hopperRPM = rearHopperMotor.getSelectedSensorVelocity() / 2048 * 10 * 60;
	}

	@Override
	public void processLoop(double timestamp) {
		SystemState newState;
		switch (currentState) {
			default:
			case IDLE:
				newState = handleIdle(timestamp);
				break;
			case SHOOT:
				newState = handleShoot(timestamp);
				break;
			case INDEX:
				newState = handleIndex(timestamp);
				break;
			case EXHAUST:
				newState = handleExhaust();
				break;
			case SHOT_SPACING:
				newState = handleShotSpacing(timestamp);
				break;
			case SPINUP:
				newState = handleSpinup();
				break;
		}
		if (newState != currentState) {
			previousState = currentState;
			currentState = newState;
			currentStateStartTime = timestamp;
		}
		SmartDashboard.putString("hopper/currentState", currentState.toString());
	}

	private SystemState handleIdle(double timestamp) {
		periodicIO.solenoidValue = false;

		/*
		if(periodicIO.cargoSensorFront && timestamp - currentStateStartTime > 0.5) {
			periodicIO.outputValue = HOPPER_IDLE_TWO_BALL_SPEED_TGT;
		} else {
			periodicIO.outputValue = 0;
		}

		if(periodicIO.cargoSensorMid && periodicIO.cargoSensorFront && timestamp - currentStateStartTime > 0.5) {
			periodicIO.indexOutputValue = INDEXER_IDLE_TWO_BALL_SPEED_TGT;
		} else {
			periodicIO.indexOutputValue = 0;
		}
		*/
		if (previousState.equals(SystemState.INDEX) && timestamp - currentStateStartTime <= 0.5){
			periodicIO.outputValue = HOPPER_INDEX_SPEED_TGT;
			periodicIO.rearHopperSpeedOutput = REAR_HOPPER_INDEX_SPEED_TGT;
			if (!periodicIO.cargoSensorIndexer){
				periodicIO.indexOutputValue = INDEXER_INDEX_IN_SPEED_TGT;
			} else {
				periodicIO.indexOutputValue = INDEXER_INDEX_SPEED_TGT;
			}
		} else {
			periodicIO.outputValue = 0.0;
			periodicIO.indexOutputValue = 0.0;
			periodicIO.rearHopperSpeedOutput = REAR_HOPPER_IDLE_SPEED_TGT;
		}

		return defaultStateChange();
	}

	private SystemState handleShoot(double timestamp) {
		periodicIO.solenoidValue = true;
		periodicIO.outputValue = HOPPER_SHOOT_SPEED_TGT;
		periodicIO.rearHopperSpeedOutput = REAR_HOPPER_SHOOT_SPEED_TGT;
		periodicIO.indexOutputValue = INDEXER_SHOOT_SPEED_TGT;
		if (periodicIO.cargoSensorRear){
			periodicIO.shotSpacingStartTime = timestamp;
			return SystemState.SHOT_SPACING;
		} else {
			return defaultStateChange();}
	}

	private SystemState handleIndex(double timestamp) {
		periodicIO.solenoidValue = false;
		periodicIO.outputValue = HOPPER_INDEX_SPEED_TGT;
		periodicIO.rearHopperSpeedOutput = REAR_HOPPER_INDEX_SPEED_TGT;

		if (!periodicIO.cargoSensorIndexer){
			periodicIO.indexOutputValue = INDEXER_INDEX_IN_SPEED_TGT;
		} else {
			periodicIO.indexOutputValue = INDEXER_INDEX_SPEED_TGT;
		}

		return defaultStateChange();
	}

	private SystemState handleExhaust() {
		periodicIO.solenoidValue = true;
		periodicIO.outputValue = HOPPER_EXHAUST_SPEED_TGT;
		periodicIO.rearHopperSpeedOutput = REAR_HOPPER_EXHAUST_SPEED_TGT;
		periodicIO.indexOutputValue = INDEXER_EXHAUST_SPEED_TGT;

		return defaultStateChange();
	}

	private SystemState handleShotSpacing(double timestamp) {
		periodicIO.solenoidValue = true;
		periodicIO.outputValue = HOPPER_SHOOT_SPEED_TGT;
		periodicIO.rearHopperSpeedOutput = REAR_HOPPER_SHOT_SPACING_SPEED_TGT;
		periodicIO.indexOutputValue = INDEXER_SHOT_SPACING_SPEED_TGT;
		SmartDashboard.putNumber("hopper/shotspacingTimestamp", timestamp);
		if (timestamp >= periodicIO.shotSpacingStartTime + periodicIO.shotSpacingTime){
			return defaultStateChange();
		} else {
			return SystemState.SHOT_SPACING;
		}
	}

	private SystemState handleSpinup() {
		periodicIO.solenoidValue = true;
		periodicIO.outputValue = HOPPER_SPINUP_SPEED_TGT;
		periodicIO.rearHopperSpeedOutput = REAR_HOPPER_SPINUP_SPEED_TGT;
		periodicIO.indexOutputValue = INDEXER_SPINUP_SPEED_TGT;
		return defaultStateChange();
	}

	private SystemState defaultStateChange(){
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
		}
	}

	@Override
	public void writePeriodicOutputs(double timestamp) {
		//solenoid.set(periodicIO.solenoidValue);
		if (Math.abs(periodicIO.outputValue) > 1.0) {
			frontHopperMotor.getPIDController().setReference(periodicIO.outputValue, CANSparkMax.ControlType.kVelocity);
		} else {
			frontHopperMotor.stopMotor();
		}
		rearHopperMotor.set(TalonFXControlMode.Velocity, Launcher.rpmToTicks(periodicIO.rearHopperSpeedOutput));
		//hopperMotor.set(TalonFXControlMode.PercentOutput, periodicIO.ballFloorPercent);
		indexLeader.getPIDController().setReference(periodicIO.indexOutputValue,CANSparkMax.ControlType.kVelocity);
	}

	@Override
	public void outputTelemetry(double timestamp) {

		SmartDashboard.putNumber("pneumatics/pressure", periodicIO.pressure);
		SmartDashboard.putBoolean("hopper/cargoSensorFront", periodicIO.cargoSensorFront);
		SmartDashboard.putBoolean("hopper/cargoSensorMid", periodicIO.cargoSensorMid);
		SmartDashboard.putBoolean("hopper/cargoSensorRear", periodicIO.cargoSensorRear);
		SmartDashboard.putNumber("hopper/rear_voltage", periodicIO.rearHopperVoltage);
		SmartDashboard.putNumber("hopper/rear_current", periodicIO.rearHopperCurrent);
		SmartDashboard.putNumber("hopper/rear_temp", periodicIO.rearHopperTemperature);
		SmartDashboard.putNumber("hopper/front_voltage", periodicIO.frontHopperVoltage);
		SmartDashboard.putNumber("hopper/front_current", periodicIO.frontHopperCurrent);
		SmartDashboard.putNumber("hopper/front_temp", periodicIO.frontHopperTemperature);
		SmartDashboard.putNumber("hopper/hopperSpeedTgt", periodicIO.outputValue);
		SmartDashboard.putNumber("hopper/ballTrackSpeed", periodicIO.hopperRPM);
		SmartDashboard.putBoolean("hopper/cargoSensorIndexer", periodicIO.cargoSensorIndexer);
		SmartDashboard.putNumber("hopper/frontBallTrackSpeed", periodicIO.frontBallTrackSpeed);
		SmartDashboard.putNumber("hopper/shotSpacingTime", periodicIO.shotSpacingTime);
	}

	@Override
	public void stop() {
		rearHopperMotor.neutralOutput();
		indexLeader.stopMotor();
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
		return "Hopper";
	}

	public void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}

	public boolean hasFirstBall(){
		return periodicIO.cargoSensorMid;
	}

	public boolean hasSecondBall(){
		return periodicIO.cargoSensorFront;
	}

	public boolean hasShot(){
		return periodicIO.cargoSensorRear;
	}

	public void setShotSpacingTime(double time){
		periodicIO.shotSpacingTime = time;
	}

	public boolean getCargoSensorRear(){
		return periodicIO.cargoSensorRear;
	}
	public boolean getCargoSensorFront(){
		return periodicIO.cargoSensorFront;
	}

	public void runBallFloor(double percent, double speed){
		periodicIO.ballFloorPercent = percent;
		periodicIO.outputValue = speed;
	}

}
