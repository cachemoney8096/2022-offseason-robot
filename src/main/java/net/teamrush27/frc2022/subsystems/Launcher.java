package net.teamrush27.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static net.teamrush27.frc2022.Constants.*;
import static net.teamrush27.frc2022.calibrations.GlobalCals.*;
import static net.teamrush27.frc2022.calibrations.LauncherCals.*;

import net.teamrush27.frc2022.FeatureEnables;
import net.teamrush27.frc2022.calibrations.DrivetrainCals;
import net.teamrush27.frc2022.calibrations.LauncherCals;
import net.teamrush27.frc2022.util.ModuleToChassisSpeeds;
import net.teamrush27.frc2022.util.XboxController;
import net.teamrush27.frc2022.util.oneDimensionalLookup;

public class Launcher implements Subsystem {

	// SUBSYSTEM SETUP

	private TalonFX leader;
	private TalonFX follower;
    private TalonFX hood;
    private final TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

	private final CANSparkMax topRollerMotor;
	private final RelativeEncoder topRollerMotorEncoder;

	private static final double DEGREES_PER_COUNT = (80d-58d) / 42_000d;

	private Limelight limelight;
	private Hopper hopper;

	private LinearFilter limelightDistanceFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * GOAL_TRACKING_DISTANCE_FC), 0.02);
	private LinearFilter limelightAngleFilter = LinearFilter.singlePoleIIR(1/(2*Math.PI * DrivetrainCals.GOAL_TRACKING_ANGLE_ERROR_FC), 0.02);

	private final Debouncer cargoSensorRearDebounce = new Debouncer(LAUNCHER_SECOND_BALL_FALL_DEBOUNCE, Debouncer.DebounceType.kFalling);
	private final Debouncer hoodSpeedDebounce = new Debouncer(5.0d, Debouncer.DebounceType.kRising);
	private final Debouncer readyToLaunchDebounce = new Debouncer(0.2, Debouncer.DebounceType.kFalling);

	private static boolean shootWhileMovingLocked;

	private boolean manualOverride = false;

	public enum ShotType {
		FENDER,
		LIMELIGHT,
		TARMAC_LINE,
		FENDER_LOW,
		OPPOSITE_COLOR,
		SHOOT_WHILE_MOVING,
		AUTON
	}

	private static class PeriodicIO {
		// INPUTS
		double timestamp;
		double actualRPM;

		double leaderTemperature;
		double leaderSupplyCurrent;
		double leaderStatorCurrent;

		double followerTemperature;
		double followerSupplyCurrent;
		double followerStatorCurrent;

		double hoodTemperature;
		double hoodSupplyCurrent;
		double hoodStatorCurrent;
		double hoodSpeed;

		double currentHoodAngle;
		double limelightDistanceToTarget;
		double limelightAngleError;

		boolean cargoSensorRear;
		boolean cargoSensorRear_prev;

		ShotType shotType;

		double topRollerSpeedActual;

		// OUTPUTS
		double outputValue;
		double hoodAngle;
		boolean secondBall;
		double topRollerSpeedTgt;
	}

	// 3.1m 4100rpm 60deg

	public Launcher(Limelight limelight, Hopper hopper){
		leader = new TalonFX(LAUNCHER_MAIN_ROLLER_LEADER_ID, "Drivetrain");
		follower = new TalonFX(LAUNCHER_MAIN_ROLLER_FOLLOWER_ID, "Drivetrain");
        hood = new TalonFX(LAUNCHER_HOOD_ID);
		follower.follow(leader);
		follower.setInverted(InvertType.OpposeMaster);

		leader.configPeakOutputReverse(0);
		follower.configPeakOutputReverse(0);
		follower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
		follower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);

        leader.configAllSettings(leaderConfig);
        leader.config_kF(0, LAUNCHER_KF);
        leader.config_kP(0, LAUNCHER_KP);
        leader.config_kI(0, LAUNCHER_KI);
        leader.config_kD(0, LAUNCHER_KD);
		leader.config_IntegralZone(0, LAUNCHER_IZONE);

        follower.configAllSettings(followerConfig);
        follower.config_kF(0, LAUNCHER_KF);
        follower.config_kP(0, LAUNCHER_KP);
        follower.config_kI(0, LAUNCHER_KI);
        follower.config_kD(0, LAUNCHER_KD);
		follower.config_IntegralZone(0, LAUNCHER_IZONE);

		hood.setSelectedSensorPosition(0);

        hood.configAllSettings(hoodConfig);
		hood.configAllSettings(followerConfig);
		hood.config_IntegralZone(0, 500.0);
		hood.config_kF(0, HOOD_KF);
		hood.config_kP(0, HOOD_KP);
		hood.config_kI(0, HOOD_KI);
		hood.config_kD(0, HOOD_KD);
		hood.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);

		hood.setNeutralMode(NeutralMode.Brake);
		hood.configPeakOutputReverse(-1);
		hood.configPeakOutputForward(1);
        hood.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 10, 1.0));
		periodicIO.outputValue = LAUNCHER_SPEED_FENDER;
		periodicIO.hoodAngle = 79.5;
		periodicIO.topRollerSpeedTgt = LAUNCHER_TOP_ROLLER_SPEED_FENDER;
        periodicIO.shotType = ShotType.LIMELIGHT;

		topRollerMotor = new CANSparkMax(LAUNCHER_TOP_ROLLER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
		topRollerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		topRollerMotor.setInverted(false);
		topRollerMotor.setSmartCurrentLimit(40);
		topRollerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		topRollerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
		topRollerMotor.getPIDController().setFF(LAUNCHER_TOP_ROLLER_KF);
		topRollerMotor.getPIDController().setP(LAUNCHER_TOP_ROLLER_KP);
		topRollerMotor.getPIDController().setI(LAUNCHER_TOP_ROLLER_KI);
		topRollerMotor.getPIDController().setD(LAUNCHER_TOP_ROLLER_KD);
		topRollerMotorEncoder = topRollerMotor.getEncoder();

		this.limelight = limelight;
		this.hopper = hopper;
		periodicIO.secondBall = false;
		periodicIO.cargoSensorRear_prev = false;
		shootWhileMovingLocked = false;
	}

	// LOGIC SETUP

	private static final double RPM_ALLOWED_ERROR = LAUNCHER_SPEED_TOLERANCE;

	private enum SystemState {
		SPINUP,
		RUN,
		IDLE,
		HOMING
	}

	public enum WantedState {
		RUN,
		IDLE,
		SPINUP
	}


	private SystemState currentState = SystemState.IDLE;
	private WantedState wantedState = WantedState.IDLE;

	private final PeriodicIO periodicIO = new PeriodicIO();

	private double currentStateStartTime;

	@Override
	public void readPeriodicInputs(double timestamp) {
		// ticks per 100ms / 2048
		periodicIO.cargoSensorRear_prev = periodicIO.cargoSensorRear;
		var rotationsPer100ms = leader.getSelectedSensorVelocity() / 2048d;
		// rotations per 100ms * 10 = rotations per second * 60 = rotations per minute
		periodicIO.actualRPM = rotationsPer100ms * 10 * 60;
		periodicIO.timestamp = timestamp;

		periodicIO.leaderTemperature = leader.getTemperature();
		periodicIO.leaderSupplyCurrent = leader.getSupplyCurrent();
		periodicIO.leaderStatorCurrent = leader.getStatorCurrent();

		periodicIO.followerTemperature = follower.getTemperature();
		periodicIO.followerSupplyCurrent = follower.getSupplyCurrent();
		periodicIO.followerStatorCurrent = follower.getStatorCurrent();

		periodicIO.hoodTemperature = hood.getTemperature();
		periodicIO.hoodSupplyCurrent = hood.getSupplyCurrent();
		periodicIO.hoodStatorCurrent = hood.getStatorCurrent();
		periodicIO.hoodSpeed = hood.getSelectedSensorVelocity();
        
        periodicIO.currentHoodAngle = 80d - (hood.getSelectedSensorPosition() * DEGREES_PER_COUNT);

		if(limelight.hasTargets()){
			periodicIO.limelightDistanceToTarget = limelightDistanceFilter.calculate(limelight.distanceToTarget());
			periodicIO.limelightAngleError = limelightAngleFilter.calculate(limelight.angle_error());
		}
		//periodicIO.outputValue = SmartDashboard.getNumber("launcher/SpeedTgt", 3400.0);
		//periodicIO.hoodAngle = SmartDashboard.getNumber("launcher/HoodAngleTgt", 78.0);
		periodicIO.cargoSensorRear = hopper.getCargoSensorRear();
		periodicIO.topRollerSpeedActual = topRollerMotorEncoder.getVelocity();
	}

	@Override
	public void processLoop(double timestamp) {
		SystemState newState;
		switch (currentState) {
			case HOMING:
				newState = handleHoming();
				break;
			case SPINUP:
				newState = handleSpinUp();
				break;
			case RUN:
				newState = handleRun();
				break;
			case IDLE:
			default:
				newState = handleIdle();
				break;
		}
		if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
	}

	private SystemState handleHoming() {
		periodicIO.secondBall = false;
		if(periodicIO.hoodStatorCurrent > 7.0 ||
			hoodSpeedDebounce.calculate(periodicIO.hoodSpeed > 100d)){
			hood.setSelectedSensorPosition(0);
			return SystemState.IDLE;
		}
		return SystemState.HOMING;
	}

	private SystemState handleIdle() {
//		periodicIO.outputValue = 0;
		periodicIO.secondBall = false;
		return defaultStateChange();
	}

	private SystemState handleRun() {
//		periodicIO.outputValue = LAUNCHER_SPEED_FENDER;
		checkFenderSecondBall();
		return defaultStateChange();
	}

	private SystemState handleSpinUp() {
//		periodicIO.outputValue = LAUNCHER_SPEED_FENDER;
		checkFenderSecondBall();
		return defaultStateChange();
	}

	private SystemState defaultStateChange(){
		switch (wantedState){
			case RUN:
				if(periodicIO.actualRPM > 100 &&
						Math.abs(periodicIO.actualRPM - periodicIO.outputValue) < RPM_ALLOWED_ERROR
						&& Math.abs(periodicIO.hoodAngle - periodicIO.currentHoodAngle) <= HOOD_ERROR_TOLERANCE
						&& Math.abs(periodicIO.topRollerSpeedActual - periodicIO.topRollerSpeedTgt) < LAUNCHER_TOP_ROLLER_SPEED_TOLERANCE) {
					/*
					if (periodicIO.shotType.equals(ShotType.SHOOT_WHILE_MOVING) ||
						periodicIO.shotType.equals(ShotType.LIMELIGHT)){*/
					if (periodicIO.shotType.equals(ShotType.SHOOT_WHILE_MOVING)){
						if (checkShootWhileMovingLocked()){
							return SystemState.RUN;
						} else {
							return SystemState.SPINUP;
						}
					} else {
						return SystemState.RUN;
					}
				}
				return SystemState.SPINUP;
			case SPINUP:
				return SystemState.SPINUP;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	public static double rpmToTicks(double rpm){
		var rps = rpm / 60d;
		var rp100ms = rps / 10;
		return rp100ms * 2048;
	}

	@Override
	public void writePeriodicOutputs(double timestamp) {
		switch(currentState){
		case SPINUP:
		case RUN:
			leader.set(TalonFXControlMode.Velocity, rpmToTicks(Double.isNaN(periodicIO.outputValue) ? 0 : periodicIO.outputValue));
			topRollerMotor.getPIDController().setReference(Double.isNaN(periodicIO.topRollerSpeedTgt) ? 0 : periodicIO.topRollerSpeedTgt, CANSparkMax.ControlType.kVelocity);
			break;
		default:
		case IDLE:
			leader.neutralOutput();
			topRollerMotor.getPIDController().setReference(0.0, CANSparkMax.ControlType.kVelocity);
			break;
		}

		if(SystemState.HOMING.equals(currentState)){
			hood.set(TalonFXControlMode.PercentOutput, -.1);
		} else {
			hood.set(TalonFXControlMode.Position, (80 - (Double.isNaN(periodicIO.hoodAngle) ? 80 : periodicIO.hoodAngle)) / DEGREES_PER_COUNT);
		}

	}

	@Override
	public void stop() {
		leader.neutralOutput();
		hood.neutralOutput();
		topRollerMotor.stopMotor();
	}

	@Override

	public boolean checkSystem() {
		/*
		System.out.println("////////////////////////////////////////////////");
		System.out.println("Checking Launcher Subsystem");

		// SETUP
		leader.neutralOutput();
		follower.neutralOutput();

		ArrayList<Double> currentList = new ArrayList<>();
		ArrayList<Double> rpmList = new ArrayList<>();
		double averageRPM, maxRPM, minRPM;

		boolean stepComplete = false;
		System.out.println("Checking Launcher Leader Motor");
		System.out.println("Entering SpinUp");
		double start = Timer.getFPGATimestamp();
		leader.set(ControlMode.PercentOutput, 0.8);
		EvictingQueue<Double> queue = EvictingQueue.create(50);
	 	do {
			queue.add(leader.getSelectedSensorVelocity() / 2048d * 10 * 60);
			if(queue.remainingCapacity() == 0){
				averageRPM = queue.stream().mapToDouble(value -> value).average().getAsDouble();
				maxRPM = queue.stream().mapToDouble(value -> value).max().getAsDouble();
				minRPM = queue.stream().mapToDouble(value -> value).min().getAsDouble();
				if(maxRPM - averageRPM < RPM_ALLOWED_ERROR && averageRPM - minRPM < RPM_ALLOWED_ERROR){
					stepComplete = true;
				}
			}
			Timer.delay(.005);
		} while(!stepComplete);

		double spinUpComplete = Timer.getFPGATimestamp() - start;
		System.out.println("SpinUp Complete, Sampling ...");
		stepComplete = false;

		do {
			rpmList.add(leader.getSelectedSensorVelocity() / 2048d * 10 * 60);
			currentList.add(leader.getSupplyCurrent());
			if(rpmList.size() == 20){
				stepComplete = true;
			} else {
				Timer.delay(.1);
			}
		} while(!stepComplete);
		leader.neutralOutput();

		averageRPM = rpmList.stream().mapToDouble(value -> value).average().getAsDouble();
		maxRPM = rpmList.stream().mapToDouble(value -> value).max().getAsDouble();
		minRPM = rpmList.stream().mapToDouble(value -> value).min().getAsDouble();

		double averageCurrent = currentList.stream().mapToDouble(value -> value).average().getAsDouble();
		double maxCurrent = currentList.stream().mapToDouble(value -> value).max().getAsDouble();
		double minCurrent = currentList.stream().mapToDouble(value -> value).min().getAsDouble();

		System.out.println("Sampling complete!");
		System.out.println("////////////////////////////////////////////////");
		System.out.println("LEADER MOTOR");
		System.out.println(String.format("SPIN UP TIME: %s seconds",spinUpComplete));
		System.out.println(String.format("RPM: avg: %s min: %s max: %s", averageRPM, minRPM, maxRPM));
		System.out.println(String.format("CURRENT: avg: %s min: %s max: %s", averageCurrent, minCurrent, maxCurrent));
		System.out.println("////////////////////////////////////////////////");
		System.out.println("Press START to continue test ...");
		while(!RobotContainer.getInstance().driverController.getStartButton()){
			// waiting
		}

		currentList = new ArrayList<>();
		rpmList = new ArrayList<>();

		stepComplete = false;
		System.out.println("Checking Launcher Follower Motor");
		System.out.println("Entering SpinUp");
		start = Timer.getFPGATimestamp();
		follower.set(ControlMode.PercentOutput, 0.8);
		queue = EvictingQueue.create(50);
		do {
			queue.add(follower.getSelectedSensorVelocity() / 2048d * 10 * 60);
			if(queue.remainingCapacity() == 0){
				averageRPM = queue.stream().mapToDouble(value -> value).average().getAsDouble();
				maxRPM = queue.stream().mapToDouble(value -> value).max().getAsDouble();
				minRPM = queue.stream().mapToDouble(value -> value).min().getAsDouble();
				if(maxRPM - averageRPM < RPM_ALLOWED_ERROR && averageRPM - minRPM < RPM_ALLOWED_ERROR){
					stepComplete = true;
				}
			}
			Timer.delay(.005);
		} while(!stepComplete);

		spinUpComplete = Timer.getFPGATimestamp() - start;
		System.out.println("SpinUp Complete, Sampling ...");
		stepComplete = false;
		do {
			rpmList.add(follower.getSelectedSensorVelocity() / 2048d * 10 * 60);
			currentList.add(follower.getSupplyCurrent());
			if(rpmList.size() == 20){
				stepComplete = true;
			} else {
				Timer.delay(.1);
			}
		} while(!stepComplete);

		averageRPM = rpmList.stream().mapToDouble(value -> value).average().getAsDouble();
		maxRPM = rpmList.stream().mapToDouble(value -> value).max().getAsDouble();
		minRPM = rpmList.stream().mapToDouble(value -> value).min().getAsDouble();

		averageCurrent = currentList.stream().mapToDouble(value -> value).average().getAsDouble();
		maxCurrent = currentList.stream().mapToDouble(value -> value).max().getAsDouble();
		minCurrent = currentList.stream().mapToDouble(value -> value).min().getAsDouble();

		System.out.println("Sampling complete!");
		System.out.println("////////////////////////////////////////////////");
		System.out.println("FOLLOWER MOTOR");
		System.out.println(String.format("SPIN UP TIME: %s seconds",spinUpComplete));
		System.out.println(String.format("RPM: avg: %s min: %s max: %s", averageRPM, minRPM, maxRPM));
		System.out.println(String.format("CURRENT: avg: %s min: %s max: %s", averageCurrent, minCurrent, maxCurrent));
		System.out.println("////////////////////////////////////////////////");
*/
		return true;
	}

	@Override
	public void outputTelemetry(double timestamp) {
		SmartDashboard.putNumber("launcher/SpeedTgt", periodicIO.outputValue);
		SmartDashboard.putNumber("launcher/HoodAngleTgt", periodicIO.hoodAngle);
		SmartDashboard.putNumber("launcher/rpm", periodicIO.actualRPM);
		SmartDashboard.putString("launcher/state", currentState.name());
		SmartDashboard.putString("launcher/wantedState", wantedState.name());
		SmartDashboard.putNumber("launcher/hood_position", periodicIO.currentHoodAngle);
		SmartDashboard.putNumber("launcher/hood_current", periodicIO.hoodStatorCurrent);
		SmartDashboard.putBoolean("launcher/secondBall", periodicIO.secondBall);
		SmartDashboard.putString("launcher/shotType", periodicIO.shotType.toString());
		SmartDashboard.putNumber("launcher/topRollerSpeedActual", periodicIO.topRollerSpeedActual);
		SmartDashboard.putNumber("launcher/topRollerSpeedTgt", periodicIO.topRollerSpeedTgt);
		SmartDashboard.putBoolean("launchCheck/mainRollerSpdInRange", Math.abs(periodicIO.actualRPM - periodicIO.outputValue) < RPM_ALLOWED_ERROR);
		SmartDashboard.putBoolean("launchCheck/topRollerSpdInRange", Math.abs(periodicIO.topRollerSpeedActual - periodicIO.topRollerSpeedTgt) < LAUNCHER_TOP_ROLLER_SPEED_TOLERANCE);
		SmartDashboard.putBoolean("launchCheck/hoodAngleInRange", Math.abs(periodicIO.hoodAngle - periodicIO.currentHoodAngle) <= HOOD_ERROR_TOLERANCE);
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public String getId() {
		return "Launcher";
	}

	public synchronized void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}

	public synchronized boolean readyToLaunch(){
		if (periodicIO.shotType.equals(ShotType.SHOOT_WHILE_MOVING)){
			return readyToLaunchDebounce.calculate(SystemState.RUN.equals(currentState));
		} else {
			return SystemState.RUN.equals(currentState);
		}
	}

    public void zeroHood(){
        currentState = SystemState.HOMING;
    }

	public synchronized void addHoodAngle(double value){
		var temp = periodicIO.hoodAngle + value;
		periodicIO.hoodAngle = Math.min(Math.max(58d, temp),80d);
	}

	public synchronized void addRpm(double value){
		var temp = periodicIO.outputValue + value;
		periodicIO.outputValue = Math.min(Math.max(0d, temp),6000d);
	}

	public boolean isSpinningUp(){
		return SystemState.SPINUP.equals(currentState);
	}

	public void setFenderShot(){
		periodicIO.shotType = ShotType.FENDER;
		periodicIO.hoodAngle = LAUNCHER_HOOD_ANGLE_FENDER;
		periodicIO.outputValue = LAUNCHER_SPEED_FENDER;
		//periodicIO.topRollerSpeedTgt = LAUNCHER_TOP_ROLLER_SPEED_FENDER;
		periodicIO.topRollerSpeedTgt = calculateTopRollerSpeed(periodicIO.outputValue, 0.8);
	}


	public void setManualOverride(boolean manualOverride){
		this.manualOverride = manualOverride;
	}

	public void setLimelightShot() {
		periodicIO.shotType = ShotType.LIMELIGHT;
		double distance;
		if (manualOverride){
			distance = CLOSE_LAUNCHPAD_SHOT_DISTANCE;
		} else {
			distance = periodicIO.limelightDistanceToTarget;
		}

		if (limelight.hasTargets()) {
			periodicIO.hoodAngle =  oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_HOOD_ANGLES, distance);
			periodicIO.outputValue = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_MOTOR_SPEEDS, distance);
			//periodicIO.topRollerSpeedTgt = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_LIMELIGHT_MOTOR_SPEEDS, distance);
			periodicIO.topRollerSpeedTgt = calculateTopRollerSpeed(periodicIO.outputValue, oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_BACKSPIN_GAIN_LL, distance));
		}
	}

	public void setMovingLimelightShot() {
		periodicIO.shotType = ShotType.SHOOT_WHILE_MOVING;
		double correctedDistance = highAngleLLDistanceCorrection() * periodicIO.limelightDistanceToTarget;

		double distanceOffset = shootWhileMovingDynamicAdjustment(correctedDistance);
		//TODO check if distance offset should go into aim distance calculation
		double distance = Math.max(shootWhileMovingAimDistance(correctedDistance - distanceOffset),
				LAUNCHER_LIMELIGHT_DISTANCE_IDX[0]);
		SmartDashboard.putNumber("launcher/shootWhileMovingDistance", distance);
		if (limelight.hasTargets()) {
			periodicIO.hoodAngle =  oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_HOOD_ANGLES, distance);
			periodicIO.outputValue = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_MOTOR_SPEEDS, distance);
			//periodicIO.topRollerSpeedTgt = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_LIMELIGHT_MOTOR_SPEEDS, distance);
			periodicIO.topRollerSpeedTgt = calculateTopRollerSpeed(periodicIO.outputValue, oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_BACKSPIN_GAIN_LL, distance));
		}
	}

	public void setFenderShotLow(){
		periodicIO.shotType = ShotType.FENDER_LOW;
		periodicIO.hoodAngle = LAUNCHER_HOOD_ANGLE_FENDER_LOW_GOAL;
		periodicIO.outputValue = LAUNCHER_SPEED_FENDER_LOW;
		//periodicIO.topRollerSpeedTgt = LAUNCHER_TOP_ROLLER_SPEED_FENDER_LOW;
		periodicIO.topRollerSpeedTgt = calculateTopRollerSpeed(periodicIO.outputValue, 0.6);
	}
	public void setAutonShot(double distance){
		periodicIO.shotType = ShotType.AUTON;
		periodicIO.hoodAngle = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_HOOD_ANGLES, distance);
		periodicIO.outputValue = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_MOTOR_SPEEDS, distance);
		//periodicIO.topRollerSpeedTgt = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_LIMELIGHT_MOTOR_SPEEDS, distance);
		periodicIO.topRollerSpeedTgt = calculateTopRollerSpeed(periodicIO.outputValue, oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_BACKSPIN_GAIN_LL, distance));
	}

	public void setTarmacLineShot(){
		periodicIO.shotType = ShotType.TARMAC_LINE;
		periodicIO.hoodAngle = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_HOOD_ANGLES, LAUNCHER_TARMAC_LINE_SHOT_DISTANCE);
		periodicIO.outputValue = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_LIMELIGHT_MOTOR_SPEEDS, LAUNCHER_TARMAC_LINE_SHOT_DISTANCE);
		//periodicIO.topRollerSpeedTgt = oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_LIMELIGHT_MOTOR_SPEEDS, LAUNCHER_TARMAC_LINE_SHOT_DISTANCE);
		periodicIO.topRollerSpeedTgt = calculateTopRollerSpeed(periodicIO.outputValue, oneDimensionalLookup.interpLinear(LAUNCHER_LIMELIGHT_DISTANCE_IDX, LAUNCHER_TOP_ROLLER_BACKSPIN_GAIN_LL, LAUNCHER_TARMAC_LINE_SHOT_DISTANCE));
	}

	public void setOppositeColorShot(){
		periodicIO.shotType = ShotType.OPPOSITE_COLOR;
		periodicIO.hoodAngle = LAUNCHER_HOOD_ANGLE_OPPOSITE_COLOR;
		periodicIO.outputValue = LAUNCHER_SPEED_OPPOSITE_COLOR;
		periodicIO.topRollerSpeedTgt = LAUNCHER_TOP_ROLLER_SPEED_OPPOSITE_COLOR;
	}

	public void setOppositeColorFarEject(){
		periodicIO.shotType = ShotType.OPPOSITE_COLOR;
		periodicIO.hoodAngle = LAUNCHER_HOOD_ANGLE_OPPOSITE_COLOR_FAR_EJECT;
		periodicIO.outputValue = LAUNCHER_SPEED_OPPOSITE_COLOR_FAR_EJECT;
		periodicIO.topRollerSpeedTgt = LAUNCHER_TOP_ROLLER_SPEED_OPPOSITE_COLOR_FAR_EJECT;
	}

	private void checkFenderSecondBall(){
		if(cargoSensorRearDebounce.calculate(periodicIO.cargoSensorRear)) {
			periodicIO.secondBall = true;
		}
		if(periodicIO.shotType == ShotType.FENDER) {
			if (periodicIO.secondBall) {
				periodicIO.outputValue = LAUNCHER_SPEED_FENDER_SHOT_TWO;
			} else {
				periodicIO.outputValue = LAUNCHER_SPEED_FENDER;
			}
		}
	}

	private double calculateTopRollerSpeed(double mainRollerSpeedTarget, double ratio){
		return Math.min(10000, mainRollerSpeedTarget * 4 / 1.5 * ratio);
	}

	private double shootWhileMovingDynamicAdjustment(double distance){
		double[] chassisSpeeds = ModuleToChassisSpeeds.getChassisSpeeds();
		double timeOfFlight = oneDimensionalLookup.interpLinear(LauncherCals.LAUNCHER_LIMELIGHT_DISTANCE_IDX,
				LauncherCals.LAUNCHER_LIMELIGHT_TIME_OF_FLIGHT_LONGITUDINAL, distance);
		return chassisSpeeds[2] * timeOfFlight;
	}

	public void setShootWhileMovingLocked(boolean locked){
		shootWhileMovingLocked = locked;
	}

	public boolean checkShootWhileMovingLocked(){
		if (periodicIO.shotType.equals(ShotType.SHOOT_WHILE_MOVING) ||
			periodicIO.shotType.equals(ShotType.LIMELIGHT)){
			return shootWhileMovingLocked;
		} else {
			return false;
		}
	}

	private double shootWhileMovingAimDistance(double distance){
		double[] chassisSpeeds = ModuleToChassisSpeeds.getChassisSpeeds();
		double timeOfFlight = oneDimensionalLookup.interpLinear(LauncherCals.LAUNCHER_LIMELIGHT_DISTANCE_IDX,
				LauncherCals.LAUNCHER_LIMELIGHT_TIME_OF_FLIGHT, distance);
		return Math.sqrt(Math.pow(distance, 2) + Math.pow(chassisSpeeds[3] * timeOfFlight, 2));
	}

	private double highAngleLLDistanceCorrection(){
		return oneDimensionalLookup.interpLinear(LAUNCHER_SHOOT_WHILE_MOVE_DISTANCE_CORRECTION_LL_ANGLE,
				LAUNCHER_SHOOT_WHILE_MOVE_DISTANCE_CORRECTION_GAIN, Math.abs(periodicIO.limelightAngleError));
	}
}
