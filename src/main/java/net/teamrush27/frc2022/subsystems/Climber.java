package net.cachemoney8096.frc2022o.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import static net.cachemoney8096.frc2022o.Constants.*;
import static net.cachemoney8096.frc2022o.calibrations.ClimberCals.*;
import static net.cachemoney8096.frc2022o.calibrations.ClimberCals.climberTicksToDegrees;

public class Climber implements Subsystem{

    private final Solenoid elevatorRelease;
    private final Solenoid hook1Open;
    private final Solenoid hook1Close;
	private final Solenoid hook2Open;
    private final Solenoid hook2Close;

    //private final DigitalInput hookADetect;
	//private final DigitalInput hookBDetect;

    private final TalonFX leaderMotor, followerMotor;

    public Climber() {
        leaderMotor = new TalonFX(CLIMBER_MOTOR_LEADER_CANID);
        followerMotor = new TalonFX(CLIMBER_MOTOR_FOLLOWER_CANID);
        //TODO Add 2 more channels for double acting
        elevatorRelease = new Solenoid(PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, CLIMBER_ELEVATOR_RELEASE_ID);
		hook2Open = new Solenoid(PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, CLIMBER_HOOK_REAR_OPEN);
        hook2Close = new Solenoid(PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, CLIMBER_HOOK_REAR_CLOSE);
        hook1Open = new Solenoid(PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, CLIMBER_HOOK_FRONT_OPEN);
        hook1Close = new Solenoid(PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, CLIMBER_HOOK_FRONT_CLOSE);



		//hookADetect = new DigitalInput(0);
		//hookBDetect = new DigitalInput(1);

        leaderMotor.configAllSettings(new TalonFXConfiguration());
        leaderMotor.setInverted(true);
		leaderMotor.setNeutralMode(NeutralMode.Brake);
        leaderMotor.config_kP(0, CLIMBER_MOTOR_KP);
        leaderMotor.config_kI(0, CLIMBER_MOTOR_KI);
        leaderMotor.config_kD(0, CLIMBER_MOTOR_KD);
        leaderMotor.config_IntegralZone(0, CLIMBER_MOTOR_IZONE);
        leaderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 60, 5.0));
        leaderMotor.configMotionCruiseVelocity(CLIMB_CRUISE_VELOCITY);
        leaderMotor.configMotionAcceleration(CLIMB_ACCELERATION);

        followerMotor.configAllSettings(new TalonFXConfiguration());
		followerMotor.setNeutralMode(NeutralMode.Brake);
        followerMotor.config_kP(0, CLIMBER_MOTOR_KP);
        followerMotor.config_kI(0, CLIMBER_MOTOR_KI);
        followerMotor.config_kD(0, CLIMBER_MOTOR_KD);
        followerMotor.config_IntegralZone(0, CLIMBER_MOTOR_IZONE);
        followerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 60, 5.0));
        followerMotor.configMotionCruiseVelocity(CLIMB_CRUISE_VELOCITY);
        followerMotor.configMotionAcceleration(CLIMB_ACCELERATION);
        followerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
        followerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);

        followerMotor.follow(leaderMotor);
        followerMotor.setInverted(InvertType.OpposeMaster);

        leaderMotor.setSelectedSensorPosition(0.0, 0, 250);
	}

    private enum SystemState {
        IDLE,
        DEPLOY,
        LVL2_READY,
        LVL2_LATCH,
        LVL23_ROTATE,
        LVL23_SETTLE,
        LVL3_LATCH,
        LVL2_RELEASE,
        LVL34_ROTATE,
        LVL34_SETTLE,
        LVL4_LATCH,
        LVL3_RELEASE,
        LVL3_RELAX,
        LVL4_RELAX
	}

    public enum WantedState {
        IDLE,
        DEPLOY,
        LEVEL_3,
        LEVEL_4
    }


	private static class PeriodicIO {
		// INPUTS
		double timestamp;

		double leaderMotorAngle;

        boolean hookASwitch;
        boolean hookBSwitch;

        double leaderOutputPercent;
        double leaderOutputCurrent;

        double followerOutputPercent;
        double followerOutputCurrent;

		// OUTPUTS
		double outputAngleTarget;
        boolean hook1Open;
        boolean hook2Open;
        boolean elevatorRelease;

	}

	private SystemState currentState = SystemState.IDLE;
    private WantedState wantedState = WantedState.IDLE;

    private final PeriodicIO periodicIO = new PeriodicIO();

    private double currentStateStartTime;
    private boolean isNewState = true;

    private boolean advanceState = false;
    private boolean advanceStatePrev = false;

    @Override
    public void readPeriodicInputs(double timestamp) {
        periodicIO.timestamp = timestamp;
        periodicIO.leaderMotorAngle = leaderMotor.getSelectedSensorPosition();
        //periodicIO.hookASwitch = !hookADetect.get();
        //periodicIO.hookBSwitch = !hookBDetect.get();

        periodicIO.leaderOutputPercent = leaderMotor.getMotorOutputPercent();
        periodicIO.leaderOutputCurrent = leaderMotor.getSupplyCurrent();
        periodicIO.followerOutputPercent = followerMotor.getMotorOutputPercent();
        periodicIO.followerOutputCurrent = followerMotor.getSupplyCurrent();
	}

    @Override
    public void processLoop(double timestamp){
        SystemState newState = SystemState.IDLE;
        switch(currentState){
            case IDLE:
                newState = handleIdle();
                break;
            case DEPLOY:
                //Release elevator
                newState = handleDeploy(timestamp);
                break;
            case LVL2_READY:
                newState = handleLevel2Ready();
                break;
            case LVL2_LATCH:
                newState = handleLevel2Latch(timestamp);
                break;
            case LVL23_ROTATE:
                newState = handleLevel2To3Rotate(timestamp);
                break;
            case LVL23_SETTLE:
                newState = handleLevel2To3Settle(timestamp);
                break;
            case LVL3_LATCH:
                newState = handleLevel3Latch(timestamp);
                break;
            case LVL2_RELEASE:
                newState = handleLevel2Release(timestamp);
                break;
            case LVL3_RELAX:
                newState = handleLevel3Relax(timestamp);
                break;
            case LVL34_ROTATE:
                newState = handleLevel3To4Rotate(timestamp);
                break;
            case LVL34_SETTLE:
                newState = handleLevel3To4Settle(timestamp);
                break;
            case LVL4_LATCH:
                newState = handleLevel4Latch(timestamp);
                break;
            case LVL3_RELEASE:
                newState = handleLevel3Release(timestamp);
                break;
            case LVL4_RELAX:
                newState = handleLevel4Relax(timestamp);
                break;
            
        }
        if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
            advanceState = false;
            isNewState = true;
		} else{
            isNewState = false;
        }
        advanceStatePrev = advanceState;

    }

    @Override
	public void writePeriodicOutputs(double timestamp) {
        if(isNewState){
            switch(currentState) {
                case LVL3_LATCH:
                case LVL4_LATCH:
                    leaderMotor.setNeutralMode(NeutralMode.Brake);
                    followerMotor.setNeutralMode(NeutralMode.Brake);
                    leaderMotor.configMotionCruiseVelocity(CLIMB_CRUISE_VELOCITY_SETTLE);
                    leaderMotor.configMotionAcceleration(CLIMB_ACCELERATION_SETTLE);
                    followerMotor.configMotionCruiseVelocity(CLIMB_CRUISE_VELOCITY_SETTLE);
                    followerMotor.configMotionAcceleration(CLIMB_ACCELERATION_SETTLE);
                    break;
                case LVL34_SETTLE:
                case LVL23_SETTLE:
                    leaderMotor.setNeutralMode(NeutralMode.Coast);
                    followerMotor.setNeutralMode(NeutralMode.Coast);
                    break;
                case LVL3_RELAX:
                case LVL4_RELAX:
                default:
                    leaderMotor.setNeutralMode(NeutralMode.Brake);
                    followerMotor.setNeutralMode(NeutralMode.Brake);
                    leaderMotor.configMotionCruiseVelocity(CLIMB_CRUISE_VELOCITY);
                    leaderMotor.configMotionAcceleration(CLIMB_ACCELERATION);
                    followerMotor.configMotionCruiseVelocity(CLIMB_CRUISE_VELOCITY);
                    followerMotor.configMotionAcceleration(CLIMB_ACCELERATION);
                break;
            }
        }

        switch(currentState){
            case LVL3_LATCH:
            case LVL4_LATCH:
                break;
            case LVL34_SETTLE:
            case LVL23_SETTLE:
                leaderMotor.neutralOutput();
            break;
            case LVL3_RELAX:
            case LVL4_RELAX:
            default:
                leaderMotor.set(TalonFXControlMode.MotionMagic, periodicIO.outputAngleTarget);
                break;
        }
            elevatorRelease.set(periodicIO.elevatorRelease);
            hook1Open.set(periodicIO.hook1Open);
            hook1Close.set(!periodicIO.hook1Open);
            hook2Open.set(periodicIO.hook2Open);
            hook2Close.set(!periodicIO.hook2Open);

	}


    @Override
	public void stop(){
//        leaderMotor.setNeutralMode(NeutralMode.Brake);
//        followerMotor.setNeutralMode(NeutralMode.Brake);
        //leaderMotor.neutralOutput();
    }

    @Override
	public boolean checkSystem(){
        return true;
    }

    @Override
	public void zeroSensors(){
        leaderMotor.setSelectedSensorPosition(0);
    }

    @Override
	public String getId(){
        return "Climber";
    }

    @Override
	public void outputTelemetry(double timestamp){
        SmartDashboard.putString("climb/state",currentState.name());
        SmartDashboard.putNumber("climb/angle_target",climberTicksToDegrees(periodicIO.outputAngleTarget));
        SmartDashboard.putNumber("climb/angle_current",climberTicksToDegrees(periodicIO.leaderMotorAngle));
        SmartDashboard.putNumber("climb/leaderOutputPercent",periodicIO.leaderOutputPercent);
        SmartDashboard.putNumber("climb/leaderCurrent",periodicIO.leaderOutputCurrent);
        SmartDashboard.putNumber("climb/followerOutputPercent",periodicIO.followerOutputPercent);
        SmartDashboard.putNumber("climb/followerCurrent",periodicIO.followerOutputCurrent);
	}

    public void advanceStateManual(){
        advanceState = true;
    }

    private SystemState handleIdle(){
        periodicIO.outputAngleTarget = IDLE_ANGLE;
        periodicIO.hook1Open = true;
        periodicIO.hook2Open = true;
        periodicIO.elevatorRelease = false;

        switch (wantedState){
            default:
            case IDLE:
                return SystemState.IDLE;
            case DEPLOY:
            case LEVEL_3:
            case LEVEL_4:
                return SystemState.DEPLOY;
        }
    }


    private SystemState handleDeploy(double timestamp){
        periodicIO.outputAngleTarget = periodicIO.leaderMotorAngle;
        periodicIO.hook1Open = false;
        periodicIO.hook2Open = false;
        periodicIO.elevatorRelease = true;

        if(timestamp - currentStateStartTime > 1d
            && !WantedState.DEPLOY.equals(wantedState)){
            return SystemState.LVL2_READY;
        }

        return SystemState.DEPLOY;
    }

    private SystemState handleLevel2Ready(){
        periodicIO.outputAngleTarget = LVL2_READY_ANGLE;
        //periodicIO.elevatorRelease = false;

        if(advanceState){
            //return SystemState.LVL2_LATCH;
            // NO NEED WITH 118 HOOKS
            return SystemState.LVL23_ROTATE;
        }
        return SystemState.LVL2_READY;
    }

    private SystemState handleLevel2Latch(double timestamp){
        periodicIO.hook1Open = false;

        if(timestamp - currentStateStartTime > .25d && advanceState){
            return SystemState.LVL23_ROTATE;
        }

        return SystemState.LVL2_LATCH;
    }

    private SystemState handleLevel2To3Rotate(double timestamp){
        periodicIO.outputAngleTarget = LVL23_ROTATE_ANGLE;
        periodicIO.hook2Open = false;
        periodicIO.elevatorRelease = false;

        if(timestamp - currentStateStartTime > .25
                && climberTicksToDegrees(Math.abs(periodicIO.leaderMotorAngle-periodicIO.outputAngleTarget)) < 1d){
            // NOT NEEDED WITH 118 HOOKS ... maybe
            return SystemState.LVL23_SETTLE;
//            return SystemState.LVL3_LATCH;
        }

        return SystemState.LVL23_ROTATE;
    }

    private SystemState handleLevel2To3Settle(double timestamp){
        periodicIO.outputAngleTarget = LVL23_SETTLE_ANGLE;

        if(timestamp - currentStateStartTime > .25
                && periodicIO.leaderMotorAngle<periodicIO.outputAngleTarget){
            return SystemState.LVL3_LATCH;
        }

        return SystemState.LVL23_SETTLE;
    }

    private SystemState handleLevel3Latch(double timestamp){
        periodicIO.hook1Open = false;
        periodicIO.hook2Open = false;

        if(timestamp - currentStateStartTime > .25d
//                && climberTicksToDegrees(Math.abs(periodicIO.leaderMotorAngle-periodicIO.outputAngleTarget)) < 1d
                && advanceState){
            return SystemState.LVL2_RELEASE;
        }

        return SystemState.LVL3_LATCH;
    }

    private SystemState handleLevel2Release(double timestamp){
        periodicIO.hook1Open = true;
        periodicIO.outputAngleTarget = LVL2_RELEASE_ANGLE;

        if(timestamp - currentStateStartTime > 0.5d){
//            if(WantedState.LEVEL_3.equals(wantedState)){
                return SystemState.LVL3_RELAX;
//            }
//            return SystemState.LVL34_ROTATE;
        }

        return SystemState.LVL2_RELEASE;
    }

    private SystemState handleLevel3Relax(double timestamp) {
        periodicIO.outputAngleTarget = LVL3_RELAX_ANGLE;

        if(timestamp - currentStateStartTime > .25
                && climberTicksToDegrees(Math.abs(periodicIO.leaderMotorAngle-periodicIO.outputAngleTarget)) < 5d) {
            leaderMotor.setNeutralMode(NeutralMode.Coast);
            followerMotor.setNeutralMode(NeutralMode.Coast);
            if (WantedState.LEVEL_4.equals(wantedState) && timestamp - currentStateStartTime > 1.12) {
                return SystemState.LVL34_ROTATE;
            }
//            if (WantedState.LEVEL_4.equals(wantedState) && advanceState) {
//                return SystemState.LVL34_ROTATE;
//            }
        }


        return SystemState.LVL3_RELAX;
    }

    private SystemState handleLevel3To4Rotate(double timestamp){
        periodicIO.outputAngleTarget = LVL34_ROTATE_ANGLE;

        if(periodicIO.leaderMotorAngle>LVL3_RELAX_ANGLE){
            periodicIO.hook1Open = false;
        }

        if(timestamp - currentStateStartTime > .25
                && climberTicksToDegrees(Math.abs(periodicIO.leaderMotorAngle-periodicIO.outputAngleTarget)) < 1d){
            // NO NEED WITH 118 HOOKS ... maybe ...
            return SystemState.LVL34_SETTLE;
//            return SystemState.LVL4_LATCH;
        }

        return SystemState.LVL34_ROTATE;
    }

    private SystemState handleLevel3To4Settle(double timestamp){
        periodicIO.outputAngleTarget = LVL34_SETTLE_ANGLE;

        if(timestamp - currentStateStartTime > .25
                && climberTicksToDegrees(Math.abs(periodicIO.leaderMotorAngle-periodicIO.outputAngleTarget)) < 5d){
            return SystemState.LVL4_LATCH;
        }

        return SystemState.LVL34_SETTLE;
    }

    private SystemState handleLevel4Latch(double timestamp){
        periodicIO.hook1Open = false;

        if(timestamp - currentStateStartTime > .25d
//                && climberTicksToDegrees(Math.abs(periodicIO.leaderMotorAngle-periodicIO.outputAngleTarget)) < 1d
                && advanceState){
            return SystemState.LVL3_RELEASE;
        }

        return SystemState.LVL4_LATCH;
    }

    private SystemState handleLevel3Release(double timestamp){
        periodicIO.hook2Open = true;
        periodicIO.outputAngleTarget = LVL3_RELEASE_ANGLE;

        if(timestamp - currentStateStartTime > 0.5d){
            return SystemState.LVL4_RELAX;
        }

        return SystemState.LVL3_RELEASE;
    }

    private SystemState handleLevel4Relax(double timestamp) {
        periodicIO.outputAngleTarget = LVL4_RELAX_ANGLE;

        if(timestamp - currentStateStartTime > .25
                && climberTicksToDegrees(Math.abs(periodicIO.leaderMotorAngle-periodicIO.outputAngleTarget)) < 1d) {
            leaderMotor.setNeutralMode(NeutralMode.Coast);
            followerMotor.setNeutralMode(NeutralMode.Coast);
        }

        return SystemState.LVL4_RELAX;
    }

    public void setWantedState(WantedState wantedState) {

        this.wantedState = wantedState;
    }
}
