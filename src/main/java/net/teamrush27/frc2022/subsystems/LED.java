package net.cachemoney8096.frc2022o.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.RobotContainer;

public class LED implements Subsystem {

    private final CANdle candle;

    public LED(){
        candle = new CANdle(Constants.CANDLE_CAN_ID, "Drivetrain");
        candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 50);
        currentStateStartTime = Timer.getFPGATimestamp();
    }

    private enum SystemState {
        DISABLED,
        ENABLED,
        LL_LOCKING,
        LL_LOCKED,
        SPEED_UP,
        READY_TO_FIRE,
        ONE_BALL,
        TWO_BALL
    }

    public enum WantedState {
        IDLE,
        LL_LOCKING,
        LL_LOCKED,
        SPEED_UP,
        READY_TO_FIRE,
        ONE_BALL,
        TWO_BALL
    }

    private SystemState currentState = SystemState.DISABLED;
    private WantedState wantedState = WantedState.IDLE;

    private double currentStateStartTime;

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch (currentState) {
            case DISABLED:
                candle.animate(new SingleFadeAnimation(1,61,126, 0, .3, 72));
                break;
            case ENABLED:
                //candle.animate(new SingleFadeAnimation(90,25,0, 0, .45, 72));
                candle.clearAnimation(0);
                candle.setLEDs(100, 0, 0, 0, 0, 72);
                break;
            case LL_LOCKING:
                candle.animate(new StrobeAnimation(90,15,0, 0, .01, 72));
                break;
            case LL_LOCKED:
                candle.animate(new StrobeAnimation(0,50,0, 0, .01, 72));
                break;
            case SPEED_UP:
                candle.animate(new StrobeAnimation(0,0,50, 0, .01, 72));
                break;
            case READY_TO_FIRE:
                candle.animate(new RainbowAnimation(0.1, 1, 72));
                break;
            case ONE_BALL:
                /*
                if(timestamp - currentStateStartTime < .5 ){
                    candle.animate(new StrobeAnimation(1,61,126, 0, .2, 72));
                } else {
                    candle.animate(new SingleFadeAnimation(90, 25, 0, 0, .45, 72));
                }*/
                candle.clearAnimation(0);
                candle.setLEDs(90, 30, 0, 0, 0, 72);
                break;
            case TWO_BALL:
                /*
                if(timestamp - currentStateStartTime < .5 ){
                    candle.animate(new StrobeAnimation(0,100,0, 0, .2, 72));
                } else {
                    candle.animate(new SingleFadeAnimation(90, 25, 0, 0, .45, 72));
                }*/
                candle.clearAnimation(0);
                candle.setLEDs(0, 80, 0, 0, 0, 72);
                break;
            default:
                break;
        }
        newState = defaultStateChange();
        if (newState != currentState) {
            currentState = newState;
            currentStateStartTime = timestamp;
        }
    }

    private SystemState defaultStateChange(){
        switch (wantedState){
            default:
            case IDLE:
                if(!RobotContainer.EnableState.DISABLED.equals(RobotContainer.getInstance().enableState)){
                    return SystemState.ENABLED;
                }
                return SystemState.DISABLED;
            case LL_LOCKING:
                return SystemState.LL_LOCKING;
            case LL_LOCKED:
                return SystemState.LL_LOCKED;
            case SPEED_UP:
                return SystemState.SPEED_UP;
            case READY_TO_FIRE:
                return SystemState.READY_TO_FIRE;
            case ONE_BALL:
                return SystemState.ONE_BALL;
            case TWO_BALL:
                return SystemState.TWO_BALL;
        }
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putString("LED/systemState", currentState.name());
        SmartDashboard.putString("LED/wantedState", wantedState.name());
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public String getId() {
        return "LED";
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }
}
