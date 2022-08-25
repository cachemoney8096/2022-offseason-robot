package net.cachemoney8096.frc2022o.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.cachemoney8096.frc2022o.FeatureEnables;
import net.cachemoney8096.frc2022o.sensors.PicoColorSensor;

public class ColorSensor implements Subsystem{

    private static class PeriodicIO {
        int colorSensorRed;
        int colorSensorGreen;
        int colorSensorBlue;
    }

    private final PicoColorSensor picoColorSensor;
    private final PeriodicIO periodicIO = new PeriodicIO();
    private final Debouncer redDebounce = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private final Debouncer blueDebounce = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    public ColorSensor() {
        this.picoColorSensor = new PicoColorSensor();
    }

    public enum detectedBallColor {
        UNKNOWN,
        RED,
        BLUE
    }

    public enum indexedBallState{
        SAME_COLOR,
        OPPOSITE_COLOR,
        UNKNOWN
    }

    private detectedBallColor ballColorState = detectedBallColor.UNKNOWN;

    @Override
    public void readPeriodicInputs(double timestamp) {
        PicoColorSensor.RawColor picoRawColor = picoColorSensor.getRawColor0();
        periodicIO.colorSensorRed = picoRawColor.red;
        periodicIO.colorSensorGreen = picoRawColor.green;
        periodicIO.colorSensorBlue = picoRawColor.blue;

    }

    @Override
    public void processLoop(double timestamp) {
        detectedBallColor newState = detectedBallColor.UNKNOWN;
        switch(ballColorState) {
            case UNKNOWN:
            case RED:
            case BLUE:
                newState = determineBallColor();
                break;
        }

        if (newState != ballColorState) {
            ballColorState = newState;
            //currentStateStartTime = timestamp;
        }
    }

    private detectedBallColor determineBallColor(){
        if (checkRed()){
            return detectedBallColor.RED;
        } else if (checkBlue()){
            return detectedBallColor.BLUE;
        } else {
            return detectedBallColor.UNKNOWN;
        }
    }

    private boolean checkRed(){
        return redDebounce.calculate(periodicIO.colorSensorRed > (periodicIO.colorSensorBlue * 1.5)
                && periodicIO.colorSensorGreen > 300);
    }

    private boolean checkBlue(){
        return blueDebounce.calculate(periodicIO.colorSensorBlue > (periodicIO.colorSensorRed * 1.5)
                && periodicIO.colorSensorGreen > 300);
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {

    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("colorSensor/redValue", periodicIO.colorSensorRed);
        SmartDashboard.putNumber("colorSensor/greenValue", periodicIO.colorSensorGreen);
        SmartDashboard.putNumber("colorSensor/blueValue", periodicIO.colorSensorBlue);
        SmartDashboard.putString("colorSensor/detectedBallColor", ballColorState.toString());
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
        return "ColorSensor";
    }

    public detectedBallColor getBallColorState(){
        return ballColorState;
    }

    public indexedBallState determineIndexedBallColorState(){
        if (FeatureEnables.COLOR_SENSOR_ENABLE) {
            if ((DriverStation.getAlliance().equals(DriverStation.Alliance.Red)
                    && getBallColorState().equals(ColorSensor.detectedBallColor.BLUE))
                    || (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
                    && getBallColorState().equals(ColorSensor.detectedBallColor.RED))) {
                return indexedBallState.OPPOSITE_COLOR;
            } else if ((DriverStation.getAlliance().equals(DriverStation.Alliance.Red)
                    && getBallColorState().equals(ColorSensor.detectedBallColor.RED))
                    || (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
                    && getBallColorState().equals(detectedBallColor.BLUE))) {
                return indexedBallState.SAME_COLOR;
            } else {
                return indexedBallState.UNKNOWN;
            }
        } else {
            return indexedBallState.UNKNOWN;
        }
    }
}
