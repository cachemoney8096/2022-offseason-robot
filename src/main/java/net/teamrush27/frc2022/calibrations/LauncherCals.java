package net.cachemoney8096.frc2022o.calibrations;

public class LauncherCals {
    public static final double LAUNCHER_KF = 0.0454;
    public static final double LAUNCHER_KP = 0.2;  //0.2
    public static final double LAUNCHER_KI = 0.001;
    public static final double LAUNCHER_KD = 10.0;  //10.5
    public static final double LAUNCHER_IZONE = 500; //units/100ms

    public static final double LAUNCHER_TOP_ROLLER_KF = 0.000085;
    public static final double LAUNCHER_TOP_ROLLER_KP = 0.00003;
    public static final double LAUNCHER_TOP_ROLLER_KI = 0.0;
    public static final double LAUNCHER_TOP_ROLLER_KD = 0.0004;
    public static final double LAUNCHER_TOP_ROLLER_IZONE = 500.0;

    public static final double LAUNCHER_SPEED_TOLERANCE = 50.0;
    public static final double LAUNCHER_TOP_ROLLER_SPEED_TOLERANCE = 1000.0;

    public static final double HOOD_KF = 0;
    public static final double HOOD_KP = 0.2;  //0.1
    public static final double HOOD_KI = 0.002;
    public static final double HOOD_KD = 0.0;  //4.0
    public static final double HOOD_ERROR_TOLERANCE = 0.25;

    public static final double LAUNCHER_SPEED_FENDER = 2450.0;//2100
    public static final double LAUNCHER_SPEED_FENDER_SHOT_TWO = LAUNCHER_SPEED_FENDER;
    public static final double LAUNCHER_TOP_ROLLER_SPEED_FENDER = 8000.0;
    public static final double LAUNCHER_SECOND_BALL_FALL_DEBOUNCE = 0.1;
    public static final double LAUNCHER_SPEED_TARMAC = 3000.0;
    public static final double LAUNCHER_SPEED_CLOSE_LAUNCHPAD = 4200.0;
    public static final double LAUNCHER_SPEED_FAR_LAUNCHPAD = 4600.0;
    public static final double LAUNCHER_SPEED_FENDER_LOW = 1300.0;
    public static final double LAUNCHER_TOP_ROLLER_SPEED_FENDER_LOW = LAUNCHER_SPEED_FENDER_LOW * 4.0 / 1.5;
    //public static final double LAUNCHER_TOP_ROLLER_BACKSPIN_GAIN_LL = 0.6;
    public static final double LAUNCHER_SPEED_OPPOSITE_COLOR = 800.0;
    public static final double LAUNCHER_TOP_ROLLER_SPEED_OPPOSITE_COLOR = 6500.0;
    public static final double LAUNCHER_SPEED_OPPOSITE_COLOR_FAR_EJECT = 800.0;
    public static final double LAUNCHER_TOP_ROLLER_SPEED_OPPOSITE_COLOR_FAR_EJECT = 10000.0;

    public static final double LAUNCHER_HOOD_ANGLE_FENDER = 74.0;
    public static final double LAUNCHER_HOOD_ANGLE_FENDER_LOW_GOAL = 67.0;
    public static final double LAUNCHER_HOOD_ANGLE_OPPOSITE_COLOR = 79.5;
    public static final double LAUNCHER_HOOD_ANGLE_OPPOSITE_COLOR_FAR_EJECT = 63.0;

    public static final double HOOD_MANUAL_GAIN = 0.1;
    public static final double HOOD_MAX_TRAVEL = 100000; //SENSOR UNITS


public static final double[] LAUNCHER_LIMELIGHT_DISTANCE_IDX =      {1.15,  2,      2.5,    3,      4,      5,      5.2,    6};
    public static final double[] LAUNCHER_LIMELIGHT_HOOD_ANGLES =   {73,    67,     65,     63,     60,     60,     60,     60};
    public static final double[] LAUNCHER_LIMELIGHT_MOTOR_SPEEDS =  {2600,  2625,   2800,   3150,   3700,   4100,   4750,   5300};//{3500, 3700, 4100, 4500, 5300};
    public static final double[] LAUNCHER_TOP_ROLLER_BACKSPIN_GAIN_LL = {0.7, 0.7, 0.7, 0.7, 0.8, 1.0, 1.0, 1.0};
    public static final double TOF_SCALER = 0.6;
    public static final double[] LAUNCHER_LIMELIGHT_TIME_OF_FLIGHT = {1.07*TOF_SCALER, 1.08*TOF_SCALER, 1.1*TOF_SCALER, 1.06*TOF_SCALER, 1.23*TOF_SCALER, 1.36*TOF_SCALER, 1.39*TOF_SCALER, 1.48*TOF_SCALER};
    public static final double TOF_SCALER_LONGITUDINAL = 1.0;
    public static final double[] LAUNCHER_LIMELIGHT_TIME_OF_FLIGHT_LONGITUDINAL = {
            1.07    *   TOF_SCALER_LONGITUDINAL,
            1.08    *   TOF_SCALER_LONGITUDINAL,
            1.1     *   TOF_SCALER_LONGITUDINAL,
            1.06    *   TOF_SCALER_LONGITUDINAL,
            1.23    *   TOF_SCALER_LONGITUDINAL,
            1.36    *   TOF_SCALER_LONGITUDINAL,
            1.39    *   TOF_SCALER_LONGITUDINAL,
            1.48    *   TOF_SCALER_LONGITUDINAL};
    public static final double LAUNCHER_TARMAC_LINE_SHOT_DISTANCE = 2.0;
    public static final double CLOSE_LAUNCHPAD_SHOT_DISTANCE = 4.2;

    public static final double[] LAUNCHER_SHOOT_WHILE_MOVE_DISTANCE_CORRECTION_LL_ANGLE = {0, 10, 15, 20, 25, 30};
    public static final double[] LAUNCHER_SHOOT_WHILE_MOVE_DISTANCE_CORRECTION_GAIN = {1.0, 1.03, 1.07, 1.11, 1.16, 1.26};

}
