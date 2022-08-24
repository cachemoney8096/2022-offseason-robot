package net.teamrush27.frc2022.calibrations;

public class DrivetrainCals {
    public static final double TELEOP_YAW_FB_CTRL_KP = 0.1;
    public static final double TELEOP_YAW_FB_CTRL_KI = 0.0;
    public static final double TELEOP_YAW_FB_CTRL_KD = 0.0;

    public static final double GOAL_TRACKING_KP = 0.15; //0.15
    public static final double GOAL_TRACKING_KI = 0.02; //0.05
    public static final double GOAL_TRACKING_KD = 0.01;
    public static final double GOAL_TRACKING_IMAX = 0.1;  //0.3
    public static final double GOAL_TRACKING_ANGLE_ERROR_FC = 5.0;

    public static final double MOVE_AND_SHOOT_MAX_SPEED = 1.5;
    public static final double MOVE_AND_SHOOT_MAX_ACCEL = 10.0;

    public static final double GOAL_TRACKING_CENTER_DISTANCE_OFFSET = 0.0; //0.2 meters

    public static final double DRIVETRAIN_IDLE_VXVY_THRESHOLD = 0.01;
    public static final double DRIVETRAIN_IDLE_WZ_THRESHOLD = 0.01;

    public static final double DRIVETRAIN_MAX_YAW_RATE = 3 * Math.PI;

    public static final double SPEED_ADJ_FACTOR = 0.978;

    public static final double[] XY_Axis_inputBreakpoints = {-1, -0.85, -0.6, -0.12, 0.12, 0.6, 0.85, 1};
    public static final double[] XY_Axis_outputTable = {-1.0, -0.6, -0.3, 0, 0, 0.3, 0.6, 1.0};
    public static final double[] RotAxis_inputBreakpoints = {-1, -0.9, -0.6, -0.12, 0.12, 0.6, 0.9, 1};
    public static final double[] RotAxis_outputTable = {-1.0, -0.5, -0.2, 0, 0, 0.2, 0.5, 1.0};

    public static final double GOAL_TRACKING_ANGLE_TOLERANCE = 2.0;
    public static final double GOAL_TRACKING_MAX_LATERAL_ERROR_AT_GOAL = 0.25; //meters

}
