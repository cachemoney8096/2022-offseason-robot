package net.cachemoney8096.frc2022o.calibrations;

public class HopperCals {
    public static final double REAR_HOPPER_F500_KF = 0.065;
    public static final double REAR_HOPPER_F500_KP = 0.05;
    public static final double REAR_HOPPER_F500_KI = 0.0;
    public static final double REAR_HOPPER_F500_KD = 1.0;

    public static final double INDEX_KF = 0.000085;
    public static final double INDEX_KP = 0.00005;
    public static final double INDEX_KI = 0;
    public static final double INDEX_KD = 0.00025; //0.00025

    //public static final double HOPPER_INDEX_SPEED_TGT = 2500.0*3;
    public static final double HOPPER_INDEX_SPEED_TGT = 1500.0*3;
    public static final double HOPPER_SHOOT_SPEED_TGT = 1000.0*3; //3500
    public static final double HOPPER_EXHAUST_SPEED_TGT = -1500.0*3;
    public static final double HOPPER_SPINUP_SPEED_TGT = 0.0; //-1000 //-2500
    public static final double HOPPER_IDLE_TWO_BALL_SPEED_TGT = 0.0; //500

    public static final double REAR_HOPPER_INDEX_SPEED_TGT = -500.0;
    public static final double REAR_HOPPER_SHOOT_SPEED_TGT = 1500.0; //500
    public static final double REAR_HOPPER_EXHAUST_SPEED_TGT = -1000.0;
    public static final double REAR_HOPPER_SPINUP_SPEED_TGT = -500.0; //-1000 //-2500
    public static final double REAR_HOPPER_IDLE_SPEED_TGT = 0.0;
    public static final double REAR_HOPPER_SHOT_SPACING_SPEED_TGT = 0.0;

    public static final double INDEXER_INDEX_SPEED_TGT = -750.0;
    public static final double INDEXER_INDEX_IN_SPEED_TGT = 250.0;
    public static final double INDEXER_SHOOT_SPEED_TGT = 1000.0; //10000
    public static final double INDEXER_SPINUP_SPEED_TGT = -667.0;
    public static final double INDEXER_EXHAUST_SPEED_TGT = -3500.0;
    public static final double INDEXER_SHOT_SPACING_SPEED_TGT = -667.0; //-2500
    public static final double INDEXER_IDLE_TWO_BALL_SPEED_TGT = 0.0;
    public static final double HOPPER_SHOT_SPACING_TIME = 0.75; //seconds

    /* NEO CALIBRATIONS */
    public static final double HOPPER_KF = 0.00014;
    public static final double HOPPER_KP = 0.0001;
    public static final double HOPPER_KI = 0;
    public static final double HOPPER_KD = 0;
}
