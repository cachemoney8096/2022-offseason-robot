package net.teamrush27.frc2022.subsystems;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2022.FeatureEnables;
import net.teamrush27.frc2022.calibrations.LauncherCals;
import net.teamrush27.frc2022.util.ModuleToChassisSpeeds;
import org.photonvision.common.hardware.VisionLEDMode;
import net.teamrush27.frc2022.util.XboxController;
import net.teamrush27.frc2022.util.oneDimensionalLookup;

import static net.teamrush27.frc2022.Constants.*;
import static net.teamrush27.frc2022.FeatureEnables.*;
import static net.teamrush27.frc2022.calibrations.DrivetrainCals.*;
import static net.teamrush27.frc2022.calibrations.GlobalCals.*;

public class Drivetrain implements Subsystem {

    private Limelight limelight;
    private Launcher launcher;
  
    public static final double MAX_VOLTAGE = 12.0;
  
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
 
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = DRIVETRAIN_MAX_YAW_RATE;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

 
    private final Pigeon2 pigeon2;
    private double gyroOffset;
   
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
    private double headingLockAngle = 0;
    private boolean lockHeading = false;
    private boolean noDriverRotCmd = false;
    private PIDController yawCtrl = new PIDController(TELEOP_YAW_FB_CTRL_KP, TELEOP_YAW_FB_CTRL_KI, TELEOP_YAW_FB_CTRL_KD);
    private PIDController goalYawCtrl = new PIDController(GOAL_TRACKING_KP, GOAL_TRACKING_KI, GOAL_TRACKING_KD);
    private SwerveModuleState[] trajectoryStates = new SwerveModuleState[4];
    private Debouncer goalTrackedDebounce = new Debouncer(0.2, Debouncer.DebounceType.kFalling);
    private double targetYawAngle;

    private enum SystemState{
        IDLE,
        TRAJECTORY_FOLLOWING,     //auton trajectory follower
        MANUAL_CONTROL,           //X,Y axis speeds relative to field
        HEADING_LOCK,             //retain body angle after rotation command
        GOAL_TRACKING,           //retrorefelctive target tracking
        BALL_HUNT,                //track game piece
        POSITION_HOLD,            //lock pod angles at 45deg to resist pushing
        KEEP_ANGLES,               //hold current poda angles after driving instead of resetting to 0
        MOVE_AND_SHOOT,
        AUTO_DRIVE_GOAL_TRACKING
    }

    public enum WantedState{
        IDLE,
        TRAJECTORY_FOLLOWING,
        MANUAL_CONTROL,
        GOAL_TRACKING,
        BALL_HUNT,
        MOVE_AND_SHOOT,
        AUTO_DRIVE_GOAL_TRACKING
    }

    private static class PeriodicIO {
        // INPUTS
        double timestamp;

        double VxCmd; //longitudinal speed
        double VyCmd; //lateral speed
        double WzCmd; //rotational rate in radians/sec
        boolean robotOrientedModifier; //drive command modifier to set robot oriented translation control

        double limelightAngleError;
        double limelightDistance;

        double yawRateMeasured;
        double adjustedYaw;

        double chassisVx;
        double chassisVy;
        double goalVx;
        double goalVy;

        // OUTPUTS
        SwerveModuleState[] swerveStatesOut;
    }

    private static class StateVariables {
        double frontLeftDriveLast;
        double frontLeftAngleLast;
        double frontRightDriveLast;
        double frontRightAngleLast;
        double rearLeftDriveLast;
        double rearLeftAngleLast;
        double rearRightDriveLast;
        double rearRightAngleLast;
    }

    private final PeriodicIO periodicIO = new PeriodicIO();
    private final StateVariables stateVariables = new StateVariables();

    private SystemState currentState = SystemState.MANUAL_CONTROL;
    private WantedState wantedState = WantedState.MANUAL_CONTROL;

    private final CargoManager cargoManager;
    private final XboxController controller;

    private double currentStateStartTime;

    private LinearFilter limelightAngleFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * GOAL_TRACKING_ANGLE_ERROR_FC), 0.02);
    private LinearFilter limelightDistanceFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * GOAL_TRACKING_DISTANCE_FC), 0.02);

    private SlewRateLimiter moveAndShootAccelLimiterX = new SlewRateLimiter(MOVE_AND_SHOOT_MAX_ACCEL);
    private SlewRateLimiter moveAndShootAccelLimiterY = new SlewRateLimiter(MOVE_AND_SHOOT_MAX_ACCEL);

    private double[] autoDriveSpeeds = new double[2];

    public Drivetrain(CargoManager cargoManager, XboxController controller, Limelight limelight, Launcher launcher) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        //yawCtrl.enableContinuousInput(-Math.PI, Math.PI);  //TODO check if Pigeon output rolls over 

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

        // We will do the same for the other modules
        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
        );

        this.cargoManager = cargoManager;
        this.controller = controller;
        this.limelight = limelight;
        this.launcher = launcher;

        pigeon2 = new Pigeon2(PIGEON_ID, "Drivetrain");
        pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 100);
        pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 100);

        //TODO TEMPORARY
        resetOdometry();
        goalYawCtrl.setIntegratorRange(-GOAL_TRACKING_IMAX, GOAL_TRACKING_IMAX);
        gyroOffset = 0;
        targetYawAngle = 0;
    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch (currentState){
            default:
            case MANUAL_CONTROL:
                newState = handleManualControl();
                break;
            case TRAJECTORY_FOLLOWING:
                newState = handleTrajectoryFollowing();
                break;
            case GOAL_TRACKING:
                newState = handleGoalTracking();
                break;
            case MOVE_AND_SHOOT:
                newState = handleMoveAndShoot();
                break;
            case BALL_HUNT:
                newState = handleBallHunt();
                break;
            case HEADING_LOCK:
                newState = handleHeadingLock();
                break;
            case POSITION_HOLD:
                newState = handlePositionHold();
                break;
            case KEEP_ANGLES:
                newState = handleKeepAngles();
                break;
            case IDLE:
                newState = handleManualControl();
                break;
            case AUTO_DRIVE_GOAL_TRACKING:
                newState = handleAutoDriveGoalTracking();
                break;
        }
        if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
        updateOdometry();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        periodicIO.VxCmd = -oneDimensionalLookup.interpLinear(XY_Axis_inputBreakpoints, XY_Axis_outputTable, controller.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND;
        periodicIO.VyCmd = -oneDimensionalLookup.interpLinear(XY_Axis_inputBreakpoints, XY_Axis_outputTable, controller.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND;
        periodicIO.WzCmd = -oneDimensionalLookup.interpLinear(RotAxis_inputBreakpoints, RotAxis_outputTable, controller.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        periodicIO.robotOrientedModifier = controller.getLeftTriggerAxis() > 0.25;
        if(limelight.hasTargets()){
            periodicIO.limelightAngleError = limelightAngleFilter.calculate(limelight.angle_error());
            periodicIO.limelightDistance = limelightDistanceFilter.calculate(limelight.distanceToTarget());
        }
        periodicIO.adjustedYaw = pigeon2.getYaw() + gyroOffset;

        double[] chassisVelocity = ModuleToChassisSpeeds.getChassisSpeeds();
        periodicIO.chassisVx = chassisVelocity[0];
        periodicIO.chassisVy = chassisVelocity[1];
        periodicIO.goalVx = chassisVelocity[2];
        periodicIO.goalVy = chassisVelocity[3];

    }
    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        switch(currentState){
            case TRAJECTORY_FOLLOWING:
                moduleStates = trajectoryStates;
                break;
            case MANUAL_CONTROL:
                moduleStates = drive(periodicIO.VxCmd, periodicIO.VyCmd, periodicIO.WzCmd, !periodicIO.robotOrientedModifier);
                break;
            case HEADING_LOCK:
                moduleStates = drive(periodicIO.VxCmd, periodicIO.VyCmd, headingLock(periodicIO.VxCmd, periodicIO.VyCmd, periodicIO.WzCmd), !periodicIO.robotOrientedModifier);
                break;
            case GOAL_TRACKING:
                moduleStates = drive(periodicIO.VxCmd, periodicIO.VyCmd, goalTrackingHeadingCtrl(), !periodicIO.robotOrientedModifier);
                break;
            case MOVE_AND_SHOOT:
                double[] moveAndShootSpeeds = calculateMoveAndShootSpeedLimiter(periodicIO.VxCmd, periodicIO.VyCmd);
                moduleStates = drive(moveAndShootSpeeds[0],
                        moveAndShootSpeeds[1],
                        goalTrackingHeadingCtrl(),
                        !periodicIO.robotOrientedModifier);
                break;
            case BALL_HUNT:
                //TODO add ball hunt method 
                moduleStates = drive(periodicIO.VxCmd, periodicIO.VyCmd, ballHuntHeadingCtrl(), !periodicIO.robotOrientedModifier);
                break;
            case POSITION_HOLD:
                moduleStates[0] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0));
                moduleStates[1] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0));
                moduleStates[2] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0));
                moduleStates[3] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0));
                break;
            case KEEP_ANGLES:
                moduleStates[0] = new SwerveModuleState(0.0, new Rotation2d(stateVariables.frontLeftAngleLast));
                moduleStates[1] = new SwerveModuleState(0.0, new Rotation2d(stateVariables.frontRightAngleLast));
                moduleStates[2] = new SwerveModuleState(0.0, new Rotation2d(stateVariables.rearLeftAngleLast));
                moduleStates[3] = new SwerveModuleState(0.0, new Rotation2d(stateVariables.rearRightAngleLast));
                break;
            default:
            case IDLE:
                moduleStates = drive(0.0, 0.0, 0.0, true);
                break;
            case AUTO_DRIVE_GOAL_TRACKING:
                moduleStates = drive(autoDriveSpeeds[0], autoDriveSpeeds[1], goalTrackingHeadingCtrl(), true);
                break;
        }
        setModuleStates(moduleStates);
        updateStateVariables(moduleStates);
    }

    /*@Override
    public void periodic() {
        updateOdometry();
    }*/

    private SystemState defaultStateChange() {
		switch (wantedState){
            /*case IDLE:
                return SystemState.IDLE;*/
			case TRAJECTORY_FOLLOWING:
				return SystemState.TRAJECTORY_FOLLOWING;
			case GOAL_TRACKING:
				return SystemState.GOAL_TRACKING;
            case MOVE_AND_SHOOT:
                return SystemState.MOVE_AND_SHOOT;
			case BALL_HUNT:
				return SystemState.BALL_HUNT;
            default:
            case MANUAL_CONTROL:
                return SystemState.MANUAL_CONTROL;
            case AUTO_DRIVE_GOAL_TRACKING:
                return SystemState.AUTO_DRIVE_GOAL_TRACKING;
		}
	}

    private void updateStateVariables(SwerveModuleState[] states)
    {
        stateVariables.frontLeftDriveLast = states[0].speedMetersPerSecond;
        stateVariables.frontRightDriveLast = states[1].speedMetersPerSecond;
        stateVariables.rearLeftDriveLast = states[2].speedMetersPerSecond;
        stateVariables.rearRightDriveLast = states[3].speedMetersPerSecond;

        stateVariables.frontLeftAngleLast = states[0].angle.getRadians();
        stateVariables.frontRightAngleLast = states[1].angle.getRadians();
        stateVariables.rearLeftAngleLast = states[2].angle.getRadians();
        stateVariables.rearRightAngleLast = states[3].angle.getRadians();
    }


    public void initAutonPosition(PathPlannerTrajectory.PathPlannerState state){
        zeroGyroscope();
        gyroOffset = state.holonomicRotation.getDegrees();
        periodicIO.adjustedYaw =  pigeon2.getYaw() + gyroOffset;
        //ErrorCode errorCode = pigeon.setYaw(state.holonomicRotation.getDegrees(), 100);
        odometry.resetPosition(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation),getGyroscopeRotation());
    }

    @Override
    public void zeroSensors(){
        resetOdometry();
        setModuleStates(drive(0.0, 0.0, 0.0, true));
    }

    @Override
    public void stop(){
        setModuleStates(drive(0.0, 0.0, 0.0, true));
    }

    @Override
    public String getId() {
        return "Drivetrain";
    }

    @Override
    public boolean checkSystem() {
        return DrivetrainSystemCheck.runDrivetrainSystemCheck();
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putString("drivetrain/wantedStateAPI", this.wantedState.toString());
        SmartDashboard.putNumber("drivetrain/heading",periodicIO.adjustedYaw);
        SmartDashboard.putString("drivetrain/pose",odometry.getPoseMeters().toString());
        SmartDashboard.putString("drivetrain/currentState", currentState.toString());
        SmartDashboard.putString("drivetrain/wantedState", wantedState.toString());
        SmartDashboard.putNumber("drivetrain/VxCmd", periodicIO.VxCmd);
        SmartDashboard.putNumber("drivetrain/VyCmd", periodicIO.VyCmd);
        SmartDashboard.putNumber("drivetrain/WzCmd", periodicIO.WzCmd);
        SmartDashboard.putNumber("driverController/LeftY", controller.getLeftY());
        SmartDashboard.putNumber("driverController/LeftX", controller.getLeftX());
        SmartDashboard.putNumber("driverController/RightX", controller.getRightX());
        SmartDashboard.putString("drivetrain/currentStateOutputs", currentState.toString());
        SmartDashboard.putString("drivetrain/wantedStateOutputs", wantedState.toString());
        SmartDashboard.putNumber("drivetrain/goalLimelightAngleError", periodicIO.limelightAngleError);
        SmartDashboard.putNumber("drivetrain/yawAngle", periodicIO.adjustedYaw);
        SmartDashboard.putString("drivetrain/pose", getPose().toString());
        SmartDashboard.putBoolean("drivetrain/isGoalTracked", isGoalTracked());
        SmartDashboard.putBoolean("launchCheck/isGoalTracked", isGoalTracked());
        SmartDashboard.putNumber("drivetrain/chassisVx", periodicIO.chassisVx);
        SmartDashboard.putNumber("drivetrain/chassisVy", periodicIO.chassisVy);
        SmartDashboard.putNumber("drivetrain/goalVx", periodicIO.goalVx);
        SmartDashboard.putNumber("drivetrain/goalVy", periodicIO.goalVy);
    }

    public void setWantedState(WantedState wantedState)
    {
        this.wantedState = wantedState;
    }

    public void zeroGyroscope() {
        pigeon2.setYaw(0.0);
        gyroOffset = 0;
    }

    public SwerveModuleState[] drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        rot = headingLock(xSpeed, ySpeed, rot);
        chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroscopeRotation())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        return states;
    }
  
    public void setModuleStates(SwerveModuleState[] states){
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    public void setModuleStatesFromTrajectory(SwerveModuleState[] states){
        trajectoryStates = states;
    }

    private double headingLock(double xSpeed, double ySpeed, double rot){
        double rotVal;
        //yawCtrl.enableContinuousInput(-Math.PI, Math.PI);
        noDriverRotCmd = TELEOP_YAW_FB_CTRL_EN && Math.abs(rot) < 0.05 && Math.max(Math.abs(xSpeed), Math.abs(ySpeed)) > 0.1;
        if (!noDriverRotCmd){
            //Lock heading angle
            headingLockAngle = periodicIO.adjustedYaw;
        }
        if (noDriverRotCmd){
            rotVal = yawCtrl.calculate(periodicIO.adjustedYaw, headingLockAngle);
            lockHeading = true;
        }
        else {
            rotVal = rot;
            lockHeading = false;
        }    
        return rotVal;
    }

    private double goalTrackingHeadingCtrl()
    {
        double WzGoalTrack; //rotation rate for goal tracking
        //double targetAngle;
        //yawCtrl.enableContinuousInput(-Math.PI, Math.PI);

        if (FeatureEnables.SHOOT_WHILE_MOVING && currentState.equals(SystemState.MOVE_AND_SHOOT)){

            targetYawAngle = periodicIO.adjustedYaw - periodicIO.limelightAngleError + calculateStaticLimelightAngleOffset(periodicIO.limelightDistance)
                    + calculateDynamicLimelightAngleOffset(periodicIO.goalVy);
        } else {
            targetYawAngle = periodicIO.adjustedYaw - periodicIO.limelightAngleError + calculateStaticLimelightAngleOffset(periodicIO.limelightDistance);
        }

        if(limelight.hasTargets() && Math.abs(periodicIO.WzCmd) < 0.01) {
            WzGoalTrack = goalYawCtrl.calculate(periodicIO.adjustedYaw, targetYawAngle);
        } else {
            WzGoalTrack = periodicIO.WzCmd;
        }
        return WzGoalTrack;
    }

    private double ballHuntHeadingCtrl()
    {
        double WzBallHunt = 0.0; //rotation rate for goal tracking
        return WzBallHunt;
    }
/*
    private SystemState handleIdle(){
        if (periodicIO.VxCmd > DRIVETRAIN_IDLE_VXVY_THRESHOLD || periodicIO.VyCmd > DRIVETRAIN_IDLE_VXVY_THRESHOLD || periodicIO.WzCmd > DRIVETRAIN_IDLE_WZ_THRESHOLD)
        {
           return SystemState.MANUAL_CONTROL;
        } else {
            return defaultStateChange();
        }
    }*/

    private SystemState handleTrajectoryFollowing(){
        return defaultStateChange();
    }

    private SystemState handleAutoDriveGoalTracking(){
        limelight.camera.setLED(VisionLEDMode.kOn);
        return defaultStateChange();
    }

    private SystemState handleManualControl(){
        limelight.camera.setLED(VisionLEDMode.kOff);

        if (checkNoDriveInput()){
            if (cargoManager.getWantedState().equals(CargoManager.WantedState.SHOOT)){
                return SystemState.POSITION_HOLD;
            } else {
                return SystemState.KEEP_ANGLES;}
        } else {
        return defaultStateChange();
        }
    }

    private SystemState handleHeadingLock(){
        return defaultStateChange();
    }
    
    private SystemState handleGoalTracking(){
        limelight.camera.setLED(VisionLEDMode.kOn);
        return defaultStateChange();
    }

    private SystemState handleMoveAndShoot(){
        limelight.camera.setLED(VisionLEDMode.kOn);
        return defaultStateChange();
    }

    private SystemState handleBallHunt(){
        return defaultStateChange();
    }

    private SystemState handlePositionHold(){
        if (wantedState == WantedState.TRAJECTORY_FOLLOWING){
            return SystemState.TRAJECTORY_FOLLOWING;
        } else if (!checkNoDriveInput()) {
            return defaultStateChange();
        } else {
            return SystemState.POSITION_HOLD;
        }
    }

    private SystemState handleKeepAngles() {
        if (wantedState == WantedState.TRAJECTORY_FOLLOWING){
            return SystemState.TRAJECTORY_FOLLOWING;
        } else if (wantedState == WantedState.GOAL_TRACKING){
            return SystemState.GOAL_TRACKING;
        } else if (wantedState == WantedState.MOVE_AND_SHOOT) {
            return SystemState.MOVE_AND_SHOOT;
        } else if (wantedState == WantedState.AUTO_DRIVE_GOAL_TRACKING){
            return SystemState.AUTO_DRIVE_GOAL_TRACKING;
        } else if (checkNoDriveInput()){
            if (cargoManager.getWantedState().equals(CargoManager.WantedState.SHOOT)){
                return SystemState.POSITION_HOLD;
            } else{
            return SystemState.KEEP_ANGLES;}
        } else {
        return defaultStateChange();
        }
    }

    private boolean checkNoDriveInput(){
        return(Math.abs(periodicIO.VxCmd) < DRIVETRAIN_IDLE_VXVY_THRESHOLD
                && Math.abs(periodicIO.VyCmd) < DRIVETRAIN_IDLE_VXVY_THRESHOLD
                && Math.abs(periodicIO.WzCmd) < DRIVETRAIN_IDLE_WZ_THRESHOLD);
    }

    public Rotation2d getGyroscopeRotation() {
        Rotation2d temp = Rotation2d.fromDegrees(periodicIO.adjustedYaw);
        return temp;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    private void updateOdometry(){
        odometry.update(getGyroscopeRotation(), 
                new SwerveModuleState(frontLeftModule.getDriveVelocity()/SPEED_ADJ_FACTOR, new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity()/SPEED_ADJ_FACTOR, new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity()/SPEED_ADJ_FACTOR, new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity()/SPEED_ADJ_FACTOR, new Rotation2d(backRightModule.getSteerAngle())));

        ModuleToChassisSpeeds.updateChassisSpeeds(frontLeftModule, frontRightModule, backLeftModule, backRightModule, periodicIO.limelightAngleError);
    }

    public void resetOdometry(){
        zeroGyroscope();
        odometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), getGyroscopeRotation());
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public boolean isGoalTracking(){
        return SystemState.GOAL_TRACKING.equals(currentState) || SystemState.MOVE_AND_SHOOT.equals(currentState)
                || SystemState.AUTO_DRIVE_GOAL_TRACKING.equals(currentState);
    }

    public boolean isGoalTracked(){
        double angleTolerance = Math.atan2(GOAL_TRACKING_MAX_LATERAL_ERROR_AT_GOAL, periodicIO.limelightDistance) * 180 / Math.PI;
        if (isGoalTracking() && goalTrackedDebounce.calculate(Math.abs(periodicIO.adjustedYaw - targetYawAngle) < angleTolerance)){
            launcher.setShootWhileMovingLocked(true);
            return true;
        } else {
            launcher.setShootWhileMovingLocked(false);
            return false;
        }
    }

    private double calculateStaticLimelightAngleOffset(double distance){
        return Math.atan2(GOAL_TRACKING_CENTER_DISTANCE_OFFSET, distance) * 180 / Math.PI;
    }


    private double calculateDynamicLimelightAngleOffset(double chassisLateralVelocity){

        double VyGoal = -periodicIO.yawRateMeasured * periodicIO.limelightDistance - chassisLateralVelocity;
        double timeOfFlight = oneDimensionalLookup.interpLinear(LauncherCals.LAUNCHER_LIMELIGHT_DISTANCE_IDX,
                LauncherCals.LAUNCHER_LIMELIGHT_TIME_OF_FLIGHT, periodicIO.limelightDistance);
        double lateralOffset = VyGoal * timeOfFlight;
        SmartDashboard.putNumber("shootWhileMoving/lateralOffset", lateralOffset);
        return Math.atan2(lateralOffset, periodicIO.limelightDistance) * 180 / Math.PI;
    }

    private double[] calculateMoveAndShootSpeedLimiter(double VxCommand, double VyCommand){
        double VxLimited;
        double VyLimited;
        double VLimited;
        double[] limitedSpeeds = new double[2];
        //TODO need limiter based on unit circle
        /*if (VxCommand >= 0){
            VxLimited = Math.min(MOVE_AND_SHOOT_MAX_SPEED, VxCommand);
        } else {
            VxLimited = Math.max(-MOVE_AND_SHOOT_MAX_SPEED, VxCommand);
        }
        if (VyCommand >= 0){
            VyLimited = Math.min(MOVE_AND_SHOOT_MAX_SPEED, VyCommand);
        } else {
            VyLimited = Math.max(-MOVE_AND_SHOOT_MAX_SPEED, VyCommand);
        }*/
        double angle = Math.atan2(VyCommand, VxCommand);
        double VCommand = Math.sqrt(Math.pow(VxCommand, 2) + Math.pow(VyCommand, 2));
        if (VCommand >= 0){
            VLimited = Math.min(MOVE_AND_SHOOT_MAX_SPEED, VCommand);
        } else {
            VLimited = Math.max(-MOVE_AND_SHOOT_MAX_SPEED, VCommand);
        }
        VxLimited = VLimited * Math.cos(angle);
        VyLimited = VLimited * Math.sin(angle);
        VxLimited = moveAndShootAccelLimiterX.calculate(VxLimited);
        VyLimited = moveAndShootAccelLimiterY.calculate(VyLimited);
        limitedSpeeds[0] = VxLimited;
        limitedSpeeds[1] = VyLimited;
        return limitedSpeeds;
    }

    public void setAutoDriveSpeeds(double xSpeed, double ySpeed){
        autoDriveSpeeds[0] = xSpeed;
        autoDriveSpeeds[1] = ySpeed;
    }
}
