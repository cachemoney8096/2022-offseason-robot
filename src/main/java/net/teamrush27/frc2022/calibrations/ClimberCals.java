// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o.calibrations;

/** Add your docs here. */
public class ClimberCals {
    //Climber angle cals in degrees
    public static final double ANGLE_OFFSET = 0;
    public static final double IDLE_ANGLE = calculateClimberArmAngle(0);
    public static final double LVL2_READY_ANGLE = calculateClimberArmAngle(-90+ANGLE_OFFSET);
    public static final double LVL3_RELAX_ANGLE = calculateClimberArmAngle(90+ANGLE_OFFSET);
    public static final double LVL23_ROTATE_ANGLE = calculateClimberArmAngle(74+ANGLE_OFFSET);
    public static final double LVL23_SETTLE_ANGLE = calculateClimberArmAngle(30+ANGLE_OFFSET);
    public static final double LVL2_RELEASE_ANGLE = calculateClimberArmAngle(15+ANGLE_OFFSET); //15
//    public static final double LVL2_RELEASE_ANGLE = calculateClimberArmAngle(90+ANGLE_OFFSET); //15
    public static final double LVL34_ROTATE_ANGLE = calculateClimberArmAngle(180+72+ANGLE_OFFSET);
    public static final double LVL34_SETTLE_ANGLE = calculateClimberArmAngle(180+30+ANGLE_OFFSET);
    public static final double LVL3_RELEASE_ANGLE = calculateClimberArmAngle(180+15+ANGLE_OFFSET); //15
    public static final double LVL4_RELAX_ANGLE = calculateClimberArmAngle(270+ANGLE_OFFSET);

    //PID Gains for climber motors
    public static final double CLIMBER_MOTOR_KP = 0.10;
    public static final double CLIMBER_MOTOR_KI = 0.0;
    public static final double CLIMBER_MOTOR_KD = 0.0;
    public static final double CLIMBER_MOTOR_IZONE = calculateClimberArmAngle(5);

    //Motion Magic settings [units/100ms]
    public static final double CLIMB_CRUISE_VELOCITY = calculateClimberArmAngle(90d * 2d) / 10d;
    public static final double CLIMB_ACCELERATION = calculateClimberArmAngle(360d * .75) / 10d;
    public static final double CLIMB_CRUISE_VELOCITY_SETTLE = calculateClimberArmAngle(90d * 2d) / 10d; // 30d
    public static final double CLIMB_ACCELERATION_SETTLE = calculateClimberArmAngle(360d * .75) / 10d; // 90

    public static final double CLIMBER_RATIO = (84d/12d)*(84d/18d)*(60d/12d); //prototype, replace with real value later


    public static double calculateClimberArmAngle(double angle)
    {
        return (CLIMBER_RATIO*2048/360)*angle;
    }

    public static double climberTicksToDegrees(double ticks)
    {
        return (ticks * 360) / (CLIMBER_RATIO * 2048);
    }

}
