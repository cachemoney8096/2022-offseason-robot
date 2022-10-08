package net.cachemoney8096.frc2022o.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import net.cachemoney8096.frc2022o.RobotMap;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase{
    private final CANSparkMax climbMotorRight;
    private final CANSparkMax climbMotorLeft;
    private final boolean INVERSION_PLACEHOLDER = false;

    public Climber(){
        climbMotorRight = new CANSparkMax(RobotMap.CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);
        climbMotorLeft = new CANSparkMax(RobotMap.CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);

        climbMotorRight.restoreFactoryDefaults();
        climbMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climbMotorRight.setInverted(INVERSION_PLACEHOLDER); // TODO see which way motors are facing and invert such that positive = in

        climbMotorLeft.restoreFactoryDefaults();
        climbMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climbMotorLeft.setInverted(INVERSION_PLACEHOLDER); // TODO see which way motors are facing and invert such that positive = in
    }

    public void rightMotorDown(){
        climbMotorRight.set(-1.0);
    }
    public void rightMotorUp(){
        climbMotorRight.set(1.0);
    }
    public void leftMotorDown(){
        climbMotorLeft.set(-1.0);
    }
    public void leftMotorUp(){
        climbMotorLeft.set(1.0);
    }
}
