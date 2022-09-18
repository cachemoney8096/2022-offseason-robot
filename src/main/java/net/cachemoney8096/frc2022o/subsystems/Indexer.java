package net.cachemoney8096.frc2022o.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.cachemoney8096.frc2022o.RobotMap;

public class Indexer implements Subsystem {

	// Actuators
	private final CANSparkMax indexerMotorOne;
	private final CANSparkMax indexerMotorTwo;
	private final CANSparkMax indexerMotorThree;

	// Sensors
	private final DigitalInput cargoSensor;

	public Indexer(){
		indexerMotorOne = new CANSparkMax(RobotMap.INDEXER_MOTOR_ONE_ID, MotorType.kBrushless);
		indexerMotorOne.restoreFactoryDefaults();
		indexerMotorOne.setIdleMode(CANSparkMax.IdleMode.kCoast);
		indexerMotorOne.setInverted(false);
		// indexerMotorOne.setSmartCurrentLimit(25);
		// following line used to reduce CAN utilization?
		// indexerMotorOne.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
		
		indexerMotorTwo = new CANSparkMax(RobotMap.INDEXER_MOTOR_TWO_ID, MotorType.kBrushless);
		indexerMotorTwo.restoreFactoryDefaults();
		indexerMotorTwo.setIdleMode(CANSparkMax.IdleMode.kCoast);
		indexerMotorTwo.follow(indexerMotorOne);
		
		indexerMotorThree = new CANSparkMax(RobotMap.INDEXER_MOTOR_THREE_ID, MotorType.kBrushless);
		indexerMotorThree.restoreFactoryDefaults();
		indexerMotorThree.setIdleMode(CANSparkMax.IdleMode.kCoast);

		cargoSensor = new DigitalInput(RobotMap.INDEXER_DIO);
	}

	@Override
	public void periodic() {
		// do nothing periodically
	}

	public boolean hasCargo() {
		return cargoSensor.get();
	}

	public void indexBall() {
		// run one/two forward
		// don't run three
	}

	public void ejectBall() {
		// run one/two forward
		// run three backwards
	}

	public void feedShooter() {
		// run all three forwards
	}

}
