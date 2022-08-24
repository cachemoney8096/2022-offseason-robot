package net.cachemoney8096.frc2022o.subsystems;

public interface Subsystem extends edu.wpi.first.wpilibj2.command.Subsystem {

	void processLoop(double timestamp);

	void stop();

	boolean checkSystem();

	void zeroSensors();

	String getId();

	default void readPeriodicInputs(double timestamp) {

	}

	default void writePeriodicOutputs(double timestamp) {

	}

	default void outputTelemetry(double timestamp){

	}
}
