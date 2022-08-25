package net.cachemoney8096.frc2022o.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic
 * gyroscope calibration, etc.)
 */
public interface Loop {

	void onStart(double timestamp);

	void onLoop(double timestamp);

	void onStop(double timestamp);

	String getId();
}
