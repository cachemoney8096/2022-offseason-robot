package net.teamrush27.frc2022.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static net.teamrush27.frc2022.Constants.*;

public class Limelight implements Subsystem {

    PhotonCamera camera = new PhotonCamera("Limelight");

    LinearFilter filter = LinearFilter.movingAverage(10);

    public Limelight() {
        PhotonCamera.setVersionCheckEnabled(false);
    }

    private class PeriodicIO {
        //Inputs

        //Outputs
        double distance;
        double angle_error;
        boolean hasTargets;
    } 

    private final PeriodicIO periodicIO = new PeriodicIO();

    @Override
	public void readPeriodicInputs(double timestamp) {
        
    }

    @Override
    public void processLoop(double timestamp) { 

            var result = camera.getLatestResult();

            if (result.hasTargets()) {

                double d_estimate =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                UPPER_HUB_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch())); 

                periodicIO.distance = filter.calculate(d_estimate - CAMERA_OFFSET);

            }

            if (result.hasTargets()) {
                
                double x_angle = result.getBestTarget().getYaw() * Math.PI / 180;

                //calculates the error of anlge from the robot heading to the hub
                double d_fromCG = Math.sqrt(Math.pow(periodicIO.distance, 2) + Math.pow(CAMERA_OFFSET, 2) - 2 * periodicIO.distance * CAMERA_OFFSET * Math.cos(x_angle)); 
                double small_angle = Math.asin( CAMERA_OFFSET * Math.sin(x_angle) / d_fromCG);

                periodicIO.angle_error = ((small_angle + x_angle)  * 180 / Math.PI);

            }

            if(result.hasTargets()){
                periodicIO.hasTargets = true;
            } else {
                periodicIO.hasTargets = false;
            }
    }


    @Override
	public void writePeriodicOutputs(double timestamp) {

        SmartDashboard.putNumber("limelight/estimatedDistance" , periodicIO.distance);
        SmartDashboard.putNumber("limelight/estimatedAngleError" , periodicIO.angle_error);
        SmartDashboard.putBoolean("limelight/hasTarget" , periodicIO.hasTargets);

    }

    @Override
    public void stop() {
        
    }

    public double angle_error() {
        return periodicIO.angle_error;
    }
    
    public double distanceToTarget() {
        return periodicIO.distance;
      }

    public boolean hasTargets() {
        return periodicIO.hasTargets;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
               
    }

    @Override
    public String getId() {
        return "Limelight";
    }

}