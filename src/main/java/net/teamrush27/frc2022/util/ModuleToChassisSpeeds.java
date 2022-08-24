package net.cachemoney8096.frc2022o.util;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static net.cachemoney8096.frc2022o.calibrations.DrivetrainCals.SPEED_ADJ_FACTOR;

public class ModuleToChassisSpeeds {
    static double[] vChassis = new double[4];

    public static void updateChassisSpeeds(SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule backLeftModule, SwerveModule backRightModule, double limelightAngleOffset){

        double VxFL = frontLeftModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.cos(frontLeftModule.getSteerAngle());
        double VyFL = frontLeftModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.sin(frontLeftModule.getSteerAngle());

        double VxFR = frontRightModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.cos(frontRightModule.getSteerAngle());
        double VyFR = frontRightModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.sin(frontRightModule.getSteerAngle());

        double VxRL = backLeftModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.cos(backLeftModule.getSteerAngle());
        double VyRL = backLeftModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.sin(backLeftModule.getSteerAngle());

        double VxRR = backRightModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.cos(backRightModule.getSteerAngle());
        double VyRR = backRightModule.getDriveVelocity()/SPEED_ADJ_FACTOR * Math.sin(backRightModule.getSteerAngle());

        //Robot relative speeds
        vChassis[0] = (VxFL + VxFR + VxRL + VxRR)/4;
        vChassis[1] = (VyFL + VyFR + VyRL + VyRR)/4;
        SmartDashboard.putNumber("shootWhileMoving/Vx", vChassis[0]);
        SmartDashboard.putNumber("shootWhileMoving/Vy", vChassis[1]);

        //Goal relative speeds
        vChassis[2] = vChassis[0]; //* Math.cos(Math.toRadians(limelightAngleOffset)) - vChassis[1] * Math.sin(Math.toRadians(limelightAngleOffset));
        vChassis[3] = vChassis[0] * Math.sin(Math.toRadians(limelightAngleOffset)) + vChassis[1] * Math.cos(Math.toRadians(limelightAngleOffset));
        SmartDashboard.putNumber("shootWhileMoving/Vx2", vChassis[2]);
        SmartDashboard.putNumber("shootWhileMoving/Vy2", vChassis[3]);
    }

    public static double[] getChassisSpeeds(){
        return vChassis;
    }

}
