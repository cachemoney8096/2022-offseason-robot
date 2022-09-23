package net.cachemoney8096.frc2022o.subsystems;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoDriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SwerveModuleFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.cachemoney8096.frc2022o.libs.CoussensModuleConfiguration;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.Constants;
import net.cachemoney8096.frc2022o.libs.AbsoluteEncoderFromDioConfiguration;
import net.cachemoney8096.frc2022o.libs.AbsoluteEncoderFromDioFactoryBuilder;

public class Drivetrain implements Subsystem {

  private final ModuleConfiguration moduleConfiguration;
  private final CoussensModuleConfiguration coussensModuleConfiguration;

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final double wheelDiametersInches = 3.0;
  private final double wheelDiameterMeters = Units.inchesToMeters(wheelDiametersInches);
  private final double driveReduction = 4.8;
  private final boolean driveInverted = false;
  private final double steerReduction = 11.25;
  private final boolean steerInverted = false;

  public Drivetrain() {
    moduleConfiguration =
        new ModuleConfiguration(
            wheelDiameterMeters, driveReduction, driveInverted, steerReduction, steerInverted);

    coussensModuleConfiguration = new CoussensModuleConfiguration();

    frontLeftModule =
        new SwerveModuleFactory<>(
                moduleConfiguration,
                new NeoDriveControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                    .build(),
                new NeoSteerControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withPidConstants(1.0, 0.0, 0.1)
                    .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                    .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
            .create(
                // container, maybe want to add shuffleboard here
                RobotMap.DRIVE_FRONT_LEFT_ID,
                new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                    RobotMap.STEER_FRONT_LEFT_ID,
                    new AbsoluteEncoderFromDioConfiguration(
                        RobotMap.SWERVE_FRONT_LEFT_DIO, Constants.FRONT_LEFT_STEER_OFFSET_RAD)));

    frontRightModule =
        new SwerveModuleFactory<>(
                moduleConfiguration,
                new NeoDriveControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                    .build(),
                new NeoSteerControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withPidConstants(1.0, 0.0, 0.1)
                    .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                    .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
            .create(
                // container, maybe want to add shuffleboard here
                RobotMap.DRIVE_FRONT_RIGHT_ID,
                new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                    RobotMap.STEER_FRONT_RIGHT_ID,
                    new AbsoluteEncoderFromDioConfiguration(
                        RobotMap.SWERVE_FRONT_RIGHT_DIO, Constants.FRONT_RIGHT_STEER_OFFSET_RAD)));

    backLeftModule =
        new SwerveModuleFactory<>(
                moduleConfiguration,
                new NeoDriveControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                    .build(),
                new NeoSteerControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withPidConstants(1.0, 0.0, 0.1)
                    .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                    .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
            .create(
                // container, maybe want to add shuffleboard here
                RobotMap.DRIVE_BACK_LEFT_ID,
                new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                    RobotMap.STEER_BACK_LEFT_ID,
                    new AbsoluteEncoderFromDioConfiguration(
                        RobotMap.SWERVE_BACK_LEFT_DIO, Constants.BACK_LEFT_STEER_OFFSET_RAD)));

    backRightModule =
        new SwerveModuleFactory<>(
                moduleConfiguration,
                new NeoDriveControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withCurrentLimit(coussensModuleConfiguration.getDriveCurrentLimit())
                    .build(),
                new NeoSteerControllerFactoryBuilder()
                    .withVoltageCompensation(coussensModuleConfiguration.getNominalVoltage())
                    .withPidConstants(1.0, 0.0, 0.1)
                    .withCurrentLimit(coussensModuleConfiguration.getSteerCurrentLimit())
                    .build(new AbsoluteEncoderFromDioFactoryBuilder().build()))
            .create(
                // container, maybe want to add shuffleboard here
                RobotMap.DRIVE_BACK_RIGHT_ID,
                new NeoSteerConfiguration<AbsoluteEncoderFromDioConfiguration>(
                    RobotMap.STEER_BACK_RIGHT_ID,
                    new AbsoluteEncoderFromDioConfiguration(
                        RobotMap.SWERVE_BACK_RIGHT_DIO, Constants.BACK_RIGHT_STEER_OFFSET_RAD)));
  }
}
