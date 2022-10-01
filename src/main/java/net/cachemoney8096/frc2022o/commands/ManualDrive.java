package net.cachemoney8096.frc2022o.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.cachemoney8096.frc2022o.FeatureFlags;
import net.cachemoney8096.frc2022o.libs.XboxController;
import net.cachemoney8096.frc2022o.subsystems.Drivetrain;

public class ManualDrive extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final XboxController driverController;

  public ManualDrive(Drivetrain drivetrainIn,
                    XboxController driverControllerIn){
    drivetrain = drivetrainIn;
    addRequirements(drivetrain);

    driverController = driverControllerIn;
}

@Override
public void initialize() {
  boolean useNonDefault = driverController.getLeftBumper();
  boolean fieldRelative = FeatureFlags.DEFAULT_FIELD_RELATIVE ^ useNonDefault;
  drivetrain.drive(
    driverController.getLeftY(), // Y on the joystick (up-down) maps to X for the robot
    driverController.getLeftX(),
    driverController.getRightX(),
    fieldRelative);
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
