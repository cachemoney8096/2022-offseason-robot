package net.cachemoney8096.frc2022o.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.cachemoney8096.frc2022o.subsystems.Indexer;
import net.cachemoney8096.frc2022o.subsystems.Shooter;
import net.cachemoney8096.frc2022o.subsystems.drive.DriveSubsystem;

/** Controls the robot to try to shoot
 * When robot doesn't see a target, driver controls translation and rotation
 * When robot sees a target, driver controls translation still
 * When robot sees a target, the shooter velocity and hood position controllers are set
 * When the robot is aligned, movement is locked
 * When the robot is aligned and everything is ready, shooting is automatic
 */
public class ShootCommand extends InstantCommand {

  // Required subsystems
  private final DriveSubsystem drivetrain;
  private final Indexer indexer;
  private final Shooter shooter;

  // Desired Robot Translation and rotation while rotating to shoot
  private final double xSpeed;
  private final double ySpeed;
  private final double rotSpeed;
  private final boolean fieldRelative;

  /**
   * @param xSpeedIn        Speed of the robot in the x direction (forward) in
   *                        [0,1].
   * @param ySpeedIn        Speed of the robot in the y direction (sideways) in
   *                        [0,1].
   * @param rotSpeedIn      Rotation of the robot (CCW) in [0,1]. Only applies if
   *                        target is not observed.
   * @param fieldRelativeIn Whether the provided x and y speeds are relative to
   *                        the field.
   */
  public ShootCommand(DriveSubsystem drivetrainIn, Indexer indexerIn, Shooter shooterIn, double xSpeedIn,
      double ySpeedIn,
      double rotSpeedIn,
      boolean fieldRelativeIn) {
    super();

    drivetrain = drivetrainIn;
    indexer = indexerIn;
    shooter = shooterIn;
    xSpeed = xSpeedIn;
    ySpeed = ySpeedIn;
    rotSpeed = rotSpeedIn;
    fieldRelative = fieldRelativeIn;

    addRequirements(drivetrain, indexer, shooter);
  }

  @Override
  public void initialize() {
    shooter.shoot(); // cargo won't get shot unless we actually feed the cargo in

    // If we're aligned, then stop!
    if (drivetrain.alignedToTarget()) {
      drivetrain.setX();
    } else {
      drivetrain.rotateToShoot(xSpeed, ySpeed, rotSpeed, fieldRelative);
    }

    // Shoot if we're ready
    if (drivetrain.alignedToTarget() && !drivetrain.isMoving() && shooter.checkShootReady()) {
      indexer.feedShooter();
    } else {
      indexer.indexBall();
    }
  }

  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Aligned to target", drivetrain::alignedToTarget, null);
    builder.addBooleanProperty("Drivetrain moving", drivetrain::isMoving, null);
    builder.addBooleanProperty("Shooter ready", shooter::checkShootReady, null);
  }
}
