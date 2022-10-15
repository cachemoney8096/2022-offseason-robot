package net.cachemoney8096.frc2022o.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.cachemoney8096.frc2022o.subsystems.Indexer;
import net.cachemoney8096.frc2022o.subsystems.Shooter;
import net.cachemoney8096.frc2022o.subsystems.drive.DriveSubsystem;

public class ShootCommand extends InstantCommand {

  // Required subsystems
  private final DriveSubsystem drivetrain;
  private final Indexer indexer;
  private final Shooter shooter;

  public ShootCommand(DriveSubsystem drivetrainIn, Indexer indexerIn, Shooter shooterIn) {
    super();

    drivetrain = drivetrainIn;
    indexer = indexerIn;
    shooter = shooterIn;

    addRequirements(drivetrain, indexer, shooter);
  }

  @Override
  public void initialize() {
    shooter.shoot(); // cargo won't get shot unless we actually feed the cargo in
    drivetrain.rotateToShoot();
    if (drivetrain.alignedToTarget() && shooter.checkShootReady()) {
      indexer.feedShooter();
    } else {
      indexer.indexBall();
    }
  }
}
