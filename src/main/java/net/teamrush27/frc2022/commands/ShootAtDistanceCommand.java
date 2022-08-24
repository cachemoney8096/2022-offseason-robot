package net.cachemoney8096.frc2022o.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.cachemoney8096.frc2022o.subsystems.Launcher;

public class ShootAtDistanceCommand extends InstantCommand {

	private final Launcher launcher;
	private final double distance;

	public ShootAtDistanceCommand(Launcher launcher, double distance){
		this.launcher = launcher;
		this.distance = distance;
	}

	@Override
	public void initialize() {
		launcher.setAutonShot(distance);
	}
}
