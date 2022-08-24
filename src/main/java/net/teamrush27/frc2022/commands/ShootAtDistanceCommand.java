package net.teamrush27.frc2022.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.teamrush27.frc2022.subsystems.Launcher;

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
