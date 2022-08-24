package net.teamrush27.frc2022.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.teamrush27.frc2022.subsystems.Hopper;

public class ConfigShotSpacingCommand extends InstantCommand {

	private final Hopper hopper;
	private final double timeToWait;

	public ConfigShotSpacingCommand(Hopper hopper, double timeToWait){
		this.hopper = hopper;
		this.timeToWait = timeToWait;
	}

	@Override
	public void initialize() {
		hopper.setShotSpacingTime(timeToWait);
	}
}
