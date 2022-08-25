package net.cachemoney8096.frc2022o.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.cachemoney8096.frc2022o.subsystems.Hopper;

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
