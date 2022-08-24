package net.cachemoney8096.frc2022o.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.cachemoney8096.frc2022o.subsystems.CargoManager;

public class CargoManagerWantedStateCommand extends InstantCommand {

	private final CargoManager cargoManager;
	private final CargoManager.WantedState wantedState;

	public CargoManagerWantedStateCommand(final CargoManager cargoManager, CargoManager.WantedState wantedState){
		this.cargoManager = cargoManager;
		this.wantedState = wantedState;
	}

	@Override
	public void initialize() {
		cargoManager.setWantedState(wantedState);
	}
}
