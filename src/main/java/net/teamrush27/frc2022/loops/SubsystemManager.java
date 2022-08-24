package net.teamrush27.frc2022.loops;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ForkJoinPool;
import java.util.concurrent.TimeUnit;
import net.teamrush27.frc2022.subsystems.Subsystem;
import net.teamrush27.frc2022.util.CrashTrackingRunnable;

public class SubsystemManager {

	private final ForkJoinPool threadPool = new ForkJoinPool(3);
	private List<Subsystem> subsystems = new ArrayList<>();
	public final double period;
	private final Notifier notifier;
	private final Object taskRunningLock = new Object();
	private boolean running;
	private double timestamp = 0;
	private double dT = 0;


	public SubsystemManager(double period){
		var ss = this;
		CrashTrackingRunnable runnable = new CrashTrackingRunnable() {
			@Override
			public void runCrashTracked() {
				synchronized (taskRunningLock) {
					if (running) {
						double now = Timer.getFPGATimestamp();

						ss.run();

						dT = now - timestamp;
						timestamp = now;
					}
				}
			}
		};

		this.period = period;
		notifier = new Notifier(runnable);
		running = false;
	}

	public void setSubsystems(Subsystem... subsystems) {
		this.subsystems = Arrays.asList(subsystems);
	}

	public boolean checkSubsystems() {

		boolean returnValue = true;

		for (Subsystem s : subsystems) {
			returnValue &= s.checkSystem();
		}

		return returnValue;
	}

	public synchronized void stop() {
		if(running){
			System.out.println("Stopping subsystem loops");
			subsystems.forEach(Subsystem::stop);
			notifier.stop();

			synchronized (taskRunningLock) {
				running = false;
				timestamp = Timer.getFPGATimestamp();
				for (Subsystem subsystem : subsystems) {
					System.out.println("Stopping " + subsystem.getId());
					subsystem.stop();
				}
			}
		}
	}

	public void stopSubsystems() {
		subsystems.forEach(Subsystem::stop);
	}

	public synchronized void run() {
		double ost = Timer.getFPGATimestamp();

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.readPeriodicInputs(st);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
//				DriverStation.reportError(String.format("%s.readPeriodicInputs took too long: %s", subsystem.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		threadPool.submit(() -> subsystems.parallelStream().forEach(loop -> {
			double st = Timer.getFPGATimestamp();
			loop.processLoop(st);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
//				DriverStation.reportError(String.format("%s.onLoop took too long: %s", loop.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.writePeriodicOutputs(st);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
//				DriverStation.reportError(String.format("%s.writePeriodicOutputs took too long: %s", subsystem.getId(), et - st), false);
			}
		}));

		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);

		var dt = Timer.getFPGATimestamp() - ost;
		if(dt > .02){
			DriverStation.reportError(String.format("Loop overrun [%s], skipping telemetry...",dt), false);
			return;
		}

		threadPool.submit(() -> subsystems.parallelStream().forEach(subsystem -> {
			double st = Timer.getFPGATimestamp();
			subsystem.outputTelemetry(timestamp);
			double et = Timer.getFPGATimestamp();

			if (et - st > 0.01) {
//				DriverStation.reportError(String.format("%s.outputTelemetry took too long: %s", subsystem.getId(), et - st), false);
			}
		}));
		threadPool.awaitQuiescence(10, TimeUnit.MILLISECONDS);
	}

	public synchronized void start() {
		if (!running) {
			System.out.println("Starting loops");

			synchronized (taskRunningLock) {
				timestamp = Timer.getFPGATimestamp();
				running = true;
			}

			notifier.startPeriodic(period);
		}
	}
}
