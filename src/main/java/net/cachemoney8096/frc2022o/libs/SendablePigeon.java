package net.cachemoney8096.frc2022o.libs;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendablePigeon extends Pigeon2 implements Sendable {

  /**
   * Create a Pigeon object that communicates with Pigeon on CAN Bus.
   *
   * @param deviceNumber CAN Device Id of Pigeon [0,62]
   */
  public SendablePigeon(int deviceNumber) {
    super(deviceNumber, "");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Yaw Degrees CCW Pos",
        () -> getYaw(),
        (double yawDegCcw) -> {
          setYaw(yawDegCcw);
        });
  }

  /**
   * Return the heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   *
   * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
   * 360 to 0 on the second time around.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * <p>This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }
}
