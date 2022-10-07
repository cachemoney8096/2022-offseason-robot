package net.cachemoney8096.frc2022o.libs_3005.electromechanical;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class EncoderSupplier implements Encoder {
  DoubleSupplier m_velocitySupplier;
  DoubleSupplier m_positionSupplier;
  Consumer<Double> m_positionSetter;

  public EncoderSupplier(
      DoubleSupplier velocitySupplier,
      DoubleSupplier positionSupplier,
      Consumer<Double> positionSetter) {
    m_velocitySupplier = velocitySupplier;
    m_positionSupplier = positionSupplier;
    m_positionSetter = positionSetter;
  }

  @Override
  public double getVelocity() {
    return m_velocitySupplier.getAsDouble();
  }

  @Override
  public double getPosition() {
    return m_positionSupplier.getAsDouble();
  }

  @Override
  public void setPosition(double position) {
    m_positionSetter.accept(position);
  }
}
