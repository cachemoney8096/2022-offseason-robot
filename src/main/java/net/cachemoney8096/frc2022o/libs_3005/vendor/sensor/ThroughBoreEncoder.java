package net.cachemoney8096.frc2022o.libs_3005.vendor.sensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import net.cachemoney8096.frc2022o.libs_3005.electromechanical.AbsoluteEncoder;
import net.cachemoney8096.frc2022o.libs_3005.util.SendableHelper;

public class ThroughBoreEncoder implements AbsoluteEncoder {
  private final DigitalInput m_digitalSource;
  private final DutyCycle m_dutyCycle;
  private final DutyCycleEncoder m_encoder;
  private double m_offset = 0.0;
  private final double m_scalar;
  private final boolean m_inverted;

  public ThroughBoreEncoder(int channel, double offset, double scalar, boolean invert) {
    m_digitalSource = new DigitalInput(channel);
    m_dutyCycle = new DutyCycle(m_digitalSource);
    m_encoder = new DutyCycleEncoder(m_dutyCycle);
    m_offset = offset;
    m_scalar = scalar;
    m_inverted = invert;
  }

  public ThroughBoreEncoder(int channel) {
    this(channel, 0.0, 1.0, false);
  }

  public DigitalSource getDigitalSource() {
    return m_digitalSource;
  }

  @Override
  public double getPosition() {
    double result;
    if (m_inverted) {
      result = ((1.0 - m_dutyCycle.getOutput()) * m_scalar - m_offset);
    } else {
      result = (m_dutyCycle.getOutput() * m_scalar - m_offset);
    }
    return MathUtil.inputModulus(result, 0, m_scalar);
  }

  @Override
  public boolean isConnected() {
    return m_encoder.isConnected();
  }

  public double dutyCycle() {
    if (m_inverted) {
      return 1.0 - m_dutyCycle.getOutput();
    }
    return m_dutyCycle.getOutput();
  }

  public double getRelativePosition() {
    return m_encoder.get();
  }

  @Override
  public void setPositionOffset(double offset) {
    m_offset = offset;
  }

  public void setPositionOffsetToCurrentPosition() {
    m_offset = -getRelativePosition();
  }

  public double getPositionOffset() {
    return m_offset;
  }

  public double getPositionScalar() {
    return m_scalar;
  }

  public boolean getPositionInverted() {
    return m_inverted;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_encoder.initSendable(builder);
    SendableHelper.addChild(builder, this, m_digitalSource, "Digital Source");
    builder.addDoubleProperty("Absolute Position", () -> getPosition(), null);
    builder.addDoubleProperty("Raw Sensor Output", this::dutyCycle, null);
    builder.addDoubleProperty("Offset", () -> m_offset, this::setPositionOffset);
    builder.addDoubleProperty("Scalar", () -> m_scalar, null);
    builder.addBooleanProperty("Inverted", () -> m_inverted, null);
  }
}
