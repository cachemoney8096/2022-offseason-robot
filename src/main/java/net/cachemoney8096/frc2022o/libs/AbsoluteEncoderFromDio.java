package net.cachemoney8096.frc2022o.libs;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.lang.Math;

// A class for a duty cycle absolute encoder on a DIO port. Supports a static offset.
// Good for something like the REV hex encoder.
public class AbsoluteEncoderFromDio implements AbsoluteEncoder{
    
  private final DutyCycleEncoder hoodAbsoluteEncoder;
  private final double offsetRad;

  AbsoluteEncoderFromDio(int dutyCycleEncoderDio, double inputOffsetRad){
    hoodAbsoluteEncoder = new DutyCycleEncoder(dutyCycleEncoderDio);
    offsetRad = inputOffsetRad;
  }

  // Returns the angle of the encoder in radians
  public double getAbsoluteAngle() {
      return (hoodAbsoluteEncoder.get() * 2 * Math.PI) + offsetRad;
  }
}
