package net.cachemoney8096.frc2022o.libs_3005.vendor.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class SparkMaxUtils {

  /**
   * @param error API return value
   * @return
   */
  public static int check(REVLibError error) {
    return error == REVLibError.kOk ? 0 : 1;
  }

  public static REVLibError setDefaultsForNeo(CANSparkMax sparkMax) {
    REVLibError error = REVLibError.kOk;

    sparkMax.setSmartCurrentLimit(60);
    sparkMax.setSecondaryCurrentLimit(90);

    return error;
  }

  public static REVLibError setDefaultsForNeo500(CANSparkMax sparkMax) {
    // TODO: Use this
    REVLibError error = REVLibError.kOk;

    sparkMax.setSmartCurrentLimit(25);
    sparkMax.setSecondaryCurrentLimit(35);

    return error;
  }

  public static class UnitConversions {
    public static REVLibError setDegreesFromGearRatio(
        RelativeEncoder sparkMaxEncoder, double ratio) {
      double degreesPerRotation = 360.0 / ratio;
      double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
    }

    public static REVLibError setRadsFromGearRatio(RelativeEncoder sparkMaxEncoder, double ratio) {
      double radsPerRotation = (2.0 * Math.PI) / ratio;
      double radsPerRotationPerSecond = radsPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
    }
  }

  public static String faultWordToString(short faults) {
    if (faults == 0) {
      return "";
    }

    StringBuilder builder = new StringBuilder();
    int faultsInt = faults;
    for (int i = 0; i < 16; i++) {
      if (((1 << i) & faultsInt) != 0) {
        builder.append(CANSparkMax.FaultID.fromId(i).toString());
        builder.append(" ");
      }
    }
    return builder.toString();
  }
}
