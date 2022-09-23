package net.cachemoney8096.frc2022o.libs;

import net.cachemoney8096.frc2022o.libs.AbsoluteEncoderFromDio;
import net.cachemoney8096.frc2022o.libs.AbsoluteEncoderFromDioConfiguration;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

// Basically a copy of CanCoderAbsoluteConfiguration from:
// src\main\java\com\swervedrivespecialties\swervelib\ctre\CanCoderAbsoluteConfiguration.java
public class AbsoluteEncoderFromDioFactoryBuilder {
  public AbsoluteEncoderFactory<AbsoluteEncoderFromDioConfiguration> build() {
    return configuration -> {

    AbsoluteEncoderFromDio encoder = new AbsoluteEncoderFromDio(configuration.getId(), configuration.getOffset());

      return encoder;
    };
  }
}
