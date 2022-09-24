package net.cachemoney8096.frc2022o.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.libs.PicoColorSensor;
import net.cachemoney8096.frc2022o.libs.CargoColor;
import net.cachemoney8096.frc2022o.libs.CargoColorDifferentiator;
import edu.wpi.first.wpilibj.Timer;

public class Intake implements Subsystem {

  // Actuators
  private final CANSparkMax intakeMotorOne;
  private final CANSparkMax intakeMotorTwo;
  private final CANSparkMax intakeMotorThree;

  // Sensors
  private final DigitalInput cargoSensor;
  private PicoColorSensor colorSensor;

  // Members
  private CargoColorDifferentiator cargoColorDifferentiator;
  private Optional<PicoColorSensor.RawColor> lastColorSeen;
  private Optional<PicoColorSensor.RawColor> ownedCargoColor;
  private Optional<PicoColorSensor.RawColor> colorPassedToindexer;
  private Optional<Timer> ejectTimer;

  public Intake() {
    intakeMotorOne = new CANSparkMax(RobotMap.INTAKE_MOTOR_ONE_ID, MotorType.kBrushless);
    intakeMotorOne.restoreFactoryDefaults();
    intakeMotorOne.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorOne.setInverted(false);

    intakeMotorTwo = new CANSparkMax(RobotMap.INTAKE_MOTOR_TWO_ID, MotorType.kBrushless);
    intakeMotorTwo.restoreFactoryDefaults();
    intakeMotorTwo.setIdleMode(CANSparkMax.IdleMode.kCoast);

    intakeMotorThree = new CANSparkMax(RobotMap.INTAKE_MOTOR_THREE_ID, MotorType.kBrushless);
    intakeMotorThree.restoreFactoryDefaults();
    intakeMotorThree.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorThree.follow(intakeMotorTwo);

    cargoSensor = new DigitalInput(RobotMap.INTAKE_CARGO_DIO);
    colorSensor = new PicoColorSensor();
  }

  @Override
  public void periodic() {
    // if all three colors return 0, reinstantiate the color sensor
    // based on https://www.chiefdelphi.com/t/rev-color-sensor-stops-outputting/405153/3
    PicoColorSensor.RawColor sensorColor = colorSensor.getRawColor0();
    if (sensorColor.red == 0 && sensorColor.green == 0 && sensorColor.blue == 0)
    {
      colorSensor = new PicoColorSensor();
    }

    
    // check for colors
    // if (colorSensor.getProximity0() < ???)
    // {
    // CargoColor cargoColor = cargoColorDifferentiator.whatColor(sensorColor);
    // color = new PicoColorSensor.RawColor();
    // colorSensor.getRawColor0(color);
    // check if the color is valid?
    // lastColorSeen = color;
    // }

    // check for a ball

  }

  public void updateAllianceColor() {
    cargoColorDifferentiator.updateAllianceColor();
  }

  public boolean hasCargo() {
    return cargoSensor.get();
  }

  public void intakeBall() {
    // run all forward
    // if we see a wrong-color, run 2-3 out for a second?
    // if we last saw a wrong color, run 2-3 out for a couple seconds?
  }
}
