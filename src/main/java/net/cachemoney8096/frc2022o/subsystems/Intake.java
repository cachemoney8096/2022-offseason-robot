package net.cachemoney8096.frc2022o.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.cachemoney8096.frc2022o.Calibrations;
import net.cachemoney8096.frc2022o.RobotMap;
import net.cachemoney8096.frc2022o.libs.PicoColorSensor;
import net.cachemoney8096.frc2022o.libs.CargoColorDifferentiator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {

  // Actuators
  private final CANSparkMax intakeMotorOne;
  private final CANSparkMax intakeMotorTwo;
  private final CANSparkMax intakeMotorThree;
  private final Compressor compressor;
  private final Solenoid intakeSolenoidLeft;
  private final Solenoid intakeSolenoidRight;

  // Sensors
  private final DigitalInput cargoSensor;
  private PicoColorSensor colorSensor;

  // Members
  private CargoColorDifferentiator cargoColorDifferentiator = new CargoColorDifferentiator();
  private Optional<PicoColorSensor.RawColor> lastColorSeen;
  private Optional<PicoColorSensor.RawColor> ownedCargoColor;
  private Optional<PicoColorSensor.RawColor> colorPassedToindexer;
  private Optional<Timer> ejectTimer;

  public Intake() {
    intakeMotorOne = new CANSparkMax(RobotMap.INTAKE_MOTOR_ONE_ID, MotorType.kBrushless);
    intakeMotorOne.restoreFactoryDefaults();
    intakeMotorOne.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorOne.setInverted(
        false); // TODO see which way motors are facing and invert such that positive = in

    intakeMotorTwo = new CANSparkMax(RobotMap.INTAKE_MOTOR_TWO_ID, MotorType.kBrushless);
    intakeMotorTwo.restoreFactoryDefaults();
    intakeMotorTwo.setIdleMode(CANSparkMax.IdleMode.kCoast);

    intakeMotorThree = new CANSparkMax(RobotMap.INTAKE_MOTOR_THREE_ID, MotorType.kBrushless);
    intakeMotorThree.restoreFactoryDefaults();
    intakeMotorThree.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotorThree.follow(intakeMotorTwo);

    compressor = new Compressor(RobotMap.COMPRESSOR_MODULE_ID, PneumaticsModuleType.CTREPCM);
    intakeSolenoidLeft =
        new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.LEFT_INTAKE_SOLENOID_CHANNEL);
    intakeSolenoidRight =
        new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_INTAKE_SOLENOID_CHANNEL);

    cargoSensor = new DigitalInput(RobotMap.INTAKE_CARGO_DIO);
    colorSensor = new PicoColorSensor();
  }

  @Override
  public void periodic() {
    // if all three colors return 0, reinstantiate the color sensor
    // based on https://www.chiefdelphi.com/t/rev-color-sensor-stops-outputting/405153/3
    PicoColorSensor.RawColor sensorColor = colorSensor.getRawColor0();
    if (sensorColor.red == 0 && sensorColor.green == 0 && sensorColor.blue == 0) {
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

    // If we saw a wrong-colored ball, then set the timer
  }

  public void updateAllianceColor() {
    cargoColorDifferentiator.updateAllianceColor();
  }

  public boolean hasCargo() {
    return cargoSensor.get();
  }

  public void intakeCargo() {
    // run all forward
    // if we see a wrong-color, run 2-3 out for a second?
    // if we last saw a wrong color, run 2-3 out for a couple seconds?
    intakeMotorOne.set(Calibrations.INTAKE_ONE_POWER); // 3 follows 2
    if (ejectTimer.isEmpty()) // nothing to eject
    {
      intakeMotorTwo.set(Calibrations.INTAKE_TWO_POWER);
    } else if (ejectTimer.get().hasElapsed(Calibrations.EJECT_CARGO_FRONT_SECONDS)) // done ejecting
    {
      intakeMotorTwo.set(Calibrations.INTAKE_TWO_POWER);
      ejectTimer = Optional.empty();
    } else {
      intakeMotorTwo.set(Calibrations.INTAKE_EJECT_POWER);
    }
  }

  public void extendIntake() {
    intakeSolenoidLeft.set(true);
    intakeSolenoidRight.set(true);
  }

  public void retractIntake() {
    intakeSolenoidLeft.set(false);
    intakeSolenoidRight.set(false);
  }
}
