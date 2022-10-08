package net.cachemoney8096.frc2022o.libs;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController extends edu.wpi.first.wpilibj.XboxController {

  public XboxController(int port) {
    super(port);
  }

  public JoystickButton A() {
    return new JoystickButton(this, Button.kA.value);
  }

  public JoystickButton B() {
    return new JoystickButton(this, Button.kB.value);
  }

  public JoystickButton Back() {
    return new JoystickButton(this, Button.kBack.value);
  }

  public JoystickButton BumperLeft() {
    return new JoystickButton(this, Button.kLeftBumper.value);
  }

  public JoystickButton BumperRight() {
    return new JoystickButton(this, Button.kRightBumper.value);
  }

  public JoystickButton Start() {
    return new JoystickButton(this, Button.kStart.value);
  }

  public JoystickButton StickLeft() {
    return new JoystickButton(this, Button.kLeftStick.value);
  }

  public JoystickButton StickRight() {
    return new JoystickButton(this, Button.kRightStick.value);
  }

  public JoystickButton X() {
    return new JoystickButton(this, Button.kX.value);
  }

  public JoystickButton Y() {
    return new JoystickButton(this, Button.kY.value);
  }

  public POVButton North() {
    return new POVButton(this, 0);
  }

  public POVButton East() {
    return new POVButton(this, 90);
  }

  public POVButton South() {
    return new POVButton(this, 180);
  }

  public POVButton West() {
    return new POVButton(this, 270);
  }

  public Trigger TriggerRight() {
    return new Trigger(() -> this.getRightTriggerAxis() > 0.1);
  }

  public Trigger TriggerLeft() {
    return new Trigger(() -> this.getLeftTriggerAxis() > 0.1);
  }
}
