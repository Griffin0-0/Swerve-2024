package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDSubsystem extends SubsystemBase {

  Spark blinkinController;

  public LEDSubsystem() {
    blinkinController = new Spark(1);
  }

  public void setColor(double color) {
    blinkinController.set(color);
  }

  public void setShoot() {
    blinkinController.set(0.5);
  }
}
