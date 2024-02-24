package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class LEDSubsystem extends SubsystemBase {

  Spark blinkinController;

  private static double red = 0.61;
  private static double green = 0.77;
  private static double blue = 0.87;
  private static double black = 0.99;
  private static double white = 0.93;

  private static double defaultColor = blue;
  public double currentColor = 0.93;

  public LEDSubsystem() {
    blinkinController = new Spark(1);
  }

  public void setColor(double color) {
    blinkinController.set(color);
  }

  public void setShoot() {
    currentColor = green;
  }

  public void setIntake() {
    currentColor = red;
  }

  public void setDefault() {
    currentColor = defaultColor;
  }

  @Override
  public void periodic() {
    blinkinController.set(currentColor);
  }
}
