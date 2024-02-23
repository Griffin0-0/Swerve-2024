package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class LEDSubsystem extends SubsystemBase {

  Spark blinkinController;

  private static double orange = 0.65;
  private static double yellow = 0.69;
  private static double blue = 0.87;
  private static double black = 0.99;
  private static double white = 0.93;

  private static double defaultColor = 0.87;
  public double currentColor = 0.93;

  public LEDSubsystem() {
    blinkinController = new Spark(1);
  }

  public void setColor(double color) {
    blinkinController.set(color);
  }

  public void setShoot() {
    currentColor = orange;
  }

  public void setIntake() {
    currentColor = yellow;
  }

  public void setDefault() {
    currentColor = defaultColor;
  }

  @Override
  public void periodic() {
    blinkinController.set(currentColor);
  }
}
