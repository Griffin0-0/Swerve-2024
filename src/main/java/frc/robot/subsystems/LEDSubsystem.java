package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class LEDSubsystem extends SubsystemBase {

  Spark blinkinController;

  private static double red = 0.61;
  private static double gold = 0.67;
  private static double green = 0.77;
  private static double blue = 0.87;
  private static double violet = 0.91;
  private static double white = 0.93;

  private static double defaultColor = white;
  public double currentColor = 0.93;

  public LEDSubsystem() {
    blinkinController = new Spark(1);
  }

  // public void init() {
  //   if (DriverStation.getAlliance().get() == Alliance.Red) {
  //     defaultColor = red;
  //   } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
  //     defaultColor = blue;
  //   }
  // }

  public void setColor(double color) {
    blinkinController.set(color);
  }

  public void setShoot() {
    currentColor = green;
  }

  public void setIntake() {
    currentColor = violet;
  }

  public void setAmp() {
    currentColor = gold;
  }

  public void setDefault() {
    currentColor = defaultColor;
  }

  @Override
  public void periodic() {
    blinkinController.set(currentColor);
  }
}
