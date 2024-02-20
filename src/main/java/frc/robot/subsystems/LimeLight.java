
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class Limelight extends SubsystemBase {

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry limelight_tx = limelight.getEntry("tx"); // Target x offset from crosshair
  NetworkTableEntry limelight_ty = limelight.getEntry("ty"); // Target y offset from crosshair
  NetworkTableEntry limelight_ta = limelight.getEntry("ta"); // Target area (0% - 100%)
  NetworkTableEntry limelight_tv = limelight.getEntry("tv"); // Number of valid targets

  NetworkTableEntry targetpose_cameraspace = limelight.getEntry("targetpose_cameraspace");
  NetworkTableEntry robotPose_wpiBlue = limelight.getEntry("botpose_wpiblue"); 
  // Transformation in fieldspace (blue alliance) as an array. 
  // Translation (X, Y, Z), Rotation (Roll, Pitch, Yaw), Total Latency (Capture latency + Pipline latency)
  // [X, Y, Z, Roll, Pitch, Yaw, (cl + tl)], length: 7



  public boolean getLimeLightTV() {
    return limelight_tv.getDouble(0.0) == 1;
  }

  public double[] robotPose_FieldSpace() {
    return robotPose_wpiBlue.getDoubleArray(new double[7]); // Get new array of type double and size 7
  } 

  public Pose2d getRoboPose() {
    double[] limelightData = robotPose_FieldSpace();
    Pose2d coordinates = new Pose2d(limelightData[0], limelightData[1], new Rotation2d(Units.degreesToRadians(limelightData[5]))); // Create new pose using x, y, and yaw
    return coordinates;
  }

  public double getRoboPoseLatency() {
    double[] limelightData = robotPose_FieldSpace();
    return limelightData[6];
  }


  public Limelight() {}

  @Override
  public void periodic() {}
}
