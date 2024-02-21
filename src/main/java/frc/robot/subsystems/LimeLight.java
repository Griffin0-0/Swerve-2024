package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.core.*;

public class LimeLight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    JsonNode node;
    Pose2d botposeRed = new Pose2d(0,0,new Rotation2d());
    Pose2d botposeBlue = new Pose2d(0,0,new Rotation2d());

    public Pose2d runLimeLight(boolean isAllianceBlue) {
        final String json = table.getEntry("json").getString(null);

        try {node = new ObjectMapper().readTree(json);} catch (JsonProcessingException e) {e.printStackTrace();};

        botposeRed = new Pose2d(node.get("Results").get("botpose_wpired").get(0).asDouble(), node.get("Results").get("botpose_wpired").get(1).asDouble(), Rotation2d.fromDegrees(node.get("Results").get("botpose_wpired").get(5).asDouble()));
        botposeBlue = new Pose2d(node.get("Results").get("botpose_wpiblue").get(0).asDouble(), node.get("Results").get("botpose_wpiblue").get(1).asDouble(), Rotation2d.fromDegrees(node.get("Results").get("botpose_wpiblue").get(5).asDouble()));

        SmartDashboard.putNumber("Number of Targets", node.get("Results").get("Fiducial").size());

        if (isAllianceBlue) {
            return botposeBlue;
        }
        else {
            return botposeRed;
        }
    }

    public boolean seesTargets() {
        final String json = table.getEntry("json").getString(null);

        try {node = new ObjectMapper().readTree(json);} catch (JsonProcessingException e) {e.printStackTrace();};

        return !node.get("Results").get("Fiducial").isEmpty();
    }

    public boolean seesMultipleTargets() {
        final String json = table.getEntry("json").getString(null);

        try {node = new ObjectMapper().readTree(json);} catch (JsonProcessingException e) {e.printStackTrace();};

        return node.get("Results").get("Fiducial").size() > 1;
    }

    public boolean seesOneGoodTarget() {
        final String json = table.getEntry("json").getString(null);

        try {node = new ObjectMapper().readTree(json);} catch (JsonProcessingException e) {e.printStackTrace();};

        boolean goodTrack = (table.getEntry("ta").getDouble(0) > 5000);

        SmartDashboard.putNumber("ta", table.getEntry("ta").getDouble(0));

        return node.get("Results").get("Fiducial").size() == 1 && goodTrack;
    }
    
}
