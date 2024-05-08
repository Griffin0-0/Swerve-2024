package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPosForLimelightCmd extends MobileCommand {

    private final LimeLight limelight;

    private boolean isDone = false;

    public MoveToPosForLimelightCmd(SwerveSubsystem swerveSubsystem, LimeLight limelight, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight;
        this.xLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC);
        this.targetPose = targetPose;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        if (moveSwerve()) {
            isDone = true;
        }
    }

    public boolean isFinished() {
        // If current position in list is greater than or equal to the length of the targetPath and repeatPath is false, then return true and exit command
        return limelight.seesMultipleTargets() || isDone;
    }
}
