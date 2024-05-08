package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPosCmd extends MobileCommand {

    private final Pose2d[] targetPath;
    private final boolean repeatPath;

    private int currentPosInList = 0;

    public MoveToPosCmd(SwerveSubsystem swerveSubsystem, Pose2d[] targetPath, boolean repeatPath) {
        this.swerveSubsystem = swerveSubsystem;
        this.xLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC);
        this.targetPath = targetPath;
        this.repeatPath = repeatPath;
        
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        // If the current position in the list is less than the length of the targetPath, then move swerve to targetPose
        if (currentPosInList < targetPath.length) {

            targetPose = targetPath[currentPosInList];

            if (moveSwerve()) {
                // If swerve reached targetPose, go to next pos in list
                currentPosInList++;
            }

        } else if (repeatPath) {
            // If repeatPath is true, then reset the current position in the list to 0
            currentPosInList = 0;
        }
    }

    public boolean isFinished() {
        // If current position in list is greater than or equal to the length of the targetPath and repeatPath is false, then return true and exit command
        return (currentPosInList >= targetPath.length && !repeatPath);
    }
}
