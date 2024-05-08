package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPosForLimelightCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final LimeLight limelight;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private Pose2d targetPose;
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

    // Moves swerve to targetPose. Returns true when it reaches the position
    public boolean moveSwerve() {

        // Calculate the error between current position and targetPos
        double xError = targetPose.getX() - swerveSubsystem.getPose().getX();
        double yError = targetPose.getY() - swerveSubsystem.getPose().getY();
        double turnError = (targetPose.getRotation().minus(swerveSubsystem.getRotation2d())).getRadians();

        // Calculate the angle and speed to move swerve to targetPose
        double angle = Math.atan2(yError, xError);
        double speed = (xError * xError + yError * yError) * 15 * (1 / AutoConstants.MAX_SPEED_METERS_PER_SEC) * (1 / AutoConstants.MAX_SPEED_METERS_PER_SEC) * (1 / AutoConstants.MAX_SPEED_METERS_PER_SEC) > AutoConstants.MAX_SPEED_METERS_PER_SEC ? AutoConstants.MAX_SPEED_METERS_PER_SEC : (xError * xError + yError * yError) * 7;

        // Calculate xSpeed, ySpeed, and turnSpeed
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = Math.sin(angle) * speed;
        double turnSpeed = (Math.abs(turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005) < AutoConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC) ? turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005 : AutoConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * Math.signum(turnError);

        // Limit xSpeed, ySpeed, and turnSpeed to min speeds
        xSpeed = Math.abs(xSpeed) > AutoConstants.MIN_SPEED ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.MIN_SPEED ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.MIN_TURN_SPEED_RAD ? turnSpeed : 0.0;

        // Limit xSpeed, ySpeed, and turnSpeed to max acceleration
        xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.MAX_SPEED_METERS_PER_SEC;
        ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.MAX_SPEED_METERS_PER_SEC;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;

        // Set the module states to move swerve to targetPose with field orientation
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        // If swerve is close to targetPose, then return true
        if (Math.sqrt(xError * xError + yError * yError) < AutoConstants.TOLERANCE_METERS && Math.abs(turnError * 180 / Math.PI) < AutoConstants.TOLERANCE_DEGREES) {
            return true; // Check if you can remove stop and start ticks now
        }

        // If swerve is not close to targetPose, then return false
        return false;
    }

    public boolean isFinished() {
        // If current position in list is greater than or equal to the length of the targetPath and repeatPath is false, then return true and exit command
        return limelight.seesMultipleTargets() || isDone;
    }
}
