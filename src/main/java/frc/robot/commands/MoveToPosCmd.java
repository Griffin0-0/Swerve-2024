package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveToPosCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int currentStopTick;
    private Pose2d[] targetPath;
    private boolean stopAtEnd;
    private boolean repeatPath;
    private Pose2d targetPose;
    private int tick = 0;
    private int startTick = -AutoConstants.kAutoStartCheckTicks;
    private int currentPosInList = 0;

    public MoveToPosCmd(SwerveSubsystem swerveSubsystem, Pose2d[] targetPath, boolean stopAtEnd, boolean repeatPath) {
        this.swerveSubsystem = swerveSubsystem;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAngularAccelerationUnitsPerSecond);
        this.targetPath = targetPath;
        this.repeatPath = repeatPath;
        this.stopAtEnd = stopAtEnd;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        tick++;
        SmartDashboard.putNumber("Auto Ticks", tick);

        // If the current position in the list is less than the length of the targetPath, then move swerve to targetPose
        if (currentPosInList < targetPath.length) {

            targetPose = targetPath[currentPosInList];

            if (moveSwerve()) {

                // If swerve reached targetPose, go to next pos in list
                currentPosInList++;
                startTick = -AutoConstants.kAutoStartCheckTicks;
                SmartDashboard.putNumber("CurrentPos", currentPosInList);

            }

        } else if (repeatPath) {
            // If repeatPath is true, then reset the current position in the list to 0
            currentPosInList = 0;
        }
    }

    // Moves swerve to targetPose. Returns true when it reaches the position
    public boolean moveSwerve() {
        startTick++;
        SmartDashboard.putNumber("startTick", startTick);

        // Calculate the error between current position and targetPos
        double xError = targetPose.getX() - swerveSubsystem.getPose().getX();
        double yError = targetPose.getY() - swerveSubsystem.getPose().getY();
        double turnError = (targetPose.getRotation().minus(swerveSubsystem.getRotation2d())).getRadians();

        // Calculate the angle and speed to move swerve to targetPose
        double angle = Math.atan2(yError, xError);
        double speed = (xError * xError + yError * yError) * 15 * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) > AutoConstants.kAutoMaxSpeedMetersPerSecond ? AutoConstants.kAutoMaxSpeedMetersPerSecond : (xError * xError + yError * yError) * 7;

        // Calculate xSpeed, ySpeed, and turnSpeed
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = Math.sin(angle) * speed;
        double turnSpeed = (Math.abs(turnError * Math.sqrt(Math.abs(turnError * 40)) * -0.005) < AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond) ? turnError * Math.sqrt(Math.abs(turnError * 40)) * -0.005 : AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond * Math.signum(turnError);

        // Limit xSpeed, ySpeed, and turnSpeed to min speeds
        xSpeed = Math.abs(xSpeed) > AutoConstants.kAutoMinSpeed ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.kAutoMinSpeed ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.kAutoMinTurnSpeedRadians ? turnSpeed : 0.0;

        // Limit xSpeed, ySpeed, and turnSpeed to max acceleration
        xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.kAutoMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.kAutoMaxSpeedMetersPerSecond;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("turnSpeed", turnSpeed);

        // Set the module states to move swerve to targetPose with field orientation
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
        
        // If swerve is close to targetPose, then return true
        if (Math.sqrt(xError * xError + yError * yError) < AutoConstants.kAutoToleranceMeters && Math.abs(turnError * 180 / Math.PI) < AutoConstants.kAutoToleranceDegrees) {
            return true; // Check if you can remove stop and start ticks now
        }

        // If swerve is not close to targetPose, then return false
        return false;
    }

    public boolean isFinished() {
        // If current position in list is greater than or equal to the length of the targetPath and repeatPath is false, then return true and exit command
        if (currentPosInList >= targetPath.length && !repeatPath) {
            return true;
        }
        return false;
    }
}
