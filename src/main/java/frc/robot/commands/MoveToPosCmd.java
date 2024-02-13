package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
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

        if (currentPosInList < targetPath.length) {
            targetPose = targetPath[currentPosInList];
            if (moveSwerve()) {
                currentPosInList++;
                startTick = -AutoConstants.kAutoStartCheckTicks;
                SmartDashboard.putNumber("CurrentPos", currentPosInList);
            }
        } else if (repeatPath) {
            currentPosInList = 0;
        }
    }

    // Moves swerve to whatever the targetPose is. Returns true when it reaches the position
    public boolean moveSwerve() {
        startTick++;
        SmartDashboard.putNumber("startTick", startTick);
        double xError = targetPose.getX() - swerveSubsystem.getPose().getX();
        double yError = targetPose.getY() - swerveSubsystem.getPose().getY();
        double turnError = (targetPose.getRotation().minus(swerveSubsystem.getRotation2d())).getRadians() * 40;

        double angle = Math.atan2(yError, xError);
        double speed = (xError * xError + yError * yError) * 15 * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) > AutoConstants.kAutoMaxSpeedMetersPerSecond ? AutoConstants.kAutoMaxSpeedMetersPerSecond : (xError * xError + yError * yError) * 7;

        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = Math.sin(angle) * speed;

        double turnSpeed = (Math.abs(turnError * Math.sqrt(Math.abs(turnError)) * -0.005) < AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond) ? turnError * Math.sqrt(Math.abs(turnError)) * -0.005 : AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond * Math.signum(turnError);


        xSpeed = Math.abs(xSpeed) > AutoConstants.kAutoMinSpeed ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.kAutoMinSpeed ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.kAutoMinTurnSpeedRadians ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.kAutoMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.kAutoMaxSpeedMetersPerSecond;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("turnSpeed", turnSpeed);
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
        

        if ((Math.abs(xSpeed) <= (0.4 * Math.sqrt(AutoConstants.kAutoMaxSpeedMetersPerSecond))) && (Math.abs(ySpeed) <= 0.4 * Math.sqrt(AutoConstants.kAutoMaxSpeedMetersPerSecond)) && (Math.abs(turnSpeed) <= 0.2 * Math.sqrt(AutoConstants.kAutoMaxSpeedMetersPerSecond)) && (startTick > 0)) {
            if (currentPosInList == targetPath.length - 1 && ((Math.abs(xSpeed) <= 0.04) && (Math.abs(ySpeed) <= 0.04) && (Math.abs(turnSpeed) == 0) && (startTick > 0)) && stopAtEnd) {
                if (currentStopTick < AutoConstants.kAutoStoppedCheckTicks) {
                    currentStopTick++;
                } else {
                    return true;
                }
            }
            return true;
        } else {
            currentStopTick = 0;
        }
        return false;
    }

    public boolean isFinished() {
        if (currentPosInList >= targetPath.length && !repeatPath) {
            return true;
        }
        return false;
    }
}
