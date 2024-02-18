package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

public class FireAtSpeakerCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int tick = 0;
    private int currentStopTick;
    private int startTick = -AutoConstants.kAutoStartCheckTicks;
    private double shootingDistance = 1.75;
    private Pose2d speakerPos = new Pose2d(0.1,5.45, Rotation2d.fromDegrees(0));
    private Pose2d targetPose;

    public FireAtSpeakerCmd(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        tick++;
        SmartDashboard.putNumber("Speaker Ticks", tick);

        Pose2d difference = new Pose2d(speakerPos.getX() - swerveSubsystem.getPose().getX(), speakerPos.getY() - swerveSubsystem.getPose().getY(), Rotation2d.fromDegrees(0));
        new Rotation2d();
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
        targetPose = new Pose2d(speakerPos.getX() - Math.cos(angle.getRadians()) * shootingDistance, speakerPos.getY() - Math.sin(angle.getRadians()) * shootingDistance, angle.minus(Rotation2d.fromDegrees(180)));
        SmartDashboard.putString("Speaker Target Pose", targetPose.toString());

        if (swerveSubsystem.pose.getTranslation().getDistance(targetPose.getTranslation()) < 1.5) {
            shooterSubsystem.spinOut();
        } else {
            shooterSubsystem.stop();
        }

        if (moveSwerve()) {
            intakeSubsystem.toggleIntake();
        } else {
            intakeSubsystem.sendStop();
        }
    }

    public boolean moveSwerve() {
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
            if (((Math.abs(xSpeed) <= 0.04) && (Math.abs(ySpeed) <= 0.04) && (Math.abs(turnSpeed) == 0) && (startTick > 0))) {
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
        // if (tick > 10000) {
        //     return true;
        // }
        return false;
    }
}
