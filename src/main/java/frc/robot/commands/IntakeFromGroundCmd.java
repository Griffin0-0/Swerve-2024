package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

public class IntakeFromGroundCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int tick = 0;
    // private int currentStopTick;
    // private int startTick = -AutoConstants.kAutoStartCheckTicks;
    private int currentShootTick = Constants.AutoConstants.kAutoSpeakerShotCheckTicks; 
    private double shootingDistance = 1.75;
    private Pose2d targetPose;
    private Translation2d targetTranslation;

    public IntakeFromGroundCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, Translation2d targetTranslation) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.targetTranslation = targetTranslation;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        tick++;
        SmartDashboard.putNumber("Speaker Ticks", tick);

        Translation2d difference = new Translation2d(targetTranslation.getX() - swerveSubsystem.getPose().getTranslation().getX(), targetTranslation.getY() - swerveSubsystem.getPose().getTranslation().getY());
        new Rotation2d();
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
        
        targetPose = new Pose2d(targetTranslation, angle);

        moveSwerve();

        if (swerveSubsystem.pose.getTranslation().getDistance(targetPose.getTranslation()) < 1) {
            intakeSubsystem.runIntake(IntakeConstants.kIntakeMotorSpeed);
        }

        if (swerveSubsystem.pose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1) {
            intakeSubsystem.stop();
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

        return false;
    }

    public boolean isFinished() {
        // if (tick > 10000) {
        //     return true;
        // }
        return false;
    }
}