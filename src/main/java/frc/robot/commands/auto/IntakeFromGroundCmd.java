package frc.robot.commands.auto;

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
    private int collectedCheckTick = AutoConstants.kAutoGroundIntakeCheckTicks;
    private Pose2d targetPose;
    private Translation2d targetTranslation;
    private Boolean isDone = false;
    private double collectionDistance = 0.75;

    public IntakeFromGroundCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, Translation2d targetTranslation) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.targetTranslation = targetTranslation;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

        // Calculate the difference between the note position and swerve's position
        Translation2d difference = new Translation2d(targetTranslation.getX() - swerveSubsystem.getPose().getX(), targetTranslation.getY() - swerveSubsystem.getPose().getY());

        new Rotation2d();
        // Calculate the angle between the note and swerve
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
        SmartDashboard.putNumber("Target Angle", angle.getDegrees());

        // Calculate the target position for swerve to move to
        this.targetPose = new Pose2d(targetTranslation.getX() - Math.cos(angle.getRadians()) * collectionDistance, targetTranslation.getY() - Math.sin(angle.getRadians()) * collectionDistance, angle);
    }

    @Override
    public void execute() {
        tick++;
        SmartDashboard.putNumber("Speaker Ticks", tick);
        
        if (moveSwerve()) {
            // If swerve reached targetPose, start collecting note
            collectedCheckTick--;
        }

        // Once made sure swerve has collected note from ground, exit command
        if (collectedCheckTick <= AutoConstants.kAutoGroundIntakeCheckTicks / 4) {
            intakeSubsystem.stopIntake();
            intakeSubsystem.intakeUp();
        } else {
            intakeSubsystem.runIntake(IntakeConstants.kIntakeMotorSpeed);
            intakeSubsystem.intakeDown();
        }

        if (collectedCheckTick <= 0) {
            isDone = true;
        }

    }

    public boolean moveSwerve() {
        double xError = targetPose.getX() - swerveSubsystem.getPose().getX();
        double yError = targetPose.getY() - swerveSubsystem.getPose().getY();
        double turnError = (targetPose.getRotation().minus(swerveSubsystem.getRotation2d())).getRadians();

        SmartDashboard.putNumber("TurnError Ground Intake", turnError);

        // Calculate the angle and speed to move swerve to targetPose
        double angle = Math.atan2(yError, xError);
        double speed = (xError * xError + yError * yError) * 15 * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) > AutoConstants.kAutoMaxSpeedMetersPerSecond ? AutoConstants.kAutoMaxSpeedMetersPerSecond : (xError * xError + yError * yError) * 7;

        // Calculate xSpeed, ySpeed, and turnSpeed
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = Math.sin(angle) * speed;
        double turnSpeed = (Math.abs(turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005) < AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond) ? turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005 : AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond * Math.signum(turnError);


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

        if (Math.sqrt(xError * xError + yError * yError) < AutoConstants.kAutoToleranceMeters && Math.abs(turnError * 180 / Math.PI) < AutoConstants.kAutoToleranceDegrees) {
            return true;
        }

        return false;
    }

    public boolean isFinished() {
        return isDone;
    }
}
