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
    private double collectionDistance = 0.2;

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

        SmartDashboard.putString("ground Target Real Position", targetTranslation.toString());

        SmartDashboard.putString("ground difference", difference.toString());
        SmartDashboard.putString("swervePos", this.swerveSubsystem.pose.toString());

        new Rotation2d();
        // Calculate the angle between the note and swerve
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
        SmartDashboard.putNumber("Target Angle", angle.getDegrees());

        // Calculate the target position for swerve to move to
        this.targetPose = new Pose2d(targetTranslation.getX() - angle.getCos() * collectionDistance, targetTranslation.getY() - angle.getSin() * collectionDistance, angle);
    }

    @Override
    public void execute() {
        tick++;
        SmartDashboard.putNumber("Ground Ticks", tick);

        SmartDashboard.putString("ground Target Position", targetPose.toString());
        
        if (moveSwerve()) {
            // If swerve reached targetPose, start collecting note
            collectedCheckTick--;
        }

        // Once made sure swerve has collected note from ground, exit command
        if (collectedCheckTick <= AutoConstants.kAutoGroundIntakeCheckTicks / 4) {
            intakeSubsystem.stopIntake();
            intakeSubsystem.intakeUp();
        } else {
            intakeSubsystem.runIntake(IntakeConstants.kGroundIntakeMotorSpeed);
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

        SmartDashboard.putNumber("TurnError Ground Intake", turnError * 180 / Math.PI);

        // Calculate the angle and speed to move swerve to targetPose
        double angle = Math.atan2(yError, xError);
        double speed = (xError * xError + yError * yError) * 15 * (1 / AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond) > AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond ? AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond : (xError * xError + yError * yError) * 7;

        // Calculate xSpeed, ySpeed, and turnSpeed
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = Math.sin(angle) * speed;
        double turnSpeed = (Math.abs(turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005) < AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond) ? turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005 : AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond * Math.signum(turnError);


        xSpeed = Math.abs(xSpeed) > AutoConstants.kAutoMinSpeed ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.kAutoMinSpeed ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.kAutoMinTurnSpeedRadians ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("turnSpeed", turnSpeed);

        // xSpeed = 0;
        // ySpeed = 0;
        
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
