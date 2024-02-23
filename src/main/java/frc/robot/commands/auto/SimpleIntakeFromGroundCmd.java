package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SimpleIntakeFromGroundCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int collectedCheckTick = AutoConstants.kAutoGroundIntakeCheckTicks;
    private Pose2d targetPose;
    private Translation2d targetTranslation;
    private Boolean isDone = false;
    private double collectionDistance = 2;
    private boolean reachedFirstPoint = false;
    private double currentSpeedLimit = AutoConstants.kAutoMaxSpeedMetersPerSecond;

    public SimpleIntakeFromGroundCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, Translation2d targetTranslation) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.targetTranslation = targetTranslation;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
        }
        
    @Override
    public void initialize() {
        Pose2d approachPoint = new Pose2d(targetTranslation.getX() - collectionDistance, targetTranslation.getY(), Rotation2d.fromDegrees(0));

        targetPose = approachPoint;
    }

    @Override
    public void execute() {
        boolean swerveMovedCondition = moveSwerve();
        
        if (swerveMovedCondition && reachedFirstPoint) {
            // If swerve reached targetPose, start collecting note
            collectedCheckTick--;
        } else if (swerveMovedCondition && !reachedFirstPoint) {
            reachedFirstPoint = true;
            targetPose = new Pose2d(targetTranslation.getX() - 0.5, targetTranslation.getY(), Rotation2d.fromDegrees(0));
            currentSpeedLimit = AutoConstants.kAutoGroundIntakingMaxSpeedMetersPerSecond;
        }

        // Once made sure swerve has collected note from ground, exit command
        if (collectedCheckTick < AutoConstants.kAutoGroundIntakeCheckTicks) {
            intakeSubsystem.stopIntake();
            intakeSubsystem.intakeUp();
        } else {
            intakeSubsystem.runIntake(IntakeConstants.kIntakeMotorSpeed_ground);
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

        // Calculate the angle and speed to move swerve to targetPose
        double angle = Math.atan2(yError, xError);
        double speed = (xError * xError + yError * yError) * 15 * (1 / currentSpeedLimit) * (1 / currentSpeedLimit) * (1 / currentSpeedLimit) > currentSpeedLimit ? currentSpeedLimit : (xError * xError + yError * yError) * 7;

        // Calculate xSpeed, ySpeed, and turnSpeed
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = Math.sin(angle) * speed;
        double turnSpeed = (Math.abs(turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005) < AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond) ? turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005 : AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond * Math.signum(turnError);


        xSpeed = Math.abs(xSpeed) > AutoConstants.kAutoMinSpeed ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.kAutoMinSpeed ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.kAutoMinTurnSpeedRadians ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * currentSpeedLimit;
        ySpeed = yLimiter.calculate(ySpeed) * currentSpeedLimit;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond;
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if (Math.sqrt(xError * xError + yError * yError) < 0.12 && Math.abs(turnError * 180 / Math.PI) < AutoConstants.kAutoToleranceDegrees) {
            return true;
        }

        return false;
    }

    public boolean isFinished() {
        return isDone;
    }
}
