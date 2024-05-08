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
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleIntakeFromGroundCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int collectedCheckTick = AutoConstants.GROUND_INTAKE_CHECK_TICKS;
    private Pose2d targetPose;
    private Translation2d targetTranslation;
    private Boolean isDone = false;
    private double collectionDistance = 2;
    private int reachedFirstPoint = 0;
    private double currentSpeedLimit = AutoConstants.MAX_SPEED_METERS_PER_SEC;

    public SimpleIntakeFromGroundCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, Translation2d targetTranslation) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.targetTranslation = targetTranslation;
        this.xLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC);
        addRequirements(swerveSubsystem);
        }
        
    @Override
    public void initialize() {
        intakeSubsystem.intakeUp();
        intakeSubsystem.stopIntake();
        Pose2d approachPoint = new Pose2d(targetTranslation.getX() - collectionDistance, targetTranslation.getY(), Rotation2d.fromDegrees(0));

        targetPose = approachPoint;

        if (intakeSubsystem.noteConfirmed) {
            isDone = true;
        }
    }

    @Override
    public void execute() {
        boolean swerveMovedCondition = moveSwerve();
        
        // if swerve reached approach point, lower the intake
        if (swerveMovedCondition && reachedFirstPoint == 0) {
            reachedFirstPoint = 1;
            intakeSubsystem.intakeDown();
            intakeSubsystem.runIntake(IntakeConstants.GROUND_MOTOR_SPEED);
        // if swerve reached approach point and intake is down, move to target point
        } else if (swerveMovedCondition && reachedFirstPoint == 1 && intakeSubsystem.isDown()) {
            targetPose = new Pose2d(targetTranslation.getX() - 0.4, targetTranslation.getY(), Rotation2d.fromDegrees(0));
            currentSpeedLimit = AutoConstants.GROUND_INTAKE_MAX_SPEED_METERS_PER_SEC;
            reachedFirstPoint = 2;
        // if swerve reached target point or colour sensor detects note, stop intake and raise it
        } else if ((swerveMovedCondition && reachedFirstPoint == 2) || (intakeSubsystem.noteConfirmed && collectedCheckTick <= 0)) {
            intakeSubsystem.stopIntake();
            intakeSubsystem.intakeUp();
            reachedFirstPoint = 3;
        // if swerve reached target point and intake is up, set isDone to true
        } else if (reachedFirstPoint == 3 && intakeSubsystem.atPoint(IntakeConstants.STORE_DESIRED_POS, 2)) {
            isDone = true;
        }

        if (intakeSubsystem.noteConfirmed) {
            collectedCheckTick--;
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
        double turnSpeed = (Math.abs(turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005) < AutoConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC) ? turnError * 60 * Math.sqrt(Math.abs(turnError * 60)) * -0.005 : AutoConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * Math.signum(turnError);


        xSpeed = Math.abs(xSpeed) > AutoConstants.MIN_SPEED ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.MIN_SPEED ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.MIN_TURN_SPEED_RAD ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * currentSpeedLimit;
        ySpeed = yLimiter.calculate(ySpeed) * currentSpeedLimit;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if (Math.sqrt(xError * xError + yError * yError) < 0.12 && Math.abs(turnError * 180 / Math.PI) < AutoConstants.TOLERANCE_DEGREES) {
            return true;
        }

        return false;
    }

    public boolean isFinished() {
        return isDone;
    }
}
