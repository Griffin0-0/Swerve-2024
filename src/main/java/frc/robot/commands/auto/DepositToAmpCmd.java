package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DepositToAmpCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int depositCheckTick = AutoConstants.DEPOSIT_CHECK_TICKS;
    private Pose2d targetPose;
    private Boolean isDone = false;
    private Boolean reachedFirstPoint = false;

    private Pose2d ampDepositPos;
    private Pose2d blueAmpDepositPos = new Pose2d(1.5,7.70, Rotation2d.fromDegrees(90));
    private Pose2d redAmpDepositPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
    
    public DepositToAmpCmd(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC);
        addRequirements(swerveSubsystem);

        if (swerveSubsystem.isAllianceBlue) {
            this.ampDepositPos = blueAmpDepositPos;
        } else {
            this.ampDepositPos = redAmpDepositPos;
        }
    }

    @Override
    public void initialize() {
        depositCheckTick = AutoConstants.DEPOSIT_CHECK_TICKS;
        isDone = false;
        reachedFirstPoint = false;

        new Rotation2d();
        targetPose = new Pose2d(ampDepositPos.getX() + 0.75, ampDepositPos.getY() - 0.75, Rotation2d.fromDegrees(45));
    }

    @Override
    public void execute() {

        SmartDashboard.putString("targetPose", targetPose.toString());

        Boolean swerveAtPos = moveSwerve();
        
        if (swerveAtPos && reachedFirstPoint) {
            intakeSubsystem.intakeAmp();
            depositCheckTick--;
        } else if (swerveAtPos) {
            targetPose = ampDepositPos;
            reachedFirstPoint = true;
        } else {
            intakeSubsystem.stopIntake();
        }

        if (depositCheckTick <= 0) {
            intakeSubsystem.stop();
            isDone = true;
        }
    }

    // Moves swerve to targetPose. Returns true when it reaches the position
    public boolean moveSwerve() {
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

        xSpeed = Math.abs(xSpeed) > AutoConstants.MIN_SPEED ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.MIN_SPEED ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.MIN_TURN_SPEED_RAD ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.MAX_SPEED_METERS_PER_SEC;
        ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.MAX_SPEED_METERS_PER_SEC;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if (Math.sqrt(xError * xError + yError * yError) < AutoConstants.TOLERANCE_METERS && Math.abs(turnError * 180 / Math.PI) < AutoConstants.TOLERANCE_DEGREES) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        intakeSubsystem.intakeUp();
    }

    public boolean isFinished() {
        return isDone;
    }
    
}
