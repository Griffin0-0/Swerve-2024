package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SimpleFireAtSpeakerCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int currentShootTick = Constants.AutoConstants.SPEAKER_SHOT_CHECK_TICKS;
    private Pose2d[] blueSpeakerPositions = {
                                            new Pose2d(2.3,5.48,Rotation2d.fromDegrees(0)),
                                            new Pose2d(1.95,6.45,Rotation2d.fromDegrees(26)),
                                            new Pose2d(1.95,4.51,Rotation2d.fromDegrees(-26))
                                            };
    private Pose2d[] redSpeakerPositions = {
                                            new Pose2d(2.3,2.57,Rotation2d.fromDegrees(0)),
                                            new Pose2d(1.83,3.64,Rotation2d.fromDegrees(26)),
                                            new Pose2d(1.83,1.50,Rotation2d.fromDegrees(-26))
                                            };

    private Pose2d[] speakerPositions;
    private Pose2d targetPose;
    private boolean isDone = false;

    public SimpleFireAtSpeakerCmd(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(AutoConstants.MAX_ACCELERATION_UNITS_PER_SEC);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC);
        addRequirements(swerveSubsystem);

        if (swerveSubsystem.isAllianceBlue) {
            this.speakerPositions = blueSpeakerPositions;
        } else {
            this.speakerPositions = redSpeakerPositions;
        }
    }

    @Override
    public void initialize() {
        shooterSubsystem.speakerSpinOut();
        intakeSubsystem.intakeUp();
        shooterSubsystem.flapSpeaker();
        isDone = false;
        currentShootTick = Constants.AutoConstants.SPEAKER_SHOT_CHECK_TICKS; 
        // Find closest shooting point and set as targetPose
        double minDistance = Double.MAX_VALUE;
        for (Pose2d speakerPos : speakerPositions) {
            double distance = Math.sqrt(Math.pow(speakerPos.getX() - swerveSubsystem.getPose().getX(), 2) + Math.pow(speakerPos.getY() - swerveSubsystem.getPose().getY(), 2));
            if (distance < minDistance) {
                minDistance = distance;
                targetPose = speakerPos;
            }
        }
    }

    @Override
    public void execute() {

        // If close to targetPos, shoot
        if (moveSwerve() && shooterSubsystem.getRollerSpeed() > 0.99) {
            intakeSubsystem.runIntake(IntakeConstants.OUT_MOTOR_SPEED);
        } else {
            intakeSubsystem.stop();
        }

        // If shooter is done shooting, stop shooter and exit command
        if (shooterSubsystem.getRollerSpeed() > 0.99 && !intakeSubsystem.noteConfirmed && currentShootTick <= 0) {
            shooterSubsystem.shooterStop();
            isDone = true;
        } else if (shooterSubsystem.getRollerSpeed() > 0.99 && !intakeSubsystem.noteConfirmed && currentShootTick > 0) {
            currentShootTick--;
        }

    }

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

    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        intakeSubsystem.stop();
        shooterSubsystem.flapDefault();
    }
}
