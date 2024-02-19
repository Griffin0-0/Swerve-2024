package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
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
    // private int currentStopTick;
    // private int startTick = -AutoConstants.kAutoStartCheckTicks;
    private int currentShootTick = Constants.AutoConstants.kAutoSpeakerShotCheckTicks; 
    private double shootingDistance = 2.0;
    private Translation2d blueSpeakerPos = new Translation2d(0.1,5.45);
    private Translation2d redSpeakerPos = new Translation2d(0,0);
    private Translation2d speakerPos;
    private Pose2d targetPose;
    private boolean isDone = false;

    public FireAtSpeakerCmd(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAngularAccelerationUnitsPerSecond);

        if (swerveSubsystem.isAllianceBlue) {
            this.speakerPos = blueSpeakerPos;
        } else {
            this.speakerPos = redSpeakerPos;
        }

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        tick++;
        SmartDashboard.putNumber("Speaker Ticks", tick);

        // Calculate the difference between the speaker position and swerve's position
        Translation2d difference = new Translation2d(speakerPos.getX() - swerveSubsystem.getPose().getX(), speakerPos.getY() - swerveSubsystem.getPose().getY());

        new Rotation2d();
        // Calculate the angle between the speaker and swerve
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));

        // Calculate the target position for swerve to move to
        targetPose = new Pose2d(speakerPos.getX() - Math.cos(angle.getRadians()) * shootingDistance, speakerPos.getY() - Math.sin(angle.getRadians()) * shootingDistance, angle.minus(Rotation2d.fromDegrees(180)));

        // Spin up shooter
        shooterSubsystem.spinOut();

        // If close to targetPos, shoot
        if (moveSwerve()) {
            intakeSubsystem.runIntake(-IntakeConstants.kIntakeOutMotorSpeed);
            currentShootTick--;
        } else {
            intakeSubsystem.stop();
        }

        // If shooter is done shooting, stop shooter and exit command
        if (currentShootTick <= 0) {
            shooterSubsystem.stop();
            isDone = true;
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

        if (Math.sqrt(xError * xError + yError * yError) < AutoConstants.kAutoToleranceMeters && Math.abs(turnError * 180 / Math.PI) < AutoConstants.kAutoToleranceDegrees) {
            return true;
        }

        return false;
    }

    public boolean isFinished() {
        return isDone;
    }
}
