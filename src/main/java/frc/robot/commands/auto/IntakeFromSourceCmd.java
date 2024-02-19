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

public class IntakeFromSourceCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private int currentStopTick;
    private int startTick = -AutoConstants.kAutoStartCheckTicks;
    private int intookCheckTick = AutoConstants.kAutoSourceIntakeCheckTicks;
    private Pose2d targetPose;
    private int tick = 0;
    private Boolean isDone = false;

    private Pose2d intakeSourcePos;
    private Pose2d BlueIntakeSourcePos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
    private Pose2d RedIntakeSourcePos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
    
    public IntakeFromSourceCmd(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kAutoMaxAngularAccelerationUnitsPerSecond);

        if (!swerveSubsystem.isAllianceBlue) { // REMOVE NOT BEFORE COMPETITION
            this.intakeSourcePos = BlueIntakeSourcePos;
        } else {
            this.intakeSourcePos = RedIntakeSourcePos;
        }

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        tick++;
        SmartDashboard.putNumber("Amp Ticks", tick);

        shooterSubsystem.spinIn();
        shooterSubsystem.flapUp();

        targetPose = intakeSourcePos;
        
        if (moveSwerve()) {
            intakeSubsystem.runIntake(IntakeConstants.kIntakeOutMotorSpeed);
            intookCheckTick--;
        } else {
            intakeSubsystem.stopIntake();
        }

        if (intookCheckTick <= 0) {
            shooterSubsystem.stop();
            intakeSubsystem.stop();
            shooterSubsystem.flapDown();
            isDone = true;
        }
    }

    // Moves swerve to targetPose. Returns true when it reaches the position
    public boolean moveSwerve() {
        startTick++;
        SmartDashboard.putNumber("startTick", startTick);

        // Calculate the error between current position and targetPos
        double xError = targetPose.getX() - swerveSubsystem.getPose().getX();
        double yError = targetPose.getY() - swerveSubsystem.getPose().getY();
        double turnError = (targetPose.getRotation().minus(swerveSubsystem.getRotation2d())).getRadians();

        // Calculate the angle and speed to move swerve to targetPose
        double angle = Math.atan2(yError, xError);
        double speed = (xError * xError + yError * yError) * 15 * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) * (1 / AutoConstants.kAutoMaxSpeedMetersPerSecond) > AutoConstants.kAutoMaxSpeedMetersPerSecond ? AutoConstants.kAutoMaxSpeedMetersPerSecond : (xError * xError + yError * yError) * 7;

        // Calculate xSpeed, ySpeed, and turnSpeed
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = Math.sin(angle) * speed;
        double turnSpeed = (Math.abs(turnError * Math.sqrt(Math.abs(turnError * 40)) * -0.005) < AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond) ? turnError * Math.sqrt(Math.abs(turnError * 40)) * -0.005 : AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond * Math.signum(turnError);

        // Limit xSpeed, ySpeed, and turnSpeed to min speeds
        xSpeed = Math.abs(xSpeed) > AutoConstants.kAutoMinSpeed ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > AutoConstants.kAutoMinSpeed ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > AutoConstants.kAutoMinTurnSpeedRadians ? turnSpeed : 0.0;

        // Limit xSpeed, ySpeed, and turnSpeed to max acceleration
        xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.kAutoMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.kAutoMaxSpeedMetersPerSecond;
        turnSpeed = turningLimiter.calculate(turnSpeed) * AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("turnSpeed", turnSpeed);

        // Set the module states to move swerve to targetPose with field orientation
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
        
        // If swerve is close to targetPose, then return true
        if (Math.sqrt(xError * xError + yError * yError) < AutoConstants.kAutoToleranceMeters && Math.abs(turnError * 180 / Math.PI) < AutoConstants.kAutoToleranceDegrees) {
            return true; // Check if you can remove stop and start ticks now
        }

        // If swerve is not close to targetPose, then return false
        return false;
    }

    public boolean isFinished() {
        return isDone;
    }
}
