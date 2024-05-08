package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DepositToAmpCmd extends MobileCommand {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

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

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        intakeSubsystem.intakeUp();
    }

    public boolean isFinished() {
        return isDone;
    }
    
}
