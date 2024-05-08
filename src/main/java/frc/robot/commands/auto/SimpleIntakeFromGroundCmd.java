package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SimpleIntakeFromGroundCmd extends MobileCommand {

    private final IntakeSubsystem intakeSubsystem;

    private int collectedCheckTick = AutoConstants.GROUND_INTAKE_CHECK_TICKS;
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

    public boolean isFinished() {
        return isDone;
    }
}
