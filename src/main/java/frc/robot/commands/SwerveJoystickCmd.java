package frc.robot.commands;

import java.util.function.Supplier;

// import javax.swing.text.Position;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> isRobotRelative, isBoost, isSlow;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> isRobotRelative, Supplier<Boolean> isBoost, Supplier<Boolean> isSlow) {
                
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.isRobotRelative = isRobotRelative;
        this.isBoost = isBoost;
        this.isSlow = isSlow;
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELEDRIVE_MAX_ACCELERATION_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELEDRIVE_MAX_ACCELERATION_UNITS_PER_SEC);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.TELEDRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.DEADBAND ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        if (isBoost.get()) {
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELEBOOST_DRIVE_MAX_SPEED_METERS_PER_SEC;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELEBOOST_DRIVE_MAX_SPEED_METERS_PER_SEC;
        } else if (isSlow.get()) {
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELESLOW_DRIVE_MAX_SPEED_METERS_PER_SEC;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELESLOW_DRIVE_MAX_SPEED_METERS_PER_SEC;
        } else {
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELEDRIVE_MAX_SPEED_METERS_PER_SEC;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELEDRIVE_MAX_SPEED_METERS_PER_SEC;
        }
        
        

        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.TELEDRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC;

        // 5. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        if (!isRobotRelative.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 6. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        
        // 7. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
