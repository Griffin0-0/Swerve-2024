package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor_1;
    private final CANSparkMax shooterMotor_2;

    private final SlewRateLimiter shooterLimiter;

    private final Servo servo_1;
    private final Servo servo_2;

    public double limiterSetting = 0;
    public boolean flapState = false; // up = true, down = false
    public boolean allowedShoot = false;


    public Command sendSpinOut() {
        return Commands.startEnd(() -> spinOut(), () -> stop());
    }

    public Command sendStop() {
        return Commands.runOnce(() -> stop());
    }

    public Command sendToggleFlap() {
        return Commands.runOnce(() -> toggleFlap());
    }
    


    public ShooterSubsystem() {
        shooterMotor_1 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_1, MotorType.kBrushless);
        shooterMotor_2 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_2, MotorType.kBrushless);

        shooterLimiter = new SlewRateLimiter(1);

        servo_1 = new Servo(ShooterConstants.kShooterFlapServoId_1);
        servo_2 = new Servo(ShooterConstants.kShooterFlapServoId_2);
    }



    public double getRollerSpeed() {
        return (shooterMotor_1.getAppliedOutput() + shooterMotor_2.getAppliedOutput()) / 2;
    }

    public double getRollerGoal() {
        return limiterSetting;
    }



    @Override
    public void periodic() {
        double speed = shooterLimiter.calculate(limiterSetting);
        shooterMotor_1.set(speed);
        shooterMotor_2.set(-speed);
    }



    public void spinOut() {
        limiterSetting = ShooterConstants.kShooterFlywheelSpeed;
    }

    public void flapMove(double pos) {
        servo_1.setAngle(pos);
        servo_2.setAngle(180 - pos);
    }

    public void toggleFlap() {
        if (flapState) {
            flapMove(30);
            flapState = false;
        } else {
            flapMove(ShooterConstants.kShooterFlapUpPos);
            flapState = true;
        }
    }

    public void stop() {
        shooterMotor_1.set(0);
        shooterMotor_2.set(0);
        limiterSetting = 0;
    }
}
