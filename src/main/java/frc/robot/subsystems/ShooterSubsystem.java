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
import frc.robot.Constants.IntakeConstants;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor_1;
    private final CANSparkMax shooterMotor_2;

    private final SlewRateLimiter shooterLimiter;

    private final Servo servo_1;
    private final Servo servo_2;

    public double setting = 0;
    
    // String shooterState = "idle";

    public Command sendSpinOut() {
        return Commands.startEnd(() -> spinOut(), () -> stop());
    }

    public Command sendSpinIn() {
        return Commands.startEnd(() -> spinIn(), () -> stop());
    }

    public Command sendStop() {
        return Commands.runOnce(() -> stop());
    }

    public Command sendAMPOut() {
        return Commands.startEnd(() -> AMPOut(), () -> stop());
    }

    public Command sendFlapUp() {
        return Commands.runOnce(() -> flapMove(ShooterConstants.kShooterFlapUpPos));
    }

    public Command sendFlapDown() {
        return Commands.runOnce(() -> flapMove(30));
    }

    // public Command sendShoot() {
    //     return Commands.runOnce(() -> shoot());
    // }

    public ShooterSubsystem() {
        shooterMotor_1 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_1, MotorType.kBrushless);
        shooterMotor_2 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_2, MotorType.kBrushless);

        shooterLimiter = new SlewRateLimiter(1);

        servo_1 = new Servo(ShooterConstants.kShooterFlapServoId_1);
        servo_2 = new Servo(ShooterConstants.kShooterFlapServoId_2);
        // intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

        // shooterMotor = new CANSparkMax(IntakeConstants.kIntakeEntryMotorId, MotorType.kBrushless);
    }

    public void spinOut() {
        setting = 1;
    }

    public void spinIn() {

    }
    @Override
    public void periodic() {
        double speed = shooterLimiter.calculate(setting);
        SmartDashboard.putNumber("speed", speed);

        shooterMotor_1.set(speed);
        shooterMotor_2.set(-speed);
    }

    public void flapMove(double pos) {
        servo_1.setAngle(pos);
        servo_2.setAngle(180 - pos);
    }

    // public void shoot() {
    //     // shooterMotor.set(0.5);
    // }

    public void AMPOut() {
        shooterMotor_1.set(ShooterConstants.kShooterAmpSpeed);
        shooterMotor_2.set(-ShooterConstants.kShooterAmpSpeed);
    }

    public void stop() {
        shooterMotor_1.set(0);
        shooterMotor_2.set(0);
        setting = 0;
        // intakeMotor.set(0);
        // shooterMotor.set(0);
    }
}
