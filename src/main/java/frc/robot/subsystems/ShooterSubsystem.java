package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor_1;
    private final CANSparkMax shooterMotor_2;

    private final SlewRateLimiter shooterLimiter;

    private final Servo servo_1;
    private final Servo servo_2;

    public double limiterSetting = 0;
    public boolean flapState = false; // up = true, down = false
    public boolean allowedShoot = false;
    public int tick = 0;



    public ShooterSubsystem() {
        shooterMotor_1 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_1, MotorType.kBrushless);
        shooterMotor_2 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_2, MotorType.kBrushless);

        shooterLimiter = new SlewRateLimiter(1);

        servo_1 = new Servo(ShooterConstants.kShooterFlapServoId_1);
        servo_2 = new Servo(ShooterConstants.kShooterFlapServoId_2);

        this.flapUp();
    }



    public double getRollerSpeed() {
        return (Math.abs(shooterMotor_1.getAppliedOutput()) + Math.abs(shooterMotor_2.getAppliedOutput())) / 2;
    }

    public double getRollerGoal() {
        return limiterSetting;
    }

    public void spinIn() {
        limiterSetting = -ShooterConstants.kShooterIntakeSpeed;
    }

    public void speakerSpinOut() {
        limiterSetting = ShooterConstants.kShooterFlywheelSpeed;
    }

    public void setRollerSpeed(double speed) {
        limiterSetting = speed;
    }

    public void ampSpinOut() {
        limiterSetting = ShooterConstants.kShooterAmpSpeed;
    }



    @Override
    public void periodic() {
        tick++;
        double speed = shooterLimiter.calculate(limiterSetting);
        shooterMotor_1.set(speed);
        shooterMotor_2.set(-speed);
    }



    public void flapMove(double pos) {
        servo_1.setAngle(pos);
        servo_2.setAngle(180 - pos);
    }

    public void flapUp() {
        flapMove(ShooterConstants.kShooterFlapUpPos);
        flapState = true;
    }

    public void flapDown() {
        flapMove(40);
        flapState = false;
    }

    public void toggleFlap() {
        if (flapState) {
            flapDown();
        } else {
            flapUp();
        }
    }

    public void shooterStop() {
        limiterSetting = 0;
    }

    public void stop() {
        limiterSetting = 0;
        shooterMotor_1.set(0);
        shooterMotor_2.set(0);
    }

    // public void vibrateFlap() {
    //     if (tick % 10 < 5) {
    //         flapMove(ShooterConstants.kShooterFlapAmpPos + 10);
    //     } else {
    //         flapMove(ShooterConstants.kShooterFlapAmpPos);
    //     }
    // }
}
