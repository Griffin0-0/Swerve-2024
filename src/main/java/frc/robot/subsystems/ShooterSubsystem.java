package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor_1;
    private final CANSparkMax shooterMotor_2;

    private final SlewRateLimiter shooterLimiter;

    private final Servo servo_1;
    private final Servo servo_2;

    public double limiterSetting = 0;
    public int flapState = 0; // up = 1, down = 0, mid = 2
    public boolean allowedShoot = false;
    public int tick = 0;



    public ShooterSubsystem() {
        shooterMotor_1 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_1, MotorType.kBrushless);
        shooterMotor_2 = new CANSparkMax(ShooterConstants.kShooterSpinMotorId_2, MotorType.kBrushless);

        shooterMotor_1.setSmartCurrentLimit(30);
        shooterMotor_2.setSmartCurrentLimit(30);

        shooterLimiter = new SlewRateLimiter(1);

        servo_1 = new Servo(ShooterConstants.kShooterFlapServoId_1);
        servo_2 = new Servo(ShooterConstants.kShooterFlapServoId_2);

        this.flapDefault();
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

    public void flapDefault() {
        flapMove(ShooterConstants.kShooterFlapDefaultPos);
        flapState = 0;
    }

    public void flapAmp() {
        flapMove(ShooterConstants.kShooterFlapAmpPos);
        flapState = 1;
    }

    public void flapSpeaker() {
        flapMove(ShooterConstants.kShooterFlapSpeakerPos);
        flapState = 2;
    }

    public void cycleFlap() {
        if (flapState == 0) {
            flapAmp();
            flapState = 1;
        } else if (flapState == 1) {
            flapSpeaker();
            flapState = 2;
        } else if (flapState == 2) {
            flapDefault();
            flapState = 0;
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
}
