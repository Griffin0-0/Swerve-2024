package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final CANSparkMax articulateMotor;
    private final SparkPIDController articulatePID;
    private final RelativeEncoder articulateEncoder;
    private final GenericEntry encoder;
    private final LEDSubsystem ledSubsystem;

    public boolean intakeOut = false;
    public boolean switchTemp = false;
    public double currentGoal = 0.0;


    public IntakeSubsystem(LEDSubsystem ledSubsystem) {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

        articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
        articulatePID = articulateMotor.getPIDController();
        articulateEncoder = articulateMotor.getEncoder();
        articulateEncoder.setPosition(0);

        articulatePID.setOutputRange(-IntakeConstants.kIntakeArticulateSpeed, IntakeConstants.kIntakeArticulateSpeed);
        articulatePID.setP(0.6);
        articulatePID.setI(0);
        articulatePID.setD(0);

        articulateEncoder.setPosition(0);

        encoder = Shuffleboard.getTab("Driver")
            .add("Encoder Pos", 0.0)
            .getEntry();

        this.ledSubsystem = ledSubsystem;
    }

    public void intakeDown() {
        currentGoal = IntakeConstants.kIntakeDesiredPos_out;    
    }
    public void intakeUp() {
        currentGoal = IntakeConstants.kIntakeDesiredPos_store;    
    }
    public void intakeAmp() {
        currentGoal = IntakeConstants.kIntakeDesiredPos_amp;
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void toggleIntake() {
        if (intakeOut) {
            intakeUp();
            intakeOut = false;
        } else if (!intakeOut) {
            intakeDown();
            intakeOut = true;
        }
    }

    public boolean atPoint() {
        return (Math.abs(getPosition() - currentGoal) < 0.5);
    }

    public boolean isDown() {
        if (intakeOut) {
            if (getPosition() < -30) {
                return true;
            }
        }
        return false;
    }

    public void spinIn() {
        runIntake(IntakeConstants.kIntakeMotorSpeed_ground);
    }

    public void spinOut() {
        runIntake(IntakeConstants.kIntakeMotorSpeed_out);
    }

    public void spinAmp() {
        runIntake(IntakeConstants.kIntakeMotorSpeed_amp);
    }

    public double getPosition() {
        return articulateEncoder.getPosition();
    }

    @Override
    public void periodic() {
        encoder.setDouble(getPosition());

        if (isDown()) {
            spinIn();
            switchTemp = true;
            ledSubsystem.setIntake();
        } else if (switchTemp) {
            stopIntake();
            switchTemp = false;
            ledSubsystem.setDefault();
        }

        articulatePID.setReference(currentGoal, ControlType.kPosition);
    }

    public void stop() {
        intakeMotor.set(0);
        articulateMotor.set(0);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }
}
