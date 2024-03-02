package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final CANSparkMax articulateMotor;
    private final SparkPIDController articulatePID;
    private final RelativeEncoder articulateEncoder;
    private final GenericEntry sb_encoder;
    private final LEDSubsystem ledSubsystem;
    
    private final Rev2mDistanceSensor distanceSensor;

    public String intakeState = "store";
    public boolean switchTemp = false;
    public double currentGoal = 0.0;
    public boolean noteConfirmed = false;
    public boolean useDistanceSensor = true;


    public IntakeSubsystem(LEDSubsystem ledSubsystem) {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

        articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
        articulatePID = articulateMotor.getPIDController();
        articulateEncoder = articulateMotor.getAlternateEncoder(100);
        articulatePID.setFeedbackDevice(articulateEncoder);
        articulateEncoder.setPosition(0);

        articulatePID.setOutputRange(-IntakeConstants.kIntakeArticulateSpeed, IntakeConstants.kIntakeArticulateSpeed);
        articulateMotor.setSmartCurrentLimit(30);
        articulatePID.setP(0.1); // Increase until oscillation
        articulatePID.setI(0); // Always leave zero 
        articulatePID.setD(1.8); // Once oscillation, increase to dampen

        articulateEncoder.setPosition(0);

        distanceSensor = new Rev2mDistanceSensor(Port.kMXP);
        distanceSensor.setAutomaticMode(true);


        sb_encoder = Shuffleboard.getTab("Driver")
            .add("Encoder Pos", 0.0)
            .withPosition(8, 4)
            .withSize(4, 1)
            .getEntry();

        this.ledSubsystem = ledSubsystem;
    }

    public void intakeDown() {
        // currentGoal = IntakeConstants.kIntakeDesiredPos_out;
        articulatePID.setReference(IntakeConstants.kIntakeDesiredPos_out, ControlType.kPosition);   
        intakeState = "out";
    }
    public void intakeUp() {
        // currentGoal = IntakeConstants.kIntakeDesiredPos_store;
        articulatePID.setReference(IntakeConstants.kIntakeDesiredPos_store, ControlType.kPosition);
        intakeState = "store";
    }
    public void intakeAmp() {
        // currentGoal = IntakeConstants.kIntakeDesiredPos_amp;
        articulatePID.setReference(IntakeConstants.kIntakeDesiredPos_amp, ControlType.kPosition);
        intakeState = "amp";
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void toggleIntake() {
        if (intakeState == "out") {
            intakeUp();
        } else if (intakeState == "store") {
            intakeDown();
        }
    }

    public boolean atPoint(double point, double tolerance) {
        return (Math.abs(getPosition() - point)) < tolerance;
    }

    public boolean isDown() {
        if (intakeState == "out") {
            if (getPosition() < -40) {
                return true;
            }
        }
        return false;
    }

    public void spinIn() {
        runIntake(IntakeConstants.kIntakeMotorSpeed_ground);
    }

    public void spinInSource() {
        runIntake(IntakeConstants.kIntakeMotorSpeed_source);
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
        sb_encoder.setDouble(getPosition());

        if (isDown()) {
            spinIn();
            switchTemp = true;
            ledSubsystem.setIntake();
        } else if (switchTemp && !isDown()) {
            stopIntake();
            switchTemp = false;
            ledSubsystem.setDefault();
        }

        if (useDistanceSensor) {
            noteConfirmed = (distanceSensor.getRange() < 10);
        } else {
            noteConfirmed = false;
        }

        if (noteConfirmed && isDown()) {
            intakeUp();
        }

        if (atPoint(IntakeConstants.kIntakeDesiredPos_amp, 0.5) && intakeState == "amp") {
            spinAmp();
        }
    }

    public void stop() {
        intakeMotor.set(0);
        articulateMotor.set(0);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }
}
